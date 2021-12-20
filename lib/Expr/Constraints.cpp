//===-- Constraints.cpp ---------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Expr/Constraints.h"

#include "klee/Expr/ExprVisitor.h"
#include "klee/Module/KModule.h"
#include "klee/Support/OptionCategories.h"

#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"

#include <map>
#include <stack>

using namespace klee;

namespace {
llvm::cl::opt<bool> RewriteEqualities(
    "rewrite-equalities",
    llvm::cl::desc("Rewrite existing constraints when an equality with a "
                   "constant is added (default=true)"),
    llvm::cl::init(true),
    llvm::cl::cat(SolvingCat));
} // namespace

class ExprReplaceVisitor : public ExprVisitor {
private:
  ref<Expr> src, dst;

public:
  ExprReplaceVisitor(const ref<Expr> &_src, const ref<Expr> &_dst)
      : src(_src), dst(_dst) {}

  Action visitExpr(const Expr &e) override {
    if (e == *src) {
      return Action::changeTo(dst);
    }
    return Action::doChildren();
  }

  Action visitExprPost(const Expr &e) override {
    if (e == *src) {
      return Action::changeTo(dst);
    }
    return Action::doChildren();
  }
};

class ExprReplaceVisitor2 : public ExprVisitor {
private:
   ExprHashMap<ref<Expr>> &replacements;

   explicit ExprReplaceVisitor2(
      ExprHashMap<ref<Expr>> &_replacements,
      visited_ty *visitorHash)
      : ExprVisitor(true), replacements(_replacements) {
     visited = visitorHash;
   }

public:
  explicit ExprReplaceVisitor2(
      ExprHashMap<ref<Expr>> &_replacements)
      : ExprVisitor(true), replacements(_replacements) {}

  Action visitExprPost(const Expr &e) override {
    auto it = replacements.find(ref<Expr>(const_cast<Expr *>(&e)));
    if (it!=replacements.end()) {
      return Action::changeTo(it->second);
    }
    return Action::doChildren();
  }

  ref<Expr> processSelect(const SelectExpr& sexpr) {
    ref<Expr> cond = visit(sexpr.cond);
    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(cond)) {
      return CE->isTrue() ? visit(sexpr.trueExpr) : visit(sexpr.falseExpr);
    }

    visited_ty currentVisited = *visited;

    ref<Expr> er;
    if (const EqExpr *ee = dyn_cast<EqExpr>(sexpr.cond)) {
      if (isa<ConstantExpr>(ee->left)) {
        replacements.insert(std::make_pair(ee->right, ee->left));
        er = ee->right;
      } else {
        replacements.insert(std::make_pair(sexpr.cond, ConstantExpr::alloc(1, Expr::Bool)));
        er = sexpr.cond;
      }
    } else {
      replacements.insert(std::make_pair(sexpr.cond, ConstantExpr::alloc(1, Expr::Bool)));
      er = sexpr.cond;
    }
    ref<Expr> trueExpr =
        ExprReplaceVisitor2(replacements, &currentVisited).visit(sexpr.trueExpr);
    replacements.erase(er);

    replacements.insert(std::make_pair(sexpr.cond, ConstantExpr::alloc(0, Expr::Bool)));
    ref<Expr> falseExpr =
        ExprReplaceVisitor2(replacements, &currentVisited).visit(sexpr.falseExpr);
    replacements.erase(sexpr.cond);

    ref<Expr> seres = SelectExpr::create(cond, trueExpr, falseExpr);
    Action res = visitExprPost(*seres.get());
    if (res.kind == Action::ChangeTo)
      seres = res.argument;
    return seres;
  }

  ref<Expr> processRead(const ReadExpr& re) {
    UpdateList updates = UpdateList(re.updates.root, 0);
    std::stack<ref<UpdateNode>> forward;

    for(auto it = re.updates.head;
             !it.isNull();
             it = it->next) {
      forward.push( it );
    }

    while(!forward.empty()) {
      ref<UpdateNode> UNode = forward.top();
      forward.pop();
      ref<Expr> newIndex = visit(UNode->index);
      ref<Expr> newValue = visit(UNode->value);
      updates.extend(newIndex, newValue);
    }

    ref<Expr> index = visit(re.index);
    ref<Expr> reres = ReadExpr::create(updates, index);
    Action res = visitExprPost(*reres.get());
    if (res.kind == Action::ChangeTo) {
      reres = res.argument;
    }
    return reres;
  }

  Action visitSelect(const SelectExpr& sexpr) override {
    return Action::changeTo(processSelect(sexpr));
  }

  Action visitRead(const ReadExpr& re) override {
    return Action::changeTo(processRead(re));
  }
};

bool ConstraintManager::rewriteConstraints(ExprVisitor &visitor, KInstruction *location, bool *sat) {
  ConstraintSet old;
  bool changed = false;

  std::swap(constraints, old);
  for (auto &ce : old) {
    KInstruction *loc = old.get_location(ce);
    ref<Expr> e = visitor.visit(ce);

    if (e!=ce) {
      addConstraintInternal(e, location, sat); // enable further reductions
      changed = true;
    } else {
      constraints.push_back(ce, loc);
    }
  }

  return changed;
}

ref<Expr> ConstraintManager::simplifyExpr(const ConstraintSet &constraints,
                                          const ref<Expr> &e) {

  if (isa<ConstantExpr>(e))
    return e;

  ExprHashMap<ref<Expr>> equalities;

  for (auto &constraint : constraints) {
    if (const EqExpr *ee = dyn_cast<EqExpr>(constraint)) {
      if (isa<ConstantExpr>(ee->left)) {
        equalities.insert(std::make_pair(ee->right,
                                         ee->left));
      } else {
        equalities.insert(
            std::make_pair(constraint, ConstantExpr::alloc(1, Expr::Bool)));
      }
    } else {
      equalities.insert(
          std::make_pair(constraint, ConstantExpr::alloc(1, Expr::Bool)));
    }
  }

  return ExprReplaceVisitor2(equalities).visit(e);
}

void ConstraintManager::addConstraintInternal(const ref<Expr> &e, KInstruction *location, bool *sat) {
  // rewrite any known equalities and split Ands into different conjuncts

  switch (e->getKind()) {
  case Expr::Constant:
    if(!cast<ConstantExpr>(e)->isTrue()) {
      if(sat) *sat = false;
      else assert(false && "attempt to add invalid (false) constraint");
    }
    break;

    // split to enable finer grained independence and other optimizations
  case Expr::And: {
    BinaryExpr *be = cast<BinaryExpr>(e);
    addConstraintInternal(be->left, location, sat);
    addConstraintInternal(be->right, location, sat);
    break;
  }

  case Expr::Eq: {
    if (RewriteEqualities) {
      // XXX: should profile the effects of this and the overhead.
      // traversing the constraints looking for equalities is hardly the
      // slowest thing we do, but it is probably nicer to have a
      // ConstraintSet ADT which efficiently remembers obvious patterns
      // (byte-constant comparison).
      BinaryExpr *be = cast<BinaryExpr>(e);
      if (isa<ConstantExpr>(be->left)) {
        ExprReplaceVisitor visitor(be->right, be->left);
        rewriteConstraints(visitor, location, sat);
      }
    }
    constraints.push_back(e, location);
    break;
  }

  default:
    constraints.push_back(e, location);
    break;
  }
}

void ConstraintManager::addConstraint(const ref<Expr> &e, KInstruction *location, bool *sat) {
  ref<Expr> simplified = simplifyExpr(constraints, e);
  addConstraintInternal(simplified, location, sat);
}

ConstraintManager::ConstraintManager(ConstraintSet &_constraints)
    : constraints(_constraints) {}

bool ConstraintSet::empty() const { return constraints.empty(); }

klee::ConstraintSet::constraint_iterator ConstraintSet::begin() const {
  return constraints.begin();
}

klee::ConstraintSet::constraint_iterator ConstraintSet::end() const {
  return constraints.end();
}

size_t ConstraintSet::size() const noexcept { return constraints.size(); }

void ConstraintSet::push_back(const ref<Expr> &e, KInstruction *loc) {
  constraints.push_back(e);
  mapToLocations.insert({e, loc});
}

KInstruction *ConstraintSet::get_location(const ref<Expr> &e) const {
  if (mapToLocations.count(e))
    return mapToLocations.at(e);
  else
    return nullptr;
}
