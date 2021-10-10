#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/CommandLine.h"

#include "AddressSpace.h"
#include "Composer.h"
#include "ExecutionState.h"
#include "PForest.h"
#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprUtil.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Module/Cell.h"
#include "klee/Module/KModule.h"

#include <algorithm>
#include <cassert>
#include <stack>

using namespace klee;
using namespace llvm;

cl::OptionCategory
    ComposerCat("State-composition-related options",
                "These options affect the behavior of the composer.");

cl::opt<bool> ComposerDebug(
    "debug-composer", cl::init(false),
    cl::desc("Turn on expression composition debug trace (default=false)"),
    cl::cat(ComposerCat));

BidirectionalExecutor* Composer::executor = nullptr;

std::map<const ExecutionState *, ExprVisitor::visited_ty,
         ExecutionStateIDCompare>
    ComposeVisitor::globalVisited;

ExprVisitor::Action ComposeVisitor::visitRead(const ReadExpr &read) {
  ref<Expr> result = processRead(read);
  return faultyPtr.isNull() ? Action::changeTo(result) : Action::abort();
}

ExprVisitor::Action ComposeVisitor::visitSelect(const SelectExpr &select) {
  ref<Expr> result = processSelect(select);
  return faultyPtr.isNull() ? Action::changeTo(result) : Action::abort();
}

ref<Expr> ComposeVisitor::shareUpdates(ref<ObjectState> OS,
                                       const ReadExpr &re) {
  std::stack<ref<UpdateNode>> forward;

  for (auto it = re.updates.head; !it.isNull(); it = it->next) {
    forward.push(it);
  }

  while (!forward.empty()) {
    ref<UpdateNode> UNode = forward.top();
    forward.pop();
    ref<Expr> newIndex = visit(UNode->index);
    ref<Expr> newValue = visit(UNode->value);
    OS->write(newIndex, newValue);
  }

  ref<Expr> index = visit(re.index);
  return OS->read(index, re.getWidth());
}


std::map<const ExecutionState *, std::map<const MemoryObject *, ref<Expr>>,
         ExecutionStateIDCompare>
    Composer::globalReadCache;

std::map<const ExecutionState *, ExprHashMap<ref<Expr>>,
         ExecutionStateIDCompare>
    Composer::globalDerefCache;

bool Composer::tryRebuild(const ref<Expr> expr, ref<Expr> &res) {
  ComposeVisitor visitor(this, -S1->stackBalance);
  res = visitor.visit(expr);
  if(ComposerDebug) {
    errs() << "--<-------------------------------------->--\n";
    errs() << "Expression before rebuild:\n";
    expr->dump();
    errs() << "Expression after rebuild:\n";
    if(res.isNull()) errs() << "No expression\n";
    else res->dump();
  }
  res = visitor.faultyPtr.isNull() ? res : visitor.faultyPtr;
  return visitor.faultyPtr.isNull();
}

void Composer::compose(ExecutionState *S1, ExecutionState *S2,
                       ExecutionState *&result) {
  assert(S1 || S2);
  if (!S1 || S1->isEmpty()) {
    result = S2->copy();
    return;
  } else if (!S2 || S2->isEmpty()) {
    result = S1->copy();
    return;
  }

  TimingSolver *solver = executor->getSolver();
  assert(solver);
  Composer composer(S1, S2);

  ExecutionState *copy = S1->copy();
  executor->addState(*copy);
  executor->getProcessForest()->attach(S1->ptreeNode, copy, S1);
  composer.compose(copy, result);
}

bool Composer::tryRebuild(const ref<Expr> expr, ExecutionState *S1,
                          ref<Expr> &res) {
  Composer composer(S1, nullptr);
  return composer.tryRebuild(expr, res);
}

void Composer::compose(ExecutionState *state, ExecutionState *&result) {
  assert(S1 && S2 && state && !S1->isEmpty() && !S2->isEmpty());
  assert(state->pc == S2->initPC);
  if (ComposerDebug) {
    errs() << "COMPOSING DEBUG: Composing begins\n";    
  }

  std::vector<ref<Expr>> rconstraints;
  for (auto &constraint : S2->constraints) {
    ref<Expr> rebuildConstraint = constraint;

    bool success;
    for (auto sti = state->statesForRebuild.rbegin(),
              ste = state->statesForRebuild.rend();
         sti != ste; ++sti) {
      ExecutionState *st = *sti;
      success = Composer::tryRebuild(rebuildConstraint, st, rebuildConstraint);
      if (!success) {
        // TODO: образуется слишком много состояний с ошибками по сравнению с
        // forward-режимом
        executor->terminateStateOnOutOfBound(*state, rebuildConstraint);
        if (ComposerDebug) {
          errs() << "COMPOSING DEBUG: Composing ends, State out of bound\n";
        }
        return;
      }
      rebuildConstraint =
          ConstraintManager::simplifyExpr(st->constraints, rebuildConstraint);
    }
    rebuildConstraint =
        ConstraintManager::simplifyExpr(state->constraints, rebuildConstraint);

    if (isa<ConstantExpr>(rebuildConstraint)) {
      if (cast<ConstantExpr>(rebuildConstraint)->isTrue())
        continue;
      else {
        executor->silentRemove(*state);
        if (ComposerDebug) {
          errs() << "COMPOSING DEBUG: Composing ends, state unreachable\n";
        }
        return;
      }
    }

    rconstraints.push_back(rebuildConstraint);
  }

  ref<Expr> check = ConstantExpr::create(true, Expr::Bool);
  for (auto &rc : rconstraints) {
    check = AndExpr::create(check, rc);
  }
  bool mayBeTrue;
  bool success = executor->getSolver()->mayBeTrue(
      state->constraints, check, mayBeTrue, state->queryMetaData);
  assert(success && "FIXME: Unhandled solver failure");
  if (mayBeTrue) {
    for (auto &rc : rconstraints) {
      executor->addConstraint(*state, rc);
    }
  } else {
    // TODO: нужна нормальная обработка недостижимого состояния
    if (ComposerDebug) {
      errs() << "COMPOSING DEBUG: Composing ends, state unreachable\n";
    }
    executor->silentRemove(*state);
    return;
  }

  assert(state->stack.back().kf == S2->stack.front().kf &&
         "can not compose states from different functions");

  if (S2->stack.size() > 1) {
    for (unsigned n = 1; n < S2->stack.size(); ++n) {
      state->stack.push_back(S2->stack[n]);
    }
  }

  state->prevPC = S2->prevPC;
  state->pc = S2->pc;
  state->incomingBBIndex = S2->incomingBBIndex;
  state->steppedInstructions =
      state->steppedInstructions + S2->steppedInstructions;
  state->depth = state->depth + S2->depth;
  state->coveredNew = S2->coveredNew;
  state->level.insert(S2->level.begin(), S2->level.end());
  state->multilevel.insert(S2->multilevel.begin(), S2->multilevel.end());
  state->transitionLevel.insert(S2->transitionLevel.begin(),
                                S2->transitionLevel.end());
  state->statesForRebuild.push_back(S2);

  result = state;
  if (ComposerDebug) {
    errs() << "COMPOSING DEBUG: Composing ends\n";
  }
}
