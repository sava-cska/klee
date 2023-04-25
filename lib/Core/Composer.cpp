#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/CommandLine.h"

#include "AddressSpace.h"
#include "Composer.h"
#include "ExecutionState.h"
#include "PForest.h"
#include "klee/Expr/ArrayExprVisitor.h"
#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprUtil.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Module/Cell.h"
#include "klee/Module/KModule.h"

#include <algorithm>
#include <cassert>
#include <optional>
#include <stack>

using namespace klee;
using namespace llvm;

Executor* Composer::executor = nullptr;

std::map<const ExecutionState *, ExprVisitor::visited_ty,
         ExecutionStateIDCompare>
    ComposeVisitor::globalVisited;

ExprVisitor::Action ComposeVisitor::visitRead(const ReadExpr &read) {
  ref<Expr> result = processRead(read);
  return faultyPtr.isNull() ? Action::changeTo(result) : Action::abort();
}

ExprVisitor::Action ComposeVisitor::visitConcat(const ConcatExpr &concat) {
  ReadExpr *base = ArrayExprHelper::hasOrderedReads(concat);
  if (base) {
    ref<Expr> result = processOrderedRead(concat, *base);
    return faultyPtr.isNull() ? Action::changeTo(result) : Action::abort();
  } else {
    return Action::doChildren();
  }
}

ExprVisitor::Action ComposeVisitor::visitSelect(const SelectExpr &select) {
  ref<Expr> result = processSelect(select);
  return faultyPtr.isNull() ? Action::changeTo(result) : Action::abort();
}

void ComposeVisitor::shareUpdates(ref<ObjectState> os, const ReadExpr &re) {
  std::stack<ref<UpdateNode>> forward;

  for (auto it = re.updates.head; !it.isNull(); it = it->next) {
    forward.push(it);
  }

  while (!forward.empty()) {
    ref<UpdateNode> UNode = forward.top();
    forward.pop();
    ref<Expr> newIndex = visit(UNode->index);
    ref<Expr> newValue = visit(UNode->value);
    os->write(newIndex, newValue);
  }
}

std::map<const ExecutionState *, std::map<const MemoryObject *, ref<Expr>>,
         ExecutionStateIDCompare>
    Composer::globalReadCache;

std::map<const ExecutionState *, ExprHashMap<ref<Expr>>,
         ExecutionStateIDCompare>
    Composer::globalDerefCache;

uint64_t Composer::id = 0;

Composer::~Composer() {
  globalReadCache.erase(&copy);
  globalDerefCache.erase(&copy);
  delete &copy;
}

bool Composer::tryRebuild(const ref<Expr> expr, ref<Expr> &res) {
  ComposeVisitor visitor(*this, -copy.stackBalance, composeID);
  res = visitor.visit(expr);
  res = visitor.faultyPtr.isNull() ? res : visitor.faultyPtr;
  return visitor.faultyPtr.isNull();
}

bool Composer::tryRebuild(const ref<Expr> expr, ExecutionState &state, ref<Expr> &res) {
  Composer composer(state);
  bool success = composer.tryRebuild(expr, res);
  return success;
}

bool Composer::tryRebuild(const klee::ProofObligation *pobAtStateLocation,
                          const ProofObligation &old, ExecutionState &state,
                          ProofObligation &rebuilt,
                          Conflict::core_ty &conflictCore,
                          ExprHashMap<ref<Expr>> &rebuildMap) {
  bool success = true;
  Composer composer(state);
  if (pobAtStateLocation != nullptr) {
    Lemma *lemma = pobAtStateLocation->initialLemma;
    assert(lemma);
    if (!lemma->constraints.empty()) {
      auto it = lemma->constraints.begin();
      ref<Expr> lemmaStatement = *it;
      it++;
      while (it != lemma->constraints.end()) {
        lemmaStatement = OrExpr::create(lemmaStatement, *it);
        it++;
      }
      composer.copy.addConstraint(lemmaStatement, nullptr);
      llvm::errs() << "Dobavochka:\n";
      llvm::errs() << lemmaStatement << '\n';
    }
  }

  for (auto &constraint : old.condition) {
    auto loc = old.condition.getLocation(constraint);
    ref<Expr> rebuiltConstraint;
    success = composer.tryRebuild(constraint, rebuiltConstraint);
    bool mayBeTrue = true;
    std::vector<ref<Expr>> unsatCore;
    time::Span timeout = executor->getSolverTimeout();
    executor->getSolver()->setTimeout(timeout);
    if (success) {
      rebuildMap[rebuiltConstraint] = constraint;
      success = executor->getSolver()->mayBeTrue(
          composer.copy.constraints, rebuiltConstraint, mayBeTrue,
          composer.copy.queryMetaData, true);
      executor->getSolver()->popUnsatCore(unsatCore);
      if (success && !mayBeTrue &&
          // confclitCore needs for summarizing and should be non empty for
          // that. It means that unsatCore sould be non empty or rebuilt
          // constraint should be constant.
          (unsatCore.size() || isa<ConstantExpr>(rebuiltConstraint))) {

        if (!isa<ConstantExpr>(rebuiltConstraint)) {
          std::vector<ref<Expr>>::iterator ie =
              std::find(unsatCore.begin(), unsatCore.end(), rebuiltConstraint);
          assert(ie != unsatCore.end());
          unsatCore.erase(ie);
        }

        Executor::makeConflictCore(composer.copy, unsatCore, rebuiltConstraint,
                                   loc, conflictCore);
      }
    }
    if (success && mayBeTrue) {
      composer.copy.addConstraint(rebuiltConstraint, loc, &success);
    } else {
      success = false;
      break;
    }
  }
  rebuilt.condition = composer.copy.constraintInfos;
  std::vector<Symbolic> sourced;
  executor->extractSourcedSymbolics(composer.copy, sourced);
  rebuilt.sourcedSymbolics.insert(rebuilt.sourcedSymbolics.end(),
                                  sourced.begin(), sourced.end());
  rebuilt.path = concat(state.path, old.path);
  return success;
}

bool ComposeVisitor::tryDeref(ref<Expr> ptr, unsigned size, ref<Expr> &result) {
  ExecutionState &state = caller.copy;
  ExprHashMap<ref<Expr>> &derefCache = Composer::globalDerefCache[&state];
  auto cachedDeref = derefCache.find(ptr);
  if (cachedDeref != derefCache.end()) {
    result = derefCache[ptr];
    return true;
  }

  ResolutionList rl;
  Solver::Validity res = Solver::False;
  state.addressSpace.resolve(state, caller.executor->getSolver(), ptr, rl, true, true);

  if (rl.empty() && isa<ConstantExpr>(ptr)) {
    faultyPtr = ptr;
    return false;
  }

  std::vector<std::pair<ref<Expr>, ref<Expr>>> results;
  ref<Expr> check = ConstantExpr::create(true, Expr::Bool);
  for(auto op = rl.begin(), e = rl.end(); op != e; op++) {
    const MemoryObject *mo = op->first;

    ref<Expr> inBounds = mo->getBoundsCheckPointer(ptr, size);

    time::Span timeout = caller.executor->getSolverTimeout();
    TimingSolver *solver = caller.executor->getSolver();
    solver->setTimeout(timeout);
    solver->evaluate(state.constraints, AndExpr::create(check, inBounds), res, state.queryMetaData);
    solver->setTimeout(time::Span());

    if (res == Solver::True) {
      ref<Expr> trueCase = op->second->read(op->first->getOffsetExpr(ptr), 8 * size);
      results.push_back({
        ConstantExpr::create(true, Expr::Bool), SExtExpr::create(trueCase, 8 * size)});
      break;
    } else if (res == Solver::False) {
      continue;
    } else {
      check = AndExpr::create(check, Expr::createIsZero(inBounds));
      ref<Expr> trueCase = op->second->read(op->first->getOffsetExpr(ptr), 8 * size);
      results.push_back({inBounds, SExtExpr::create(trueCase, 8 * size)});
    }
  }

  if (!isa<ConstantExpr>(ptr) && res != Solver::True) {
    ObjectPair p = caller.executor->transparentLazyInitializeVariable(
      state, ptr, nullptr, size);
    ref<Expr> trueCase = p.second->read(0, 8 * p.second->size);
    results.push_back({ConstantExpr::create(true, Expr::Bool),
                        SExtExpr::create(trueCase, 8 * size)});
  }

  assert(!results.empty());
  if (results.size() == 1) {
    result = results.back().second;
  } else {
    result = results.back().second;
    results.pop_back();
    for (auto cri = results.rbegin(), cre = results.rend(); cri != cre; ++cri) {
      result = SelectExpr::create(cri->first, cri->second, result);
    }
  }

  derefCache[ptr] = result;
  return true;
}


ref<Expr> ComposeVisitor::processObject(const MemoryObject *object, const Array *array) {
  ExecutionState &state = caller.copy;
  std::map<const MemoryObject *, ref<Expr>> &readCache = Composer::globalReadCache[&state];

  if(!object) {
    return nullptr;
  }

  auto cachedRead = readCache.find(object);
  if(cachedRead != readCache.end()) {
    ref<Expr> res = cachedRead->second;
    return res;
  }

  if (object->isLazyInitialized()) {
    auto LI = visit(object->lazyInitializedSource);

    ref<Expr> result;
    if (tryDeref(LI, object->size, result)) {
      readCache[object] = result;
      return result;
    } else {
      return nullptr;
    }
  } else {
    ref<Expr> prevVal;

    if ((array->index >= 0 && diffLevel > 0) ||
        (array->index > 0 && array->index + diffLevel > 0)) {
      prevVal = reindexArray(array);
      readCache[object] = prevVal;
      return prevVal;
    }

    const llvm::Value *allocSite = object->allocSite;
    if (!allocSite)
      return nullptr;

    if (array->index == -1) {
      assert(isa<CallInst>(allocSite) || isa<InvokeInst>(allocSite));
      const Instruction *inst = cast<Instruction>(allocSite);
      const KInstruction *ki  = caller.executor->getKInst(const_cast<Instruction *>(inst));
      KFunction *kf = ki->parent->parent;

      if (state.stack.empty()) {
#if LLVM_VERSION_CODE >= LLVM_VERSION(8, 0)
        const CallBase &cs = cast<CallBase>(*allocSite);
        Value *fp = cs.getCalledOperand();
#else
        CallSite cs(firstit);
        Value *fp = cs.getCalledValue();
#endif
        Function *calledf = getTargetFunction(fp);
        KFunction *lastkf = state.pc->parent->parent;
        KBlock *pckb = lastkf->blockMap[state.getPCBlock()];
        bool isFinalPCKB = std::find(lastkf->finalKBlocks.begin(),
                                    lastkf->finalKBlocks.end(),
                                    pckb) != lastkf->finalKBlocks.end();
        assert(isFinalPCKB && calledf == lastkf->function);
        prevVal = state.returnValue;
      } else {
        StackFrame &frame = state.stack.back();
        KFunction *framekf = frame.kf;
        assert(kf->function == framekf->function);
        prevVal = caller.executor->getDestCell(frame, ki).value;
        assert(prevVal);
      }
    } else {
      assert(!state.stack.empty());
      StackFrame &frame = state.stack.at(state.stack.size() - array->index - 1);
      KFunction *framekf = frame.kf;

      if (isa<Argument>(allocSite)) {
        const Argument *arg = cast<Argument>(allocSite);
        const Function *f   = arg->getParent();
        const KFunction *kf = caller.executor->getKFunction(f);
        const unsigned argN = arg->getArgNo();
        assert(kf->function == framekf->function);
        prevVal = caller.executor->readArgument(state, frame, kf, argN);
      } else if(isa<Instruction>(allocSite)) {
        const Instruction *inst = cast<Instruction>(allocSite);
        const KInstruction *ki  = caller.executor->getKInst(const_cast<Instruction *>(inst));
        if (isa<PHINode>(inst)) {
          assert(framekf->function == inst->getParent()->getParent());
          if (inst->getParent() == state.getPCBlock())
            prevVal = caller.executor->eval(ki, state.incomingBBIndex, state, frame).value;
          else
            prevVal = caller.executor->readDest(state, frame, ki);
        }
        else if ((isa<CallInst>(inst) || isa<InvokeInst>(inst))) {
          KFunction *kf = ki->parent->parent;
          assert(kf->function == framekf->function);
          prevVal = caller.executor->readDest(state, frame, ki);
        } else {
          const Instruction *inst = cast<Instruction>(allocSite);
          const Function *f = inst->getParent()->getParent();
          const KFunction *kf = caller.executor->getKFunction(f);
          assert(kf->function == framekf->function);
          prevVal = caller.executor->readDest(state, frame, ki);
        }
      }
    }

    assert(prevVal);

    readCache[object] = prevVal;
    return prevVal;
  }
}

ref<Expr> ComposeVisitor::processRead(const ReadExpr &re) {
  ref<Expr> index = visit(re.index);
  ref<ObjectState> os;
  if (re.updates.root->isExternal ||
     re.updates.root->isConstantArray()) {
    const Array *root = re.updates.root;
    if (root->isExternal) {
      root = caller.executor->arrayManager.CreateArray(
          root->name + "<" + std::to_string(id) + ">", root->size, true, ref<Expr>());
    }
    os = new ObjectState(root, caller.executor->getMemoryManager());
    shareUpdates(os, re);
    return os->read(index, re.getWidth());
  }

  const MemoryObject *object = re.updates.root->binding;
  ref<Expr> res = processObject(object, re.updates.root);
  if (!res.isNull()) {
    os = new ObjectState(object);
    os->write(0, res);
    shareUpdates(os, re);
    res = os->read(index, re.getWidth());
  }
  return res;
}


ref<Expr> ComposeVisitor::processOrderedRead(const ConcatExpr &ce, const ReadExpr &base) {
  ref<Expr> index = visit(base.index);
  ref<ObjectState> os;
  if (base.updates.root->isExternal ||
    base.updates.root->isConstantArray()) {
    const Array *root = base.updates.root;
    if (root->isExternal) {
      root = caller.executor->arrayManager.CreateArray(
          root->name + "<" + std::to_string(id) + ">", root->size, true,
          ref<Expr>());
    }
    os = new ObjectState(root, caller.executor->getMemoryManager());
    shareUpdates(os, base);
    ref<Expr> res = os->read(index, ce.getWidth());
    return res;
  }

  const MemoryObject *object = base.updates.root->binding;
  ref<Expr> res = processObject(object, base.updates.root);

  if (!res.isNull()) {
    os = new ObjectState(object);
    os->write(0, res);
    shareUpdates(os, base);
    res = os->read(index, ce.getWidth());
  }
  return res;
}

ref<Expr> ComposeVisitor::processSelect(const SelectExpr &sexpr) {
  ref<Expr> cond = visit(sexpr.cond);
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(cond)) {
    return CE->isTrue() ? visit(sexpr.trueExpr) : visit(sexpr.falseExpr);
  }
  ref<Expr> trueExpr = visit(sexpr.trueExpr);
  ref<Expr> falseExpr = visit(sexpr.falseExpr);
  ref<Expr> result = SelectExpr::create(cond, trueExpr, falseExpr);
  return result;
}

ref<Expr> ComposeVisitor::reindexArray(const Array *array) {
  int reindex = array->index + diffLevel;
  const MemoryObject *mo = array->binding;
  assert(mo && mo->isLocal);
  assert(array->liSource.isNull());
  const Array *root = caller.executor->getArrayManager()->CreateArray(array, reindex);
  const ObjectState *os = nullptr;
  ExecutionState &state = caller.copy;
  if (!root->binding) {
    ref<Expr> liSource = mo->lazyInitializedSource;
    const MemoryObject *reindexMO = caller.executor->getMemoryManager()->allocateTransparent(
      mo->size, mo->isLocal, mo->isGlobal, mo->allocSite, /*allocationAlignment=*/8, liSource
    );
    os = caller.executor->bindSymbolicInState(state, reindexMO, false, root);
    const_cast<Array*>(root)->binding = reindexMO;
  } else {
    os = state.addressSpace.findObject(root->binding);
    if(!os) {
      os = caller.executor->bindSymbolicInState(state, root->binding, false, root);
    }
  }
  assert(os);
  ref<Expr> res = os->read(0, 8 * root->binding->size);
  return res;
}
