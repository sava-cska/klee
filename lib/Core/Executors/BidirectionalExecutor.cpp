#include "BidirectionalExecutor.h"

#include "../MemoryManager.h"
#include "../PForest.h"
#include "../Searcher.h"
#include "../StatsTracker.h"
#include "../Composer.h"

#include <stack>


using namespace llvm;

namespace klee {

extern cl::OptionCategory TerminationCat;

cl::opt<unsigned long long> MaxCycles(
    "max-cycles",
    cl::desc("stop execution after visiting some basic block this amount of times (default=1)."),
    cl::init(1),
    cl::cat(TerminationCat));

BidirectionalExecutor::BidirectionalExecutor(LLVMContext &ctx,
                                     const InterpreterOptions &opts,
                                     InterpreterHandler *ih)
    : BaseExecutor(ctx, opts, ih) {
  Composer::executor = this;
}


void BidirectionalExecutor::addCompletedResult(ExecutionState &state) {
  results[state.getInitPCBlock()]
      .completedStates[state.getPrevPCBlock()]
      .insert(&state);
}

void BidirectionalExecutor::addErroneousResult(ExecutionState &state) {
  results[state.getInitPCBlock()]
      .erroneousStates[state.getPrevPCBlock()]
      .insert(&state);
}

void BidirectionalExecutor::addHistoryResult(ExecutionState &state) {
  results[state.getInitPCBlock()].history[state.getPrevPCBlock()].insert(
      state.level.begin(), state.level.end());
  results[state.getInitPCBlock()]
      .transitionHistory[state.getPrevPCBlock()]
      .insert(state.transitionLevel.begin(), state.transitionLevel.end());
}

void BidirectionalExecutor::addTargetable(ExecutionState &state) {
  assert(state.isIsolated());
  targetableStates[state.getInitPCBlock()->getParent()].insert(&state);
}

void BidirectionalExecutor::removeTargetable(ExecutionState &state) {
  assert(state.isIsolated());
  std::unordered_set<ExecutionState *> &sts =
      targetableStates[state.getInitPCBlock()->getParent()];
  std::unordered_set<ExecutionState *>::iterator it = sts.find(&state);
  if (it != sts.end()) {
    sts.erase(it);
  }
}

bool BidirectionalExecutor::isTargetable(ExecutionState &state) {
  assert(state.isIsolated());
  std::unordered_set<ExecutionState *> &sts =
      targetableStates[state.getInitPCBlock()->getParent()];
  std::unordered_set<ExecutionState *>::iterator it = sts.find(&state);
  return it != sts.end();
}

void BidirectionalExecutor::initializeRoot(ExecutionState &state, KBlock *kb) {
  ExecutionState *root = state.withKBlock(kb);
  prepareSymbolicArgs(*root, kb->parent);
  if (statsTracker)
    statsTracker->framePushed(*root, 0);
  processForest->addRoot(root);
  addedStates.push_back(root);
}

void BidirectionalExecutor::runWithTarget(ExecutionState &state, KBlock *target) {
  if (pathWriter)
    state.pathOS = pathWriter->open();
  if (symPathWriter)
    state.symPathOS = symPathWriter->open();

  if (statsTracker)
    statsTracker->framePushed(state, 0);

  processForest = std::make_unique<PForest>();
  processForest->addRoot(&state);
  targetedRun(state, target);
  processForest = nullptr;

  if (statsTracker)
    statsTracker->done();
}

bool BidirectionalExecutor::tryBoundedExecuteStep(ExecutionState &state,
                                           unsigned bound) {
  KInstruction *prevKI = state.prevPC;

  if (prevKI->inst->isTerminator()) {
    addHistoryResult(state);
    if (state.multilevel.count(state.getPCBlock()) > bound) {
      return false;
    }
  }

  executeStep(state);
  return true;
}

void BidirectionalExecutor::isolatedExecuteStep(ExecutionState &state) {
  assert(state.isIsolated());
  KInstruction *ki = state.pc;

  if (state.isCriticalPC() || isa<ReturnInst>(ki->inst)) {
    addTargetable(state);
    addCompletedResult(state);
    silentRemove(state);
    if (isa<ReturnInst>(ki->inst)) {
      executeStep(state);
    } else
      updateStates(nullptr);
    return;
  }

  if (state.redundant) {
    pauseRedundantState(state);
    updateStates(nullptr);
    return;
  }

  executeStep(state);
}

// Was named tryCoverStep
bool BidirectionalExecutor::tryExploreStep(ExecutionState &state,
                                    ExecutionState &initialState) {
  KFunction *kf = kmodule->functionMap[state.getPCBlock()->getParent()];
  if (state.isIntegrated() && state.isCriticalPC()) {
    if (results.find(state.getPCBlock()) == results.end()) {
      initializeRoot(initialState, kf->blockMap[state.getPCBlock()]);
      updateStates(nullptr);
    } else {
      if (!state.target &&
          state.multilevel.count(state.getPCBlock()) > MaxCycles - 1)
        return false;
      else
        composeStep(state);
    }
  } else if (state.isIsolated()) {
    isolatedExecuteStep(state);
  } else {
    executeStep(state);
  }
  return true;
}

// Absent in Bidir
void BidirectionalExecutor::executeReturn(ExecutionState &state, KInstruction *ki) {
  assert(isa<ReturnInst>(ki->inst));
  ReturnInst *ri = cast<ReturnInst>(ki->inst);
  KInstIterator kcaller = state.stack.back().caller;
  Instruction *caller = kcaller ? kcaller->inst : 0;
  bool isVoidReturn = (ri->getNumOperands() == 0);
  ref<Expr> result = ConstantExpr::alloc(0, Expr::Bool);

  if (!isVoidReturn) {
    result = eval(ki, 0, state).value;
  }
  if (state.stack.size() <= 1) {
    assert(!caller && "caller set on initial stack frame");
    --state.stackBalance;
    bindLocal(ki, state, result);
    state.pc = state.prevPC;
  } else {
    state.stack.pop_back();

    addHistoryResult(state);

    if (InvokeInst *ii = dyn_cast<InvokeInst>(caller)) {
      transferToBasicBlock(ii->getNormalDest(), caller->getParent(), state);
    } else {
      state.pc = kcaller;
      ++state.pc;
      state.addLevel(state.getPrevPCBlock(), state.getPCBlock());
      BranchInst *bi = cast<BranchInst>(state.pc->inst);
      assert(bi && bi->isUnconditional());
      KInstruction *ki = state.pc;
      stepInstruction(state);
      executeInstruction(state, ki);
    }
  }
}

void BidirectionalExecutor::composeStep(ExecutionState &lstate) {
  assert(lstate.isIntegrated());
  std::vector<ExecutionState *> result;
  for (auto &blockstate : results[lstate.getPCBlock()].completedStates) {
    for (auto rstate : blockstate.second) {
      ExecutionState *currentResult = nullptr;
      Composer::compose(&lstate, const_cast<ExecutionState *>(rstate),
                        currentResult);
      if (currentResult) {
        if (isTargetable(*rstate)) {
          removeTargetable(*rstate);
        }
        if (isa<ReturnInst>(currentResult->pc->inst) &&
            currentResult->stack.size() > 1) {
          executeReturn(*currentResult, currentResult->pc);
        }
        addHistoryResult(*currentResult);

        result.push_back(currentResult);
      }
    }
  }
  for (auto &blockstate : results[lstate.getPCBlock()].redundantStates) {
    for (auto rstate : blockstate.second) {
      ExecutionState *currentResult = nullptr;
      Composer::compose(&lstate, const_cast<ExecutionState *>(rstate),
                        currentResult);
      if (currentResult) {
        removeTargetable(*rstate);
        addHistoryResult(*currentResult);
        result.push_back(currentResult);
      }
    }
  }
  removedStates.push_back(&lstate);
  updateStates(nullptr);
}

void BidirectionalExecutor::targetedRun(ExecutionState &initialState,
                                    KBlock *target) {
  // Delay init till now so that ticks don't accrue during optimization and
  // such.
  timers.reset();
  states.insert(&initialState);

  auto targetedSearcher = new TargetedSearcher(target);
  searcher.reset(targetedSearcher);
  searcher->update(0, {states.begin(), states.end()}, {});

  // main interpreter loop
  KInstruction *terminator = (target ? target->getLastInstruction() : nullptr);
  while (!searcher->empty() && !haltExecution) {
    ExecutionState &state = searcher->selectState();
    KInstruction *ki = state.pc;

    executeStep(state);

    if (ki == terminator) {
      terminateStateEarly(state, "The target has been found!");
      updateStates(&state);
      break;
    }
  }

  searcher.reset();

  doDumpStates();
  haltExecution = false;
}

KBlock *BidirectionalExecutor::calculateCoverTarget(ExecutionState &state) {
  BasicBlock *initialBlock = state.getInitPCBlock();
  VisitedBlock &history = results[initialBlock].history;
  VisitedTransition &transitionHistory =
      results[initialBlock].transitionHistory;
  BasicBlock *bb = state.getPCBlock();
  KFunction *kf = kmodule->functionMap[bb->getParent()];
  KBlock *kb = kf->blockMap[bb];
  KBlock *nearestBlock = nullptr;
  unsigned int minDistance = -1;
  unsigned int sfNum = 0;
  bool newCov = false;
  for (auto sfi = state.stack.rbegin(), sfe = state.stack.rend(); sfi != sfe;
       sfi++, sfNum++) {
    kf = sfi->kf;
    std::map<KBlock *, unsigned int> &kbd = kf->getDistance(kb);
    for (auto &tstate : targetableStates[kf->function]) {
      KBlock *currKB = kf->blockMap[tstate->getPCBlock()];
      if (kbd.find(currKB) != kbd.end() && kbd[currKB] > 0 &&
          kbd[currKB] < minDistance) {
        nearestBlock = currKB;
        minDistance = kbd[currKB];
      }
    }

    if (nearestBlock) {
      return nearestBlock;
    }

    for (auto currKB : kf->finalKBlocks) {
      if (kbd.find(currKB) != kbd.end() && kbd[currKB] > 0 &&
          kbd[currKB] < minDistance) {
        if (history[currKB->basicBlock].size() != 0) {
          std::vector<Transition> diff;
          if (!newCov) {
            std::set<Transition> left(state.transitionLevel.begin(),
                                      state.transitionLevel.end());
            std::set<Transition> right(
                transitionHistory[currKB->basicBlock].begin(),
                transitionHistory[currKB->basicBlock].end());
            std::set_difference(left.begin(), left.end(), right.begin(),
                                right.end(), std::inserter(diff, diff.begin()));
          }

          if (diff.empty()) {
            continue;
          }
        } else {
          newCov = true;
        }
        nearestBlock = currKB;
        minDistance = kbd[currKB];
      }
    }

    if (nearestBlock) {
      return nearestBlock;
    }

    if (sfi->caller) {
      kb = sfi->caller->parent;
    }
  }

  return nearestBlock;
}

KBlock *BidirectionalExecutor::calculateTarget(ExecutionState &state) {
  BasicBlock *initialBlock = state.getInitPCBlock();
  VisitedBlock &history = results[initialBlock].history;
  BasicBlock *bb = state.getPCBlock();
  KFunction *kf = kmodule->functionMap[bb->getParent()];
  KBlock *kb = kf->blockMap[bb];
  KBlock *nearestBlock = nullptr;
  unsigned int minDistance = -1;
  unsigned int sfNum = 0;
  bool newCov = false;
  for (auto sfi = state.stack.rbegin(), sfe = state.stack.rend(); sfi != sfe;
       sfi++, sfNum++) {
    kf = sfi->kf;

    for (auto &kbd : kf->getDistance(kb)) {
      KBlock *target = kbd.first;
      unsigned distance = kbd.second;
      if ((sfNum > 0 || distance > 0) && distance < minDistance) {
        if (history[target->basicBlock].size() != 0) {
          std::vector<BasicBlock *> diff;
          if (!newCov)
            std::set_difference(state.level.begin(), state.level.end(),
                                history[target->basicBlock].begin(),
                                history[target->basicBlock].end(),
                                std::inserter(diff, diff.begin()));
          if (diff.empty()) {
            continue;
          }
        } else
          newCov = true;
        nearestBlock = target;
        minDistance = distance;
      }
    }

    if (nearestBlock) {
      return nearestBlock;
    }

    if (sfi->caller) {
      kb = sfi->caller->parent;
    }
  }
  return nearestBlock;
}

void BidirectionalExecutor::guidedRun(ExecutionState &initialState) {
  // Delay init till now so that ticks don't accrue during optimization and
  // such.
  timers.reset();

  states.clear();
  addedStates.clear();
  removedStates.clear();
  states.insert(&initialState);

  if (usingSeeds) {
    seed(initialState);
  }

  searcher = std::make_unique<GuidedSearcher>(constructUserSearcher(*this));

  std::vector<ExecutionState *> newStates(states.begin(), states.end());
  searcher->update(0, newStates, std::vector<ExecutionState *>());
  // main interpreter loop
  while (!states.empty() && !haltExecution) {
    while (!searcher->empty() && !haltExecution) {
      ExecutionState &state = searcher->selectState();
      if (state.target)
        executeStep(state);
      else if (!tryBoundedExecuteStep(state, MaxCycles - 1)) {
        KBlock *target = calculateTarget(state);
        if (target) {
          state.target = target;
        } else {
          pauseState(state);
          updateStates(nullptr);
        }
      }
    }

    if (searcher->empty())
      haltExecution = true;
  }

  searcher.reset();

  doDumpStates();
  haltExecution = false;
}

void BidirectionalExecutor::addState(ExecutionState &state) {
  addedStates.push_back(&state);
}

void BidirectionalExecutor::run(ExecutionState &state) {
  std::unique_ptr<ExecutionState> initialState(state.copy());
  initialState->stack.clear();
  initialState->stackBalance = 0;
  initialState->isolated = true;
  state.statesForRebuild.push_back(state.copy());

  // Delay init till now so that ticks don't accrue during optimization and such.
  timers.reset();

  states.insert(&state);

  if (usingSeeds)
    seed(state);

  ExecutionStateIsolationRank rank;
  searcher = std::make_unique<BinaryRankedSearcher>(
        rank,
        constructUserSearcher(*this),
        std::make_unique<GuidedSearcher>(constructUserSearcher(*this)));

  std::vector<ExecutionState *> newStates(states.begin(), states.end());
  searcher->update(0, newStates, std::vector<ExecutionState *>());

  // main interpreter loop
  while (!searcher->empty() && !haltExecution) {
    ExecutionState &es = searcher->selectState();
    if (!tryExploreStep(es, *initialState)) {
      KBlock *target = calculateCoverTarget(es);
      if (target) {
        es.target = target;
        updateStates(&es);
      } else {
        pauseState(es);
        updateStates(nullptr);
      }
    }
  }

  searcher.reset();

  doDumpStates();
  results.clear();
  haltExecution = false;
}

void BidirectionalExecutor::pauseState(ExecutionState &state) {
  results[state.getInitPCBlock()].pausedStates[state.getPCBlock()].insert(&state);
  removedStates.push_back(&state);
}

void BidirectionalExecutor::pauseRedundantState(ExecutionState &state) {
  results[state.getInitPCBlock()].redundantStates[state.getPCBlock()].insert(&state);
  removedStates.push_back(&state);
}

void BidirectionalExecutor::unpauseState(ExecutionState &state) {
  ExecutedInterval &pausedStates = results[state.getInitPCBlock()].pausedStates;

  pausedStates[state.getPCBlock()].erase(&state);
  if (pausedStates[state.getPCBlock()].empty())
    pausedStates.erase(state.getPCBlock());
  addedStates.push_back(&state);
}

void BidirectionalExecutor::actionBeforeStateTerminating(ExecutionState &state, TerminateReason reason) {
  switch (reason) {
    case TerminateReason::Model       : addCompletedResult(state); break;
    case TerminateReason::ReportError : addErroneousResult(state); break;
    default                           : assert(false && "unreachable");
  }
  addHistoryResult(state);
}

/// TODO: remove?
void BidirectionalExecutor::runMainWithTarget(Function *mainFn,
                                 BasicBlock *target,
                                 int argc,
                                 char **argv,
                                 char **envp) {
  ExecutionState *state = formState(mainFn, argc, argv, envp);
  bindModuleConstants();
  KFunction *kf = kmodule->functionMap[mainFn];
  KBlock *kb = kmodule->functionMap[target->getParent()]->blockMap[target];
  runWithTarget(*state, kb);
  // hack to clear memory objects
  memory = std::make_unique<MemoryManager>(nullptr);
  clearGlobal();
}

bool ComposeVisitor::tryDeref(ref<Expr> ptr, unsigned size, ref<Expr> &result) {
  ExecutionState *S1 = caller->S1;
  ExprHashMap<ref<Expr>> &derefCache = Composer::globalDerefCache[S1];
  auto cachedDeref = derefCache.find(ptr);
  if (cachedDeref != derefCache.end()) {
    result = derefCache[ptr];
    return true;
  }

  ResolutionList rl;
  Solver::Validity res = Solver::False;
  S1->addressSpace.resolve(*S1, caller->executor->getSolver(), ptr, rl, true);

  if (rl.empty() && isa<ConstantExpr>(ptr)) {
    faultyPtr = ptr;
    return false;
  }

  std::vector<std::pair<ref<Expr>, ref<Expr>>> results;
  ref<Expr> check = ConstantExpr::create(true, Expr::Bool);
  for(auto op = rl.begin(), e = rl.end(); op != e; op++) {
    const MemoryObject *mo = op->first;
    const ObjectState *os = op->second;

    ref<Expr> inBounds = mo->getBoundsCheckPointer(ptr, size);

    time::Span timeout = caller->executor->getSolverTimeout();
    TimingSolver *solver = caller->executor->getSolver();
    solver->setTimeout(timeout);
    solver->evaluate(S1->constraints, AndExpr::create(check, inBounds), res, S1->queryMetaData);
    solver->setTimeout(time::Span());

    if (res == Solver::True) {
      ref<Expr> trueCase = op->second->read(op->first->getOffsetExpr(ptr), 8 * op->second->size);
      results.push_back({
        ConstantExpr::create(true, Expr::Bool), SExtExpr::create(trueCase, 8 * size)});
      break;
    } else if (res == Solver::False) {
      continue;
    } else {
      check = AndExpr::create(check, Expr::createIsZero(inBounds));
      ref<Expr> trueCase = op->second->read(op->first->getOffsetExpr(ptr), 8 * op->second->size);
      results.push_back({inBounds, SExtExpr::create(trueCase, 8 * size)});
    }
  }

  if (!isa<ConstantExpr>(ptr) && res != Solver::True) {
    ObjectPair p = caller->executor->transparentLazyInstantiateVariable(
      *S1, ptr, nullptr, size);
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

ref<Expr> ComposeVisitor::processRead(const ReadExpr &re) {
  ref<Expr> refRe = new ReadExpr(re);
  ExecutionState *S1 = caller->S1;
  assert(S1 && "no context passed");

   std::map<const MemoryObject *, ref<Expr>> &readCache = Composer::globalReadCache[S1];

  ref<ObjectState> OS;
  if(re.updates.root->isForeign ||
     re.updates.root->isConstantArray()) {
    OS = new ObjectState(re.updates.root, caller->executor->getMemoryManager());
    ref<Expr> res = shareUpdates(OS, re);
    return res;
  }

  const MemoryObject *object = re.updates.root->binding ? re.updates.root->binding->getObject() : nullptr;
  if(!object) {
    return refRe;
  }

  OS = new ObjectState(object);

  if(object->isLazyInstantiated()) {
    auto cachedLIexpr = readCache.find(object);
    if(cachedLIexpr != readCache.end()) {
      ref<Expr> res = cachedLIexpr->second;
      OS->write(0, res);
      res = shareUpdates(OS, re);
      return res;
    }

    auto LI = visit(object->lazyInstantiatedSource);

    ref<Expr> result;
    if (tryDeref(LI, OS->size, result)) {
      readCache[object] = result;
      OS->write(0, result);
      result = shareUpdates(OS, re);
      return result;
    } else {
      return refRe;
    }
  }

  auto cachedLIexpr = readCache.find(object);
  if(cachedLIexpr != readCache.end()) {
    ref<Expr> res = cachedLIexpr->second;
    OS->write(0, res);
    res = shareUpdates(OS, re);
    return res;
  }

  ref<Expr> prevVal;

  if((re.updates.root->index >= 0 && diffLevel > 0) ||
    (re.updates.root->index > 0 && diffLevel < 0)) {
    prevVal = reindexRead(re);
    readCache[object] = prevVal;
    OS->write(0, prevVal);
    ref<Expr> res = shareUpdates(OS, re);
    return res;
  }

  const llvm::Value *allocSite = object->allocSite;
  if(!allocSite)
    return new ReadExpr(re);

  KFunction *kf1 = S1->stack.back().kf;
  KBlock *pckb1 = kf1->blockMap[S1->getPCBlock()];

  if(isa<Argument>(allocSite)) {
    const Argument *arg = cast<Argument>(allocSite);
    const Function *f   = arg->getParent();
    const KFunction *kf = caller->executor->getKFunction(f);
    const unsigned argN = arg->getArgNo();
    prevVal =
      kf->function == kf1->function ?
        caller->executor->getArgumentCell(*S1, kf, argN).value : nullptr;
  } else if(isa<Instruction>(allocSite)) {
    const Instruction *inst = cast<Instruction>(allocSite);
    const KInstruction *ki  = caller->executor->getKInst(const_cast<Instruction*>(inst));
    bool isFinalPCKb1 =
      std::find(kf1->finalKBlocks.begin(), kf1->finalKBlocks.end(), pckb1) != kf1->finalKBlocks.end();
    if (isa<PHINode>(inst))
      prevVal = caller->executor->eval(ki, S1->incomingBBIndex, *S1, false).value;
    else if ((isa<CallInst>(inst) || isa<InvokeInst>(inst))) {
      Function *f =
        isa<CallInst>(inst) ?
          dyn_cast<CallInst>(inst)->getCalledFunction() :
          dyn_cast<InvokeInst>(inst)->getCalledFunction();
      prevVal =
        isFinalPCKb1 && f == kf1->function ?
          caller->executor->getDestCell(*S1, S1->prevPC).value : nullptr;
    } else {
      const Instruction *inst = cast<Instruction>(allocSite);
      const Function *f = inst->getParent()->getParent();
      const KFunction *kf = caller->executor->getKFunction(f);
      prevVal =
        kf->function == kf1->function ?
          caller->executor->getDestCell(*S1, ki).value : nullptr;
    }
  } else {
    return new ReadExpr(re);
  }

  if(prevVal.isNull()) {
    return new ReadExpr(re);
  }

  readCache[object] = prevVal;
  OS->write(0, prevVal);
  ref<Expr> res = shareUpdates(OS, re);
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

ref<Expr> ComposeVisitor::reindexRead(const ReadExpr & read) {
  int reindex = read.updates.root->index + diffLevel;
  ref<Expr> indexExpr = read.index;
  const MemoryObject *mo = read.updates.root->binding ? read.updates.root->binding->getObject() : nullptr;
  assert(mo && mo->isLocal);
  const Array *root = caller->executor->getArrayManager()->CreateArray(read.updates.root, reindex);
  if (mo && !root->binding) {
    ref<Expr> liSource = mo->lazyInstantiatedSource;
    const MemoryObject *reindexMO = caller->executor->getMemoryManager()->allocate(
      mo->size, mo->isLocal, mo->isGlobal, mo->allocSite, /*allocationAlignment=*/8, liSource
    );

    ObjectState *os = new ObjectState(reindexMO, root);

    const_cast<Array*>(root)->binding = os;
  }
  ref<Expr> res = root->binding->read(0, 8 * root->binding->size);
  return res;
}
  
} // namespace klee
