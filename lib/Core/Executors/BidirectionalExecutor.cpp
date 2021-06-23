//===-- BidirectionalExecutor.cpp ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

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

BidirectionalExecutor *Composer::executor = nullptr;

BidirectionalExecutor::BidirectionalExecutor(LLVMContext &ctx, const InterpreterOptions &opts,
                   InterpreterHandler *ih)
    : BaseExecutor(ctx, opts, ih)
{
  Composer::executor = this;
}

void BidirectionalExecutor::addCompletedResult(ExecutionState &state) {
  results[state.getInitPCBlock()].completedStates[state.getPrevPCBlock()].insert(&state);
}

void BidirectionalExecutor::addErroneousResult(ExecutionState &state) {
  results[state.getInitPCBlock()].erroneousStates[state.getPrevPCBlock()].insert(&state);
}

void BidirectionalExecutor::addHistoryResult(ExecutionState &state) {
  results[state.getInitPCBlock()].history[state.getPrevPCBlock()].insert(state.level.begin(), state.level.end());
}

void BidirectionalExecutor::addTargetable(ExecutionState &state) {
  assert(state.isIsolated());
  targetableStates[state.getInitPCBlock()->getParent()].insert(&state);
}

void BidirectionalExecutor::removeTargetable(ExecutionState &state) {
  assert(state.isIsolated());
  std::unordered_set<ExecutionState *> &sts = targetableStates[state.getInitPCBlock()->getParent()];
  std::unordered_set<ExecutionState*>::iterator it = sts.find(&state);
  if (it!=sts.end())
    sts.erase(it);
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

bool BidirectionalExecutor::tryBoundedExecuteStep(ExecutionState &state, unsigned bound) {
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

  if (state.isCriticalPC() ||
      isa<ReturnInst>(ki->inst) || isa<CallInst>(ki->inst) || isa<InvokeInst>(ki->inst)) {
    addTargetable(state);
    addCompletedResult(state);
    silentRemove(state);
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

bool BidirectionalExecutor::tryCoverStep(ExecutionState &state, ExecutionState &initialState) {
  KFunction *kf = kmodule->functionMap[state.getPCBlock()->getParent()];
  KInstruction *ki = state.pc;
  if (state.isIntegrated() && state.isCriticalPC()) {
    if (results.find(state.getPCBlock()) == results.end()) {
      initializeRoot(initialState, kf->blockMap[state.getPCBlock()]);
      updateStates(nullptr);
    } else {
      if (!state.target && state.multilevel.count(state.getPCBlock()) > MaxCycles - 1)
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

void BidirectionalExecutor::backwardStep(ExecutionState &state, ExecutionState &pobState) {
  assert(state.pc == pobState.initPC);
  ref<Expr> rebuildCond = ConstantExpr::create(true, Expr::Bool);
  for (auto &constraint : pobState.constraints) {
    ref<Expr> rebuildCond = AndExpr::create(rebuildCond, constraint);
  }
  for (auto &st : state.statesForRebuild) {
    rebuildCond = Composer::rebuild(rebuildCond, st);
  }
  bool mayBeTrue;
  if (!solver->mayBeTrue(state.constraints, rebuildCond, mayBeTrue,
                         state.queryMetaData)) {
    return;
  }
  if (mayBeTrue) {
    // TODO: добавить новый pob с rebuildCond
  } else {
    // TODO: заблокировать state для query
  }
}

/// TODO: remove?
void BidirectionalExecutor::composeStep(ExecutionState &lstate) {
  assert(lstate.isIntegrated());
  if (lstate.pc->inst->isTerminator()) {
    executeStep(lstate);
    return;
  }
  std::vector<ExecutionState *> result;
  for (auto &blockstate : results[lstate.getPCBlock()].completedStates) {
    for (auto rstate : blockstate.second) {
      ExecutionState *currentResult = nullptr;
      Composer::compose(&lstate, const_cast<ExecutionState*>(rstate), currentResult);
      if (currentResult) {
        removeTargetable(*rstate);
        result.push_back(currentResult);
      }
    }
  }
  for (auto &blockstate : results[lstate.getPCBlock()].redundantStates) {
    for (auto rstate : blockstate.second) {
      ExecutionState *currentResult = nullptr;
      Composer::compose(&lstate, const_cast<ExecutionState*>(rstate), currentResult);
      if (currentResult) {
        removeTargetable(*rstate);
        result.push_back(currentResult);
      }
    }
  }
  removedStates.push_back(&lstate);
  addedStates.insert(addedStates.end(), result.begin(), result.end());
  updateStates(nullptr);
}

/// TODO: update?
void BidirectionalExecutor::targetedRun(ExecutionState &initialState, KBlock *target) {
  // Delay init till now so that ticks don't accrue during optimization and such.
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
  BasicBlock *bb = state.getPCBlock();
  KFunction *kf = kmodule->functionMap[bb->getParent()];
  KBlock *kb = kf->blockMap[bb];
  KBlock *nearestBlock = nullptr;
  unsigned int minDistance = -1;
  unsigned int sfNum = 0;
  bool newCov = false;
  for (auto sfi = state.stack.rbegin(), sfe = state.stack.rend(); sfi != sfe; sfi++, sfNum++) {
    kf = sfi->kf;
    std::map<KBlock*, unsigned int> &kbd = kf->getDistance(kb);
    for (auto &tstate : targetableStates[kf->function]) {
      KBlock *currKB = kf->blockMap[tstate->getInitPCBlock()];
      unsigned distance = kbd[currKB];
      if (kbd.find(currKB) != kbd.end() && distance < minDistance) {
        nearestBlock = currKB;
        minDistance = distance;
      }
    }
    for (auto currKB : kf->finalKBlocks) {
      unsigned distance = kbd[currKB];
      if (kbd.find(currKB) != kbd.end() && distance < minDistance) {
        if (history[currKB->basicBlock].size() != 0) {
          std::vector<BasicBlock*> diff;
          if (!newCov) {
            std::set<BasicBlock*> left(state.level.begin(), state.level.end());
            std::set<BasicBlock*> right(history[currKB->basicBlock].begin(), history[currKB->basicBlock].end());
            std::set_difference(left.begin(), left.end(),
                                right.begin(), right.end(),
                                std::inserter(diff, diff.begin()));
          }

          if (diff.empty()) {
            continue;
          }
        } else
          newCov = true;
      nearestBlock = currKB;
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
  for (auto sfi = state.stack.rbegin(), sfe = state.stack.rend(); sfi != sfe; sfi++, sfNum++) {
    kf = sfi->kf;

    for (auto &kbd : kf->getDistance(kb)) {
      KBlock *target = kbd.first;
      unsigned distance = kbd.second;
      if ((sfNum >0 || distance > 0) && distance < minDistance) {
        if (history[target->basicBlock].size() != 0) {
          std::vector<BasicBlock*> diff;
          if (!newCov)
            std::set_difference(state.level.begin(), state.level.end(),
                                history[target->basicBlock].begin(), history[target->basicBlock].end(),
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

/// TODO: update?
void BidirectionalExecutor::guidedRun(ExecutionState &initialState) {
  // Delay init till now so that ticks don't accrue during optimization and such.
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
  initialState->isolated = true;

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
    if (!tryCoverStep(es, *initialState)) {
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
  haltExecution = false;
}

void BidirectionalExecutor::silentRemove(ExecutionState &state) {
  removedStates.push_back(&state);
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

//-----Composer-----//
typedef std::pair<ref<const MemoryObject>, const Array *> symb;

void Composer::compose(ExecutionState *state, ExecutionState *&result) {
  assert(S1 && S2 && state && !S1->isEmpty() && !S2->isEmpty());
  assert(state->pc == S2->initPC);
  executor->getProcessForest()->attach(S1->ptreeNode, state, const_cast<ExecutionState *>(S1));
  for (auto &constraint : S2->constraints) {
    ref<Expr> rebuildConstraint = constraint;
    for (auto &st : state->statesForRebuild) {
      rebuildConstraint = Composer::rebuild(rebuildConstraint, st);
    }
    rebuildConstraint = Composer::rebuild(rebuildConstraint, state);
    bool mayBeTrue;
    if (!executor->getSolver()->mayBeTrue(state->constraints, rebuildConstraint, 
                                          mayBeTrue, state->queryMetaData)) {
      return;
    }
    if (mayBeTrue)
      state->addConstraint(rebuildConstraint);
    else
      return;
  }

  state->statesForRebuild.push_back(S2);
  state->prevPC = S2->prevPC;
  state->pc = S2->pc;
  state->incomingBBIndex = S2->incomingBBIndex;
  state->steppedInstructions = state->steppedInstructions + S2->steppedInstructions;
  state->depth = state->depth + S2->depth;
  state->level.insert(S2->level.begin(), S2->level.end());
  state->multilevel.insert(S2->multilevel.begin(), S2->multilevel.end());

  auto & stateFrame = state->stack.back();
  auto & newFrame = S2->stack.back();
  assert(stateFrame.kf == newFrame.kf &&
        "can not compose states from different functions");
  for(unsigned i = 0; i < newFrame.kf->numRegisters; i++) {
    ref<Expr> newVal = newFrame.locals[i].value;
    if(!newVal.isNull()) {
        for (auto &st : state->statesForRebuild) {
          newVal = Composer::rebuild(newVal, st);
        }
        stateFrame.locals[i].value = newVal;
    }
  }

  result = state;
}


//-----ComposeVisitor-----//

ref<Expr> ComposeVisitor::shareUpdates(ObjectState & OS, const ReadExpr & re) {
  std::stack < const UpdateNode* > forward{};

  for(auto it = re.updates.head;
           !it.isNull();
           it = it->next) {
      forward.push( it.get() );
  }
  // const bool doPrint = !forward.empty();
  // auto out = [&doPrint](const auto & obj) { if(doPrint) llvm::errs() << obj << "\n"; };
  while(!forward.empty()) {
      const UpdateNode* UNode = forward.top();
      forward.pop();
      ref<Expr> newIndex = visit(UNode->index);
      ref<Expr> newValue = visit(UNode->value);
      OS.write(newIndex, newValue);
  }
  ref<Expr> index = visit(re.index);
  return OS.read(index, re.getWidth());
}

ref<Expr> ComposeVisitor::processRead(const ReadExpr & re) {
    ExecutionState *S1 = caller->S1;
    assert(S1 && "no context passed");
    const MemoryObject *object = re.updates.root->binding;
    if(!object) return new ReadExpr(re);
    ObjectState OS{object};
    if(object->isLazyInstantiated()) {
      auto cachedLIexpr = caller->liCache.find(object);
      if(cachedLIexpr != caller->liCache.end()) {
        OS.write(0, cachedLIexpr->second);
        return shareUpdates(OS, re);
      }

      ResolutionList rl;
      Solver::Validity res;
      auto LI = visit(object->lazyInstantiatedSource);
      S1->addressSpace.resolve(*S1, caller->executor->getSolver(), LI, rl);
      assert(!rl.empty() && "should termitate state?");

      std::vector<std::pair<ref<Expr>, ref<Expr>>> results;
      ref<Expr> check = ConstantExpr::create(true, Expr::Bool);
      for(auto op = rl.begin(), e = rl.end(); op != e; op++) {
        const MemoryObject *mo = op->first;
        const ObjectState *os = op->second;
        ref<Expr> inBounds = mo->getBoundsCheckPointer(LI, OS.size);

        time::Span timeout = caller->executor->getMaxSolvTime();
        TimingSolver *solver = caller->executor->getSolver();
        solver->setTimeout(timeout);
        solver->evaluate(S1->constraints, AndExpr::create(check, inBounds), res, S1->queryMetaData);
        solver->setTimeout(time::Span());

        if (res == Solver::True) {
          ref<Expr> trueCase = op->second->read(0, 8 * op->second->size);
          results.push_back({ConstantExpr::create(true, Expr::Bool),
                             SExtExpr::create(trueCase, 8 * OS.size)});
          break;
        } else if (res == Solver::False) {
          continue;
        } else {
          check = AndExpr::create(check, Expr::createIsZero(inBounds));
          ref<Expr> trueCase = op->second->read(0, 8 * op->second->size);
          results.push_back({inBounds, SExtExpr::create(trueCase, 8 * OS.size)});
        }
      }

      if (res != Solver::True) {
        ObjectPair p = caller->executor->lazyInstantiateVariable(
        *S1, LI, object->allocSite, object->size);
        ref<Expr> trueCase = p.second->read(0, 8 * p.second->size);
        results.push_back({ConstantExpr::create(true, Expr::Bool),
                           SExtExpr::create(trueCase, 8 * OS.size)});
      }

      ref<Expr> result;

      if (results.size() == 1) {
        result = results.back().second;
      } else {
        result = results.back().second;
        results.pop_back();
        for (auto cri = results.rbegin(), cre = results.rend(); cri != cre; ++cri) {
          result = SelectExpr::create(cri->first, cri->second, result);
        }
      }

      caller->liCache[object] = result;
      OS.write(0, result);
      return shareUpdates(OS, re);
    }

    const llvm::Value *allocSite = object->allocSite;
    if(!allocSite) return new ReadExpr(re);

    ref<Expr> prevVal;
    if(isa<Argument>(allocSite)) {
      const Argument *arg = cast<Argument>(allocSite);
      const Function *f   = arg->getParent();
      const KFunction *kf = caller->executor->getKFunction(f);
      const unsigned argN = arg->getArgNo();
      prevVal = caller->executor->getArgumentCell(*S1, kf, argN).value;
    } else if(isa<Instruction>(allocSite)) {
      const Instruction *inst = cast<Instruction>(allocSite);
      const KInstruction *ki  = caller->executor->getKInst(const_cast<Instruction*>(inst));
      if (isa<PHINode>(inst))
        prevVal = caller->executor->eval(ki, S1->incomingBBIndex, *S1).value;
      else
        prevVal = caller->executor->getDestCell(*S1, ki).value;
    } else {
      return new ReadExpr(re);
    }

    if(prevVal.isNull() /*|| prevVal->isFalse()*/) {
      return new ReadExpr(re);
    }
    OS.write(0, prevVal);
    return shareUpdates(OS, re);
}
//~~~~~ComposeVisitor~~~~~//

} // namespace klee
