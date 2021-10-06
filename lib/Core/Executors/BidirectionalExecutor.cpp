//===-- BidirectionalExecutor.cpp ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "BidirectionalExecutor.h"

#include "../BidirectionalSearcher.h"
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

void BidirectionalExecutor::initializeRoot(ExecutionState const &state, KBlock *kb) {
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
    if (state.multilevel > bound) {
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
      if (!state.target && state.multilevel.count(state.getPCBlock()) > MaxCycles - 1) // ok
        return false;
      composeStep(state);
    }
  } else if (state.isIsolated()) {
    isolatedExecuteStep(state);
  } else {
    executeStep(state);
  }
  return true;
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
      std::vector<ExecutionState *> currentResult;
      Composer::compose(&lstate, const_cast<ExecutionState*>(rstate), currentResult);
      if (!currentResult.empty()) removeTargetable(*rstate);
      result.insert(result.end(), currentResult.begin(), currentResult.end());
    }
  }
  for (auto &blockstate : results[lstate.getPCBlock()].redundantStates) {
    for (auto rstate : blockstate.second) {
      std::vector<ExecutionState *> currentResult;
      Composer::compose(&lstate, const_cast<ExecutionState*>(rstate), currentResult);
      result.insert(result.end(), currentResult.begin(), currentResult.end());
    }
  }
  removedStates.push_back(&lstate);
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
  BasicBlock *bb = state.getPCBlock();
  KFunction *kf = kmodule->functionMap[bb->getParent()];
  KBlock *kb = kf->blockMap[bb];
  KBlock *nearestBlock = nullptr;
  unsigned int sfNum = 0;
  for (auto sfi = state.stack.rbegin(), sfe = state.stack.rend(); sfi != sfe; sfi++, sfNum++) {
    kf = sfi->kf;
    std::map<KBlock*, unsigned int> &kbd = kf->getDistance(kb);
    for (auto &tstate : targetableStates[kf->function]) {
      KBlock *currKB = kf->blockMap[tstate->getInitPCBlock()];
      if (kbd.find(currKB) != kbd.end())
        nearestBlock = currKB;
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
    if (state.target)
      executeStep(es);
    else if (!tryCoverStep(es, *initialState)) {
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

void Composer::compose(ExecutionState *state, std::vector<ExecutionState*> &result) {
  assert(S1 && S2 && state && !S1->isEmpty() && !S2->isEmpty());

  executor->getProcessForest()->attach(S1->ptreeNode, state, const_cast<ExecutionState *>(S1));
  state->prevPC = S2->prevPC;
  state->pc = S2->pc;
  state->incomingBBIndex = S2->incomingBBIndex;
  state->pathOS = S2->pathOS.copy(); //TreeStream.cpp
  state->symPathOS = S2->symPathOS.copy();
  state->queryMetaData.queryCost =  state->queryMetaData.queryCost + S2->queryMetaData.queryCost;
  state->depth = state->depth + S2->depth;
  state->coveredNew = state->coveredNew || S2->coveredNew;
  state->forkDisabled = state->forkDisabled || S2->forkDisabled;
  state->steppedInstructions = state->steppedInstructions + S2->steppedInstructions;
  state->executionPath = state->executionPath + S2->executionPath;
  state->level.insert(S2->level.begin(), S2->level.end());
  state->multilevel.insert(S2->multilevel);

  //state->arrayNames = P1->arrayNames; by default
  auto &names = S2->arrayNames;
  state->arrayNames.insert(names.begin(), names.end());

  //state->openMergestack = P1->openMergeStack
  auto &OMcur = state->openMergeStack;
  auto &OMoth = S2->openMergeStack;
  OMcur.insert( OMcur.end(), OMoth.begin(), OMoth.end() );

  //state->instsSinceCovNew
  if(S2->coveredNew) {
    state->instsSinceCovNew = S2->instsSinceCovNew;
  } else if(S1->coveredNew) {
    state->instsSinceCovNew = S2->steppedInstructions + S1->instsSinceCovNew;
  } else {
    state->instsSinceCovNew = 0;
  }

  //state->coveredLines = P1->coveredLines; by default
  for(const auto & file: S2->coveredLines) {
    auto acceptor = state->coveredLines.find(file.first);
    if(acceptor != state->coveredLines.end()) {
      const auto &p1set = file.second;
      acceptor->second.insert(p1set.begin(), p1set.end());
    } else {
      state->coveredLines[file.first] = file.second;
    }
  }

  // CONSTRAINTS //
  if(!addComposedConstraints(*state,
          executor->getMaxSolvTime(),
          executor->getSolver())) {
    return;
  }

  // STACK //
  auto & stateFrame = state->stack.back();
  auto & newFrame = S2->stack.back();
  assert(stateFrame.kf == newFrame.kf &&
        "can not compose states from different functions");
  for(unsigned i = 0; i < newFrame.kf->numRegisters; i++) {
    ref<Expr> newVal = newFrame.locals[i].value;
    if(!newVal.isNull()) {
        newVal = rebuild(newVal);
        stateFrame.locals[i].value = newVal;
    }
  }

  // SYMBOLICS //
  // state->symbolics = S1->symbolics
  for(const symb & symbolic: S2->symbolics) {
    state->symbolics.erase(
      std::remove_if(
        state->symbolics.begin(),
        state->symbolics.end(),
        [&symbolic] (const symb& obj) { return obj.second == symbolic.second; }
      ),
      state->symbolics.end()
    );
    state->symbolics.push_back(symbolic);
  }

  // ADDRESS SPACE //
  //state->addressSpace = P1->addressSpace
  result.push_back(state);
  executor->addState(*state);
  for(auto it  = S2->addressSpace.objects.begin();
           it != S2->addressSpace.objects.end();
           ++it) {
    const ObjectState *thisOS{ it->second.get() };
    const MemoryObject *MO{ it->first };
    assert(thisOS->size == MO->size &&
           MO->size != 0 &&
           thisOS->size != 0 &&
           "corrupted object in addressSpace");
    if(thisOS->readOnly) {
      for(auto st : result) {
        st->addressSpace.bindObject(MO, new ObjectState(*thisOS));
      }
      continue;
    }

    ref<Expr> valInS2 = thisOS->read(0, 8*thisOS->size);
    ref<Expr> valInS1 = rebuild(valInS2);

    if(MO->isLazyInstantiated()) { // is LI
      ref<Expr> LI = rebuild(MO->lazyInstantiatedSource);
      std::vector<ExecutionState*> oldRes;
      std::swap(result, oldRes);
      for(auto st : oldRes) {
        std::vector<ExecutionState*> curRes;
        executor->executeMemoryOperation(*st, BidirectionalExecutor::Write, LI, valInS1, nullptr, &curRes);
        result.insert(result.end(), curRes.begin(), curRes.end());
      }
      continue;
    }
    // compose a real object

    for(auto st : result) {
      ObjectState *newOS = new ObjectState(MO);
      newOS->write(0, valInS1);
      st->addressSpace.bindObject(MO, newOS);
    }
  }
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
    const ExecutionState *S1 = caller->S1;
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
      auto LI = visit(object->lazyInstantiatedSource);
      S1->addressSpace.resolve(
        *S1, caller->executor->getSolver(), LI, rl);
      assert(!rl.empty() && "should termitate state?");
      ObjectState defaultCase{object, re.updates.root};
      auto re1 = new ReadExpr(re);
      defaultCase.write(re1->index, re1);
      ref<Expr> sel = SExtExpr::create(defaultCase.read(0, 8*defaultCase.size), 8*OS.size);
      for(auto op = rl.begin(), e = rl.end(); op != e; op++) {
        sel = branchResolvePointer(*op, LI, sel, 8*OS.size);
      }
      caller->liCache[object] = sel;
      OS.write(0, sel);
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

namespace {

ConstraintSet compose(ExecutionState & state, ConstraintSet const & fromPob, TimingSolver & solver, bool & canBeTrue) {
  auto resultConstraints = state.constraints;
  canBeTrue = true;
  bool mayBeTrue;
  for (auto const & expr : fromPob) {
    auto newExpr = Composer::rebuild(expr, &state);
    if (!solver.mayBeTrue(resultConstraints, newExpr, mayBeTrue, state.queryMetaData))
      assert(false && "Unhandled solver failure");
    if (mayBeTrue) {
      resultConstraints.push_back(newExpr);
    } else {
      canBeTrue = false;
      break;
    }
  }
  return resultConstraints;
}

} // namespace

void BidirectionalExecutor::goFront(ExecutionState & state, std::queue<ExecutionState *> & forwardQueue) {
  executeInstruction(state, state.pc);
  for (auto const & newState : addedStates) {
    forwardQueue.push(newState);
    if (newState != &state) {
      assert(newState->unblockedPobs.empty());
      for (auto const & pob : state.unblockedPobs) {
        assert(!pob->answered);
        markUnblocked(*newState, *pob);
      }
    }
  }
  assert(removedStates.empty());
}

void BidirectionalExecutor::goBack(ExecutionState & state, ProofObligation & pob, std::deque<ProofObligation> & backwardQueue, ExecutionState const & entryPoint) {
  assert(state.pc->parent == &pob.location);
  bool mayBeTrue;
  auto composedCondition = compose(state, pob.condition, *solver, mayBeTrue);
  if (mayBeTrue) {
    pob.block(state);
    state.block(pob);
    auto nearestReachableParent = pob.propagateUnreachability();
    assert(nearestReachableParent->isOriginPob() || !nearestReachableParent->isUnreachable());
    if (nearestReachableParent->isOriginPob() && nearestReachableParent->isUnreachable()) {
        // answer_no(...);
        // static_assert(false && "TODO");
    }
    return;
  }

  if (&state == &entryPoint) {
    auto root = pob.propagateReachability();
    assert(root->isOriginPob());
    std::unique_ptr<ExecutionState> testCaseState(state.copy());
    testCaseState->constraints = std::move(composedCondition);
    interpreterHandler->processTestCase(*testCaseState, "meow", "woof");
    return;
  }

  auto lvl = state.multilevel.level();
  auto newLocation = kmodule->functionMap[state.getInitPCBlock()->getParent()]->blockMap[state.getInitPCBlock()];
  backwardQueue.push_back(pob.makeNewChild(*newLocation, lvl, std::move(composedCondition)));
}

bool BidirectionalExecutor::canReach(ExecutionState & state, ProofObligation & pob) const {
  // kmodule->getDistance(??);
  static_assert(false && "TODO");
  return false;
}

void BidirectionalExecutor::markUnblocked(ExecutionState & state, ProofObligation & pob) const {
  if (state.multilevel <= pob.current_lvl && canReach(state, pob)) {
    pob.addAsUnblocked(state);
    state.unblock(pob);
  }
}

void BidirectionalExecutor::reachTarget(ExecutionState const & initialState, KBlock const & target, size_t lvl_bound) {
  using namespace std;
  queue<ExecutionState *> forwardQueue;
  deque<ProofObligation> backwardQueue;
  SimpleBidirectionalSearcher searcher;

  // we rely on that an empty ConstrainSet represents "true"
  backwardQueue.emplace_back(target);

  while(!backwardQueue.empty()) {
    auto action = searcher.selectState();
    switch (action.type) {
      case Action::Type::Init:
        initializeRoot(initialState, action.location->parent->blockMap[action.state->getPCBlock()]);
        updateStates(nullptr);
        for (auto & pob : backwardQueue)
          markUnblocked(*action.state, pob);
        break;
      case Action::Type::Forward:
        assert(forwardQueue.front() == action.state);
        forwardQueue.pop();
        if (action.state->multilevel < lvl_bound)
          goFront(*action.state, forwardQueue);
        break;
      case Action::Type::Backward:
        if (!action.pob->answered) // otherwise we have already handle this pob
          goBack(*action.state, *action.pob, backwardQueue, initialState);
        break;
      default: assert(false && "unreachable");
    }
  }
}

} // namespace klee
