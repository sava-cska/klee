//===-- BidirectionalSearcher.h ----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "BidirectionalSearcher.h"
#include "BackwardSearcher.h"
#include "ExecutionState.h"
#include "Executor.h"
#include "ForwardSearcher.h"
#include "MergeHandler.h"
#include "ProofObligation.h"
#include "SearcherUtil.h"
#include "UserSearcher.h"
#include "klee/Core/Interpreter.h"
#include "klee/Module/KModule.h"
#include "klee/Support/ErrorHandling.h"
#include "klee/Support/OptionCategories.h"

#include "llvm/ADT/StringExtras.h"
#include "llvm/IR/Instructions.h"

#include <iostream>
#include <memory>
#include <unordered_set>
#include <vector>

#include <cstdlib>

namespace {
llvm::cl::opt<bool> DebugBidirectionalSearcher("debug-bidirectional-searcher",
                                               llvm::cl::desc(""),
                                               llvm::cl::init(false),
                                               llvm::cl::cat(klee::DebugCat));

llvm::cl::opt<unsigned>
    MaxCycles("max-cycles",
              llvm::cl::desc("stop execution after visiting some basic block "
                             "this amount of times (default=0)."),
              llvm::cl::init(0), llvm::cl::cat(klee::TerminationCat));

llvm::cl::opt<bool> DoBackwardFirst(
    "do-backward-first",
    llvm::cl::desc("do all possible backward steps with a state before "
                   "forward step (default=true)."),
    llvm::cl::init(true), llvm::cl::cat(klee::ExecCat));

llvm::cl::opt<bool> PruneStates(
    "prune-states",
    llvm::cl::desc("prune a state of failed backward step (default=false)."),
    llvm::cl::init(false), llvm::cl::cat(klee::ExecCat));

llvm::cl::opt<bool> UseOnlyIsolatedStates(
    "use-only-isolated-states",
    llvm::cl::desc(
        "use only isolated states for backward step (default=false)."),
    llvm::cl::init(false), llvm::cl::cat(klee::ExecCat));

} // namespace

namespace klee {

BidirectionalSearcher::StepKind BidirectionalSearcher::selectStep() {
  size_t initial_choice = ticker.getCurrent();
  size_t choice = initial_choice;

  do {
    switch (choice) {
    case 0: {
      if (!forward->empty()) {
        return StepKind::Forward;
      }
      break;
    }
    case 1: {
      if (!branch->empty() && !pobs.empty()) {
        return StepKind::Branch;
      }
      break;
    }
    case 2: {
      if (!backward->empty()) {
        return StepKind::Backward;
      }
      break;
    }
    case 3: {
      if (!initializer->empty()) {
        return StepKind::Initialize;
      }
      break;
    }
    }
    ticker.moveToNext();
    choice = ticker.getCurrent();
  } while (choice != initial_choice);

  return StepKind::Terminate;
}

ref<BidirectionalAction> BidirectionalSearcher::selectAction() {
  ref<BidirectionalAction> action;
  while (action.isNull()) {
    switch (selectStep()) {

    case StepKind::Forward: {
      auto &state = forward->selectState();

      if (DoBackwardFirst && state.backwardStepsLeftCounter > 0) {
        break;
      }

      if (PruneStates && state.failedBackwardStepsCounter > 0) {
        forward->update(nullptr, {}, {&state});
        ex->pauseState(state);
        break;
      }

      if (isStuck(state)) {
        KBlock *target = ex->calculateTargetByBlockHistory(state);
        if (target) {
          state.addTarget(Target(target));
          forward->update(&state, {}, {});
          action = new ForwardAction(&state);
        } else {
          forward->update(nullptr, {}, {&state});
          ex->pauseState(state);
        }
      } else
        action = new ForwardAction(&state);
      break;
    }

    case StepKind::Branch: {
      auto &state = branch->selectState();
      /*KInstruction *prevKI = state.prevPC;
      if (ex->initialState->getInitPCBlock() != state.getInitPCBlock() &&
          prevKI->inst->isTerminator() &&
          state.multilevel.count(state.getPCBlock()) > 0) {
        branch->update(nullptr, {}, {&state}); // вот тут беда, забыли про условие
        ex->pauseState(state);
      } else {*/
        action = new BranchAction(&state);
      //}
      break;
    }

    case StepKind::Backward: {
      auto pobState = backward->selectAction();
      action = new BackwardAction(pobState.second, pobState.first);
      break;
    }

    case StepKind::Initialize: {
      auto initAndTargets = initializer->selectAction();
      action =
          new InitializeAction(initAndTargets.first, initAndTargets.second);
      break;
    }

    case StepKind::Terminate: {
      action = new TerminateAction();
      break;
    }
    }
  }
  return action;
}

void BidirectionalSearcher::updateForward(
    ExecutionState *current, const std::vector<ExecutionState *> &addedStates,
    const std::vector<ExecutionState *> &removedStates,
    ref<TargetedConflict> targetedConflict) {

  forward->update(current, addedStates, removedStates);

  std::vector<ExecutionState *> states;
  if (current)
    states.push_back(current);
  states.insert(states.end(), addedStates.begin(), addedStates.end());

  if (!UseOnlyIsolatedStates) {
    for (auto &state : states) {
      if (state->getPrevPCBlock() != state->getPCBlock() &&
          !isa<KReturnBlock>(state->pc->parent) &&
          !isa<KReturnBlock>(state->prevPC->parent)) {
        Target target = Target(state->pc->parent);
        backward->addState(target, state);
      }
    }
  }

  if (targetedConflict) {
    if (!rootBlocks.count(targetedConflict->target->basicBlock) &&
        !reachableBlocks.count(targetedConflict->target->basicBlock)) {
      rootBlocks.insert(targetedConflict->target->basicBlock);
      initializer->addConflictInit(targetedConflict->conflict,
                                   targetedConflict->target);
      ProofObligation *pob = new ProofObligation(targetedConflict->target);

      if (DebugBidirectionalSearcher) {
        llvm::errs() << "Add new proof obligation.\n";
        llvm::errs() << "At: " << pob->location->getIRLocation() << "\n";
        llvm::errs() << "\n";
      }
      addPob(pob);
    }
  }
}

void BidirectionalSearcher::updateBranch(
    ExecutionState *current, const std::vector<ExecutionState *> &addedStates,
    const std::vector<ExecutionState *> &removedStates) {
  std::map<Target, std::unordered_set<ExecutionState *>> reached;
  std::map<Target, std::unordered_set<ExecutionState *>> unreached;

  for (const ExecutionState *state : addedStates) {
    for (const Target &target : state->targets) {
      reachabilityTracker->addRunningStateToTarget(state, target);
    }
  }

  std::vector<ExecutionState *> tmpAddedStates = addedStates, tmpRemovedStates = removedStates;
  {
    KBlock *initPCBlock = ex->initialState->initPC->parent;
    if (current != nullptr) {
      KInstruction *prevKI = current->prevPC;
      if (initPCBlock->basicBlock != current->getInitPCBlock() &&
          prevKI->inst->isTerminator() &&
          current->multilevel.count(current->getPCBlock()) > 0) {
        tmpRemovedStates.push_back(current);
        // ex->pauseState(*current);
        // вот тут беда, забыли про условие
      }
    }

    auto it = tmpAddedStates.begin();
    while (it != tmpAddedStates.end()) {
      ExecutionState *state = *it;
      KInstruction *prevKI = state->prevPC;
      if (initPCBlock->basicBlock != state->getInitPCBlock() &&
          prevKI->inst->isTerminator() &&
          state->multilevel.count(state->getPCBlock()) > 0) {
        it = tmpAddedStates.erase(it);
        // вот тут беда, забыли про условие
      } else {
        it++;
      }
    }
  }

  //addedStates идут до Target (.targets)
  branch->update(current, tmpAddedStates, tmpRemovedStates, reached, unreached);
  //вернулся из reached и unreached -- удаляю (.location)
  //множество опустело -- всё
  //зарегистрировать все state из addedStates
  //все стейты из addedStates я добавляю в Running
  //reached и unreached я удаляю из Running


  for (const auto &targetAndStates : reached) {
    for (const auto &reachedState : targetAndStates.second) {
      reachabilityTracker->removeRunningStateToTarget(reachedState,
                                              targetAndStates.first);
    }
  }
  for (const auto &targetAndStates : unreached) {
    for (const auto &unreachedState : targetAndStates.second) {
      reachabilityTracker->removeRunningStateToTarget(unreachedState,
                                              targetAndStates.first);
    }
  }

  for (const auto &targetAndStates : reached) {
    for (const auto &reachedState : targetAndStates.second) {
      if (targetAndStates.first.atReturn() && reachedState->stack.size() > 0) {
        closePobsInTargetIfNeeded(targetAndStates.first);
        continue;
      }
      if (DebugBidirectionalSearcher) {
        llvm::errs() << "New isolated state.\n";
        llvm::errs() << "Id: " << reachedState->id << "\n";
        llvm::errs() << "Path: " << reachedState->path.toString() << "\n";
        llvm::errs() << "Constraints:\n" << reachedState->constraints;
        llvm::errs() << "\n";
      }
      ExecutionState *newBackwardState = backward->addState(targetAndStates.first, reachedState);
      //добавляю во второе множество
      reachabilityTracker->addWaitingStateToTarget(newBackwardState, targetAndStates.first);
    }
  }

  for (const auto &targetAndStates : unreached) {
      closePobsInTargetIfNeeded(targetAndStates.first);
  }

  {
    KBlock *initPCBlock = ex->initialState->initPC->parent;
    if (current != nullptr) {
      KInstruction *prevKI = current->prevPC;
      if (initPCBlock->basicBlock != current->getInitPCBlock() &&
          prevKI->inst->isTerminator() &&
          current->multilevel.count(current->getPCBlock()) > 0) {
        ex->pauseState(*current);
        // вот тут беда, забыли про условие
      }
    }
  }
}

void BidirectionalSearcher::updateBackward(
    std::vector<ProofObligation *> newPobs, ProofObligation *oldPob, ExecutionState *state) {
  for (auto pob : newPobs) {
    addPob(pob);
  }

  // прокинуть state, удаляю state у pob.location
  //проверить на пустоту два множества

  if (state->isIsolated()) {
    std::string s;
    llvm::raw_string_ostream ss(s);
    state->initPC->parent->basicBlock->printAsOperand(ss, false);
    klee_message("updateBackward: state %s", ss.str().c_str());
    reachabilityTracker->removeWaitingStateToPob(state, oldPob);
    if (newPobs.empty()) {
      klee_message("updateBackward: newPobs is empty\n%s", oldPob->print().c_str());
      closePobIfNoPathLeft(oldPob);
    }
  }
}

void BidirectionalSearcher::updateInitialize(KInstruction *location,
                                             ExecutionState &state) {
  branch->update(nullptr, {&state}, {});
  for (const Target &target : state.targets) {
    reachabilityTracker->createRunningStateToTarget(&state, target);
  }
}

void BidirectionalSearcher::update(ref<ActionResult> r) {
  switch (r->getKind()) {
  case ActionResult::Kind::Forward: {
    auto fr = cast<ForwardResult>(r);
    updateForward(fr->current, fr->addedStates, fr->removedStates,
                  fr->targetedConflict);
    break;
  }
  case ActionResult::Kind::Branch: {
    auto brr = cast<BranchResult>(r);
    updateBranch(brr->current, brr->addedStates, brr->removedStates);
    break;
  }
  case ActionResult::Kind::Backward: {
    auto bckr = cast<BackwardResult>(r);
    updateBackward(bckr->newPobs, bckr->oldPob, bckr->state);
    if (bckr->state->backwardStepsLeftCounter > 0) {
      --bckr->state->backwardStepsLeftCounter;
      if (bckr->newPobs.empty())
        ++bckr->state->failedBackwardStepsCounter;
    }
    break;
  }
  case ActionResult::Kind::Initialize: {
    auto ir = cast<InitializeResult>(r);
    updateInitialize(ir->location, ir->state);
    break;
  }
  default:
    break;
  }
}

BidirectionalSearcher::BidirectionalSearcher(const SearcherConfig &cfg)
    : ticker({80, 10, 5, 5}) {
  ex = cfg.executor;
  forward = new GuidedSearcher(constructUserSearcher(*cfg.executor), true);
  branch = new GuidedSearcher(
      std::unique_ptr<ForwardSearcher>(new BFSSearcher()), false);
  backward = new RecencyRankedSearcher(-1);
  initializer = new ConflictCoreInitializer(ex->initialState->pc);
  reachabilityTracker = new ReachabilityTracker();
}

BidirectionalSearcher::~BidirectionalSearcher() {
  for (auto targetAndPobs : pobs) {
    for (ProofObligation *pob : targetAndPobs.second) {
      delete pob;
    }
  }
  pobs.clear();
  delete forward;
  delete branch;
  delete backward;
  delete initializer;
  delete reachabilityTracker;
}

void BidirectionalSearcher::closeProofObligation(ProofObligation *pob) {
  answerPob(pob);
  std::queue<ProofObligation *> pobs;
  pobs.push(pob->root ? pob->root : pob);
  while (pobs.size()) {
    ProofObligation *currPob = pobs.front();
    pobs.pop();
    for (auto child : currPob->children) {
      pobs.push(child);
    }
    removePob(currPob);
    delete currPob;
  }
}

bool BidirectionalSearcher::closePobIfNoPathLeft(ProofObligation *pob) {
  bool deletePob = false;
  while (pob && pob->children.empty()) {
    reachabilityTracker->updateFinishPob(pob);
    if (reachabilityTracker->isPobProcessAllStates(pob)) {
      deletePob = true;
      klee_message("Close POB!!!!!!!\n%s\n", pob->print().c_str());
      ProofObligation *parent = pob->parent;
      if (parent == nullptr) {
        Target rootTarget = Target(pob->location);
        forward->removeTarget(rootTarget);
        reachabilityTracker->addUnreachableRootTarget(rootTarget);
      }
      removePob(pob);
      pob->detachParent();
      delete pob;
      pob = parent;
    }
    else {
      break;
    }
  }
  return deletePob;
}

void BidirectionalSearcher::closePobsInTargetIfNeeded(const Target &target) {
  if (!reachabilityTracker->emptyRunningStateToTarget(target)) {
    return;
  }

  while (true) {
    if (pobs.find(target.block) == pobs.end()) {
      return;
    }
    std::set<ProofObligation *> allPobsInLoc = pobs[target.block];
    bool deletePob = false;
    for (ProofObligation *pob : allPobsInLoc) {
      if (closePobIfNoPathLeft(pob)) {
        deletePob = true;
        break;
      }
    }
    if (!deletePob) {
      return;
    }
  }
}

bool isStuck(ExecutionState &state) {
  KInstruction *prevKI = state.prevPC;
  return prevKI->inst->isTerminator() && state.targets.empty() &&
         state.multilevel.count(state.getPCBlock()) > MaxCycles;
}

void BidirectionalSearcher::addPob(ProofObligation *pob) {
  pobs[pob->location].insert(pob);
  backward->update(pob);
  initializer->addPob(pob);
}

void BidirectionalSearcher::removePob(ProofObligation *pob) {
  assert(pobs.find(pob->location) != pobs.end());
  assert(pobs[pob->location].find(pob) != pobs[pob->location].end());
  std::set<ProofObligation *> pobsInLoc = pobs[pob->location];
  pobsInLoc.erase(pob); //runningStates опустел, при этом мы ничего не добавили в waitingStates. Не создан waitingStates[target]
  if (!pobsInLoc.empty()) {
    pobs[pob->location] = pobsInLoc;
  } else {
    pobs.erase(pob->location);
  }
  backward->removePob(pob);
  initializer->removePob(pob);
}

void BidirectionalSearcher::answerPob(ProofObligation *pob) {
  while (pob) {
    reachableBlocks.insert(pob->location->basicBlock);
    pob = pob->parent;
  }
}

ReachabilityTracker *BidirectionalSearcher::getReachabilityTracker() const {
  return reachabilityTracker;
}

ref<BidirectionalAction> ForwardOnlySearcher::selectAction() {
  if (searcher->empty()) {
    return new TerminateAction();
  }
  return new ForwardAction(&searcher->selectState());
}

void ForwardOnlySearcher::update(ref<ActionResult> r) {
  switch (r->getKind()) {
  case ActionResult::Kind::Forward: {
    auto fr = cast<ForwardResult>(r);
    searcher->update(fr->current, fr->addedStates, fr->removedStates);
    break;
  }
  case ActionResult::Kind::Terminate: {
    break;
  }
  default: {
    klee_error("ForwardOnlySearcher received non-forward action result.");
  }
  }
}

void ForwardOnlySearcher::closeProofObligation(ProofObligation *) {}

ForwardOnlySearcher::ForwardOnlySearcher(const SearcherConfig &cfg) {
  searcher = constructUserSearcher(*cfg.executor);
}

ForwardOnlySearcher::~ForwardOnlySearcher() {}

ref<BidirectionalAction> GuidedOnlySearcher::selectAction() {
  if (searcher->empty()) {
    return new TerminateAction();
  }

  ref<BidirectionalAction> action;
  while (action.isNull()) {
    auto &state = searcher->selectState();
    if (isStuck(state)) {
      KBlock *target = ex->calculateTargetByBlockHistory(state);
      if (target) {
        state.addTarget(Target(target));
        searcher->update(&state, {}, {});
        action = new ForwardAction(&state);
      } else {
        searcher->update(nullptr, {}, {&state});
        ex->pauseState(state);
      }
    } else
      action = new ForwardAction(&state);
  }
  return new ForwardAction(&searcher->selectState());
}

void GuidedOnlySearcher::update(ref<ActionResult> r) {
  switch (r->getKind()) {
  case ActionResult::Kind::Forward: {
    auto fr = cast<ForwardResult>(r);
    searcher->update(fr->current, fr->addedStates, fr->removedStates);
    break;
  }
  case ActionResult::Kind::Terminate: {
    break;
  }
  default: {
    klee_error("ForwardOnlySearcher received non-forward action result.");
  }
  }
}

void GuidedOnlySearcher::closeProofObligation(ProofObligation *) {}

GuidedOnlySearcher::GuidedOnlySearcher(const SearcherConfig &cfg) {
  ex = cfg.executor;
  searcher = std::unique_ptr<GuidedSearcher>(
      new GuidedSearcher(constructUserSearcher(*cfg.executor), true));
}

GuidedOnlySearcher::~GuidedOnlySearcher() {}

} // namespace klee
