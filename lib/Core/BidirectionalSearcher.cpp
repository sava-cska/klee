//===-- BidirectionalSearcher.h ----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "BidirectionalSearcher.h"
#include "SearcherUtil.h"
#include "BackwardSearcher.h"
#include "Executor.h"
#include "ExecutionState.h"
#include "ForwardSearcher.h"
#include "Initializer.h"
#include "MergeHandler.h"
#include "ProofObligation.h"
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
llvm::cl::opt<bool> DebugBidirectionalSearcher(
    "debug-bidirectional-searcher",
    llvm::cl::desc(""),
    llvm::cl::init(false),
    llvm::cl::cat(klee::DebugCat));

llvm::cl::opt<unsigned> MaxCycles(
    "max-cycles",
    llvm::cl::desc("stop execution after visiting some basic block this amount of times (default=0)."),
    llvm::cl::init(0),
    llvm::cl::cat(klee::TerminationCat));

}

namespace klee {

BidirectionalSearcher::StepKind
BidirectionalSearcher::selectStep() {
  size_t initial_choice = ticker.getCurrent();
  size_t choice = initial_choice;

  do {
    switch (choice) {
    case 0:
      if(!forward->empty()) return StepKind::Forward;
    case 1:
      if(!branch->empty()) return StepKind::Branch;
    case 2:
      if(!backward->empty()) return StepKind::Backward;
    case 3:
      if(!initializer->empty()) return StepKind::Initialize;
    }
    ticker.moveToNext();
    choice = ticker.getCurrent();
  } while (choice != initial_choice);

  return StepKind::Terminate;
}

Action &BidirectionalSearcher::selectAction() {
  Action *action;
  bool actionWasSelected = false;
  while (!actionWasSelected) {
    actionWasSelected = true;
    switch (selectStep()) {

    case StepKind::Forward: {
      auto &state = forward->selectState();
      if (isStuck(state)) {
        KBlock *target = ex->calculateTargetByTransitionHistory(state);
        if (target) {
          state.targets.insert(Target(target));
          forward->update(&state, {}, {});
          action = new ForwardAction(&state);
        } else {
          forward->update(nullptr, {}, {&state});
          ex->pauseState(state);
          actionWasSelected = false;
        }
      } else
        action = new ForwardAction(&state);
      break;
    }

    case StepKind::Branch: {
      auto &state = branch->selectState();
      if (ex->initialState->getInitPCBlock() != state.getInitPCBlock() &&
          state.maxLevel > 1) {
        branch->update(nullptr, {}, {&state});
        ex->pauseState(state);
        actionWasSelected = false;
      } else {
        action = new BranchAction(&state);
      }
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
  return *action;
}

void BidirectionalSearcher::update(ActionResult *r) {
  if (r->getKind() == ActionResult::Kind::Forward) {
    auto fr = cast<ForwardResult>(r);
    forward->update(fr->current, fr->addedStates, fr->removedStates);


    if (fr->current && fr->current->getPrevPCBlock() != fr->current->getPCBlock()) {
      Target target =
        isa<KReturnBlock>(fr->current->prevPC->parent) ?
          Target(fr->current->prevPC->parent) :
          Target(fr->current->pc->parent);
      backward->addState(target, fr->current);
    }
    for (auto &state : fr->addedStates) {
      if (state->getPrevPCBlock() != state->getPCBlock()) {
        Target target =
          isa<KReturnBlock>(state->prevPC->parent) ?
            Target(state->prevPC->parent) :
            Target(state->pc->parent);
        backward->addState(target, state);
      }
    }

    if (fr->targetedConflict) {
      if (!rootBlocks.count(fr->targetedConflict->target->basicBlock) &&
          !reachableBlocks.count(fr->targetedConflict->target->basicBlock)) {
        rootBlocks.insert(fr->targetedConflict->target->basicBlock);
        initializer->addConflictInit(fr->targetedConflict->conflict, fr->targetedConflict->target);
        ProofObligation* pob = new ProofObligation(fr->targetedConflict->target);


        if (DebugBidirectionalSearcher) {
          llvm::errs() << "Add new proof obligation.\n";
          llvm::errs() << "At: " << pob->location->getIRLocation() << "\n";
          llvm::errs() << "\n";
        }
        addPob(pob);
      }
    }


  } else if (r->getKind() == ActionResult::Kind::Branch) {
    auto br = cast<BranchResult>(r);
    branch->update(br->current, br->addedStates, br->removedStates);

    auto reached = branch->collectAndClearReached();
    for (auto &targetStates : reached) {
      for (auto state : targetStates.second) {
        if (DebugBidirectionalSearcher) {
          llvm::errs() << "New isolated state.\n";
          llvm::errs() << "Id: " << state->id << "\n";
          llvm::errs() << "Path: " << state->path.toString() << "\n";
          llvm::errs() << "Constraints:\n" << state->constraints;
          llvm::errs() << "\n";
        }
        backward->addState(targetStates.first, state);
      }
    }

  } else if (r->getKind() == ActionResult::Kind::Backward) {
    auto br = cast<BackwardResult>(r);
    for (auto pob : br->newPobs) {
      addPob(pob);
    }
  } else if (r->getKind() == ActionResult::Kind::Initialize) {
    auto ir = cast<InitializeResult>(r);
    branch->update(nullptr, {&ir->state}, {});
  }
}

BidirectionalSearcher::BidirectionalSearcher(const SearcherConfig &cfg)
    : ticker({80,10,5,5}) {
  ex = cfg.executor;
  forward = new GuidedSearcher(constructUserSearcher(*cfg.executor), true);
  forward->update(nullptr,{cfg.initial_state},{});
  branch = new GuidedSearcher(std::unique_ptr<ForwardSearcher>(new BFSSearcher()), false);
  backward = new RecencyRankedSearcher();
  initializer = new ConflictCoreInitializer(ex->initialState->pc);
}

BidirectionalSearcher::~BidirectionalSearcher() {
  for (auto pob : pobs) {
    delete pob;
  }
  pobs.clear();
  delete forward;
  delete branch;
  delete backward;
  delete initializer;
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

bool BidirectionalSearcher::isStuck(ExecutionState& state) {
  KInstruction *prevKI = state.prevPC;
  return prevKI->inst->isTerminator() &&
         state.targets.empty() &&
         state.multilevel.count(state.getPCBlock()) > MaxCycles;
}

void BidirectionalSearcher::addPob(ProofObligation *pob) {
  pobs.push_back(pob);
  backward->update(pob);
  initializer->addPob(pob);
}

void BidirectionalSearcher::removePob(ProofObligation *pob) {
  auto pos = std::find(pobs.begin(), pobs.end(), pob);
  if (pos != pobs.end()) {
    pobs.erase(pos);
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

bool BidirectionalSearcher::empty() {
  return forward->empty() && backward->empty() && initializer->empty();
}

} // namespace klee
