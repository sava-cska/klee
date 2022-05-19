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
#include "Executor.h"
#include "ExecutionState.h"
#include "ForwardSearcher.h"
#include "Initializer.h"
#include "MergeHandler.h"
#include "ProofObligation.h"
#include "SearcherUtil.h"
#include "UserSearcher.h"
#include "klee/Core/Interpreter.h"
#include "klee/Module/KModule.h"
#include "klee/Support/ErrorHandling.h"
#include <iostream>
#include <llvm/ADT/StringExtras.h>
#include <memory>
#include <unordered_set>
#include <variant>
#include <vector>

#include <cstdlib>

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
          state.targets.insert(Target(target, false));
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

    case StepKind::Branch : {
      auto& state = branch->selectState();
      if (ex->initialState->getInitPCBlock() == state.getInitPCBlock() ||
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

void BidirectionalSearcher::update(ActionResult r) {
  if(std::holds_alternative<ForwardResult>(r)) {
    auto fr = std::get<ForwardResult>(r);
    forward->update(fr.current, fr.addedStates, fr.removedStates);

    if(fr.validityCore) {
      initializer->addValidityCoreInit(fr.validityCore->core, fr.validityCore->target);
      if(!mainLocs.count(fr.validityCore->target->basicBlock)) {
        mainLocs.insert(fr.validityCore->target->basicBlock);
        ProofObligation* pob = new ProofObligation(fr.validityCore->target, nullptr, false);
        llvm::errs() << "Add new proof obligation.\n";
        llvm::errs() << "At: " << pob->location->getIRLocation() << "\n";
        addPob(pob);
      }
    }

  } else if(std::holds_alternative<BranchResult>(r)) {
    auto br = std::get<BranchResult>(r);
    branch->update(br.current, br.addedStates, br.removedStates);

    auto reached = branch->collectAndClearReached();
    for (auto &targetStates : reached) {
      for (auto state : targetStates.second) {
        llvm::errs() << "New isolated state.\n";
        llvm::errs() << "Id: " << state->id << "\n";
        llvm::errs() << "Path: " << state->path.toString() << "\n";
        llvm::errs() << "Constraints:\n" << state->constraints << "\n";
        llvm::errs() << "\n";
        ex->emanager->insert(targetStates.first, *state->copy());
      }
    }
  } else if (std::holds_alternative<BackwardResult>(r)) {
    auto br = std::get<BackwardResult>(r);
    for(auto pob : br.newPobs) {
      addPob(pob);
    }
  } else if (std::holds_alternative<InitializeResult>(r)) {
    auto ir = std::get<InitializeResult>(r);
    branch->update(nullptr, {&ir.state}, {});
  }
}

BidirectionalSearcher::BidirectionalSearcher(const SearcherConfig &cfg)
    : ticker({80,10,5,5}) {
  ex = cfg.executor;
  forward = new GuidedSearcher(constructUserSearcher(*cfg.executor), true);
  forward->update(nullptr,{cfg.initial_state},{});
  branch = new GuidedSearcher(std::unique_ptr<ForwardSearcher>(new BFSSearcher()), false);
  backward = new RecencyRankedSearcher(ex->emanager);
  initializer = new ValidityCoreInitializer(ex->initialState->pc);
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

void BidirectionalSearcher::closeProofObligation(ProofObligation* pob) {
  removePob(pob);
  for(auto child : pob->children) {
    child->parent = nullptr;
    closeProofObligation(child);
  }
  ProofObligation *parent = pob->parent;
  if (parent) {
    parent->children.erase(pob);
  }
  delete pob;
  if (parent) {
    closeProofObligation(parent);
  }
}


bool BidirectionalSearcher::isStuck(ExecutionState& state) {
  KInstruction *prevKI = state.prevPC;
  return prevKI->inst->isTerminator() &&
         state.targets.empty() &&
         state.multilevel.count(state.getPCBlock()) > 0;
}


void BidirectionalSearcher::addPob(ProofObligation *pob) {
  pobs.push_back(pob);
  backward->update(pob);
  initializer->addPob(pob);
}

void BidirectionalSearcher::removePob(ProofObligation *pob) {
  auto pos = std::find(pobs.begin(), pobs.end(), pob);
  if(pos != pobs.end()) {
    pobs.erase(pos);
  }
  backward->removePob(pob);
  initializer->removePob(pob);
}

bool BidirectionalSearcher::empty() {
  return forward->empty() && backward->empty() && initializer->empty();
}

} // namespace klee
