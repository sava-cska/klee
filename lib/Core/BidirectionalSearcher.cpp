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
  Action *action = nullptr;
  while (!action) {
    switch (selectStep()) {

    case StepKind::Forward: {
      auto &state = forward->selectState();
      if(isStuck(state)) {
        KBlock *target = ex->calculateTargetByTransitionHistory(state);
        if (target) {
          state.targets.insert(Target(target, false));
          forward->update(&state, {}, {});
          action = new ForwardAction(&state);
        } else {
          ex->pauseState(state);
        }
      } else
        action = new ForwardAction(&state);
      break;
    }

    case StepKind::Branch : {
      auto& state = branch->selectState();
      action = new BranchAction(&state);
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
        backward->update(pob);
      }
    }

  } else if(std::holds_alternative<BranchResult>(r)) {
    auto br = std::get<BranchResult>(r);
    std::vector<ExecutionState *> addedStates = br.addedStates;
    std::vector<ExecutionState *> removedStates = br.removedStates;

    if (br.current &&
        std::find(removedStates.begin(), removedStates.end(), br.current) == removedStates.end() &&
        (ex->initialState->getInitPCBlock() == br.current->getInitPCBlock() ||
         br.current->maxLevel > 1)) {
      removedStates.push_back(br.current);
    }
    for (auto state : br.addedStates) {
      if (ex->initialState->getInitPCBlock() == state->getInitPCBlock() ||
          state->maxLevel > 1) {
        std::vector<ExecutionState *>::iterator ita =
          std::find(addedStates.begin(), addedStates.end(), state);
        addedStates.erase(ita);
      }
    }

    branch->update(br.current, addedStates, removedStates);

    auto reached = branch->collectAndClearReached();
    for (auto &targetStates : reached) {
      for (auto state : targetStates.second) {
        ex->emanager->states[targetStates.first].insert(&state->copy());
      }
    }
  } else if (std::holds_alternative<BackwardResult>(r)) {
    auto br = std::get<BackwardResult>(r);
    for(auto i : br.newPobs) {
      backward->update(i);
      initializer->addPob(i);
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
  backward = new RecencyRankedSearcher;
  backward->emanager = ex->emanager;
  initializer = new ValidityCoreInitializer(ex->initialState->pc);
}

void BidirectionalSearcher::closeProofObligation(ProofObligation* pob) {
  initializer->removePob(pob);
  backward->removePob(pob);
  for(auto i : pob->children) {
    i->parent = nullptr;
    closeProofObligation(i);
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

bool BidirectionalSearcher::empty() {
  return forward->empty() && backward->empty() && initializer->empty();
}

} // namespace klee
