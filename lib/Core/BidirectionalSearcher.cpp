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
  unsigned int tick = choice;
  do {
    unsigned int i = choice;
    choice = (choice + 1) % 4;
    if (i == 0 && !forward->empty())
      return StepKind::Forward;
    else if (i == 1 && !branch->empty())
      return StepKind::Branch;
    else if (i == 2 && !backward->empty())
      return StepKind::Backward;
    else if (i == 3 && !initializer->empty())
      return StepKind::Initialize;
  } while (tick != choice);

  return StepKind::Terminate;
}

Action &BidirectionalSearcher::selectAction() {
  Action *action = nullptr;
  while (!action) {
    switch (selectStep()) {

    case StepKind::Forward: {
      auto &state = forward->selectState();
      action = new ForwardAction(&state);
      break;
    }

    case StepKind::Branch: {
      auto &state = branch->selectState();
      action = new ForwardAction(&state);
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
    ExecutionState* fwdCur = nullptr, *brnchCur = nullptr;
    std::vector<ExecutionState*> fwdAdded;
    std::vector<ExecutionState*> brnchAdded;
    std::vector<ExecutionState*> fwdRemoved;
    std::vector<ExecutionState*> brnchRemoved;

    if(fr.current) {
      if(fr.current->isIsolated())
        brnchCur = fr.current;
      else
        fwdCur = fr.current;
    }
    for(auto i : fr.addedStates) {
      if(i->isIsolated())
        brnchAdded.push_back(i);
      else fwdAdded.push_back(i);
    }
    for(auto i : fr.removedStates) {
      if(i->isIsolated())
        brnchRemoved.push_back(i);
      else fwdRemoved.push_back(i);
    }

    branch->update(brnchCur, brnchAdded, brnchRemoved);
    auto reached = branch->collectAndClearReached();
    for(auto i : reached) {
      for(auto state : i.second) {
        if (ex->initialState->getInitPCBlock() == state->getInitPCBlock() ||
            state->maxLevel == 1) {
          ex->emanager->states[i.first].insert(state->copy());
        }
      }
    }

    forward->update(fwdCur, fwdAdded, fwdRemoved);

    if(fr.validityCore) {
      initializer->addValidityCoreInit(fr.validityCore->core, fr.validityCore->target);
      if(!mainLocs.count(fr.validityCore->target->basicBlock)) {
        mainLocs.insert(fr.validityCore->target->basicBlock);
        ProofObligation* pob = new ProofObligation(fr.validityCore->target, nullptr, false);
        backward->update(pob);
      }
    }

  } else if (std::holds_alternative<BackwardResult>(r)) {
    auto br = std::get<BackwardResult>(r);
    for(auto i : br.newPobs) {
      backward->update(i);
      initializer->addPob(i);
    }
  } else {
    auto ir = std::get<InitializeResult>(r);
    branch->update(nullptr, {ir.state}, {});
  }
}

BidirectionalSearcher::BidirectionalSearcher(SearcherConfig cfg) {
  ex = cfg.executor;
  forward = new GuidedSearcher(constructUserSearcher(*cfg.executor), true);
  forward->update(nullptr,{cfg.initial_state},{});
  branch = new GuidedSearcher(std::unique_ptr<ForwardSearcher>(new BFSSearcher()), false);
  backward = new BFSBackwardSearcher;
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

bool BidirectionalSearcher::empty() {
  return forward->empty() && backward->empty() && initializer->empty();
}

} // namespace klee
