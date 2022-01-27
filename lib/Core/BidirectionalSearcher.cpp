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
#include <llvm/ADT/StringExtras.h>
#include <memory>
#include <unordered_set>
#include <variant>
#include <vector>

#include <cstdlib>

namespace klee {

Action ForwardBidirectionalSearcher::selectAction() {
  while(!searcher->empty()) {
    auto &state = searcher->selectState();
    KInstruction *prevKI = state.prevPC;

    if (prevKI->inst->isTerminator() &&
        state.targets.empty() &&
        state.multilevel.count(state.getPCBlock()) > 0 /* maxcycles - 1 */) {
      KBlock *target = ex->calculateTargetByTransitionHistory(state);
      if (target) {
        state.targets.insert(target);
        ex->updateStates(&state);
        return Action(&state);
      } else {
        ex->pauseState(state);
        ex->updateStates(nullptr);
      }
    } else {
      return Action(&state);
    }
  }
  return Action(Action::Type::Terminate, nullptr, nullptr, nullptr, {});
}

void ForwardBidirectionalSearcher::update(ActionResult r) {
  if(std::holds_alternative<ForwardResult>(r)) {
    auto fr = std::get<ForwardResult>(r);
    searcher->update(fr.current, fr.addedStates, fr.removedStates);
  }
}

ForwardBidirectionalSearcher::ForwardBidirectionalSearcher(SearcherConfig cfg) {
  searcher = new GuidedForwardSearcher(
      constructUserSearcher(*(Executor *)(cfg.executor)));
  for(auto target : cfg.targets) {
    cfg.initial_state->targets.insert(target);
  }
  searcher->update(nullptr,{cfg.initial_state},{});
  ex = cfg.executor;
}

Action BidirectionalSearcher::selectAction() {
  while (true) {
    choice = (choice + 1) % 4;
    if (choice == 0) {
      while (!forward->empty()) {
        auto &state = forward->selectState();
        KInstruction *prevKI = state.prevPC;

        if (prevKI->inst->isTerminator() &&
            state.targets.empty() &&
            state.multilevel.count(state.getPCBlock()) > 0 /* maxcycles - 1 */) {
          KBlock *target = ex->calculateTargetByTransitionHistory(state);
          if (target) {
            state.targets.insert(target);

            ProofObligation *pob = new ProofObligation(target);
            backward->update(pob);
            initializer->addPob(pob);

            ex->updateStates(&state);
            // klee_message("Forward");
            return Action(&state);
          } else {
            ex->pauseState(state);
            ex->updateStates(nullptr);
          }
        } else {
          // klee_message("Forward");
          return Action(&state);
        }
      }
      // klee_message("Forward");
      return Action(Action::Type::Terminate, nullptr, nullptr, nullptr, {});
    }
    if (choice == 1) {
      if (branch->empty()) continue;
      auto& state = branch->selectState();
      // klee_message("Branch");
      return Action(&state);
    }
    if (choice == 2) {
      if (backward->empty()) continue;
      auto a = backward->selectAction();
      // klee_message("Backward");
      return Action(Action::Type::Backward, a.second, nullptr, a.first, {});
    }
    if (choice == 3) {
      if(initializer->empty()) continue;
      auto a = initializer->selectAction();
      klee_message("Initializer");
      return Action(Action::Type::Init, nullptr, a.first, nullptr, a.second);
    }
  }
}

void BidirectionalSearcher::update(ActionResult r) {
  if(std::holds_alternative<ForwardResult>(r)) {
    auto fr = std::get<ForwardResult>(r);
    ExecutionState* fwd_cur = nullptr, *brnch_cur = nullptr;
    std::vector<ExecutionState*> fwd_added;
    std::vector<ExecutionState*> brnch_added;
    std::vector<ExecutionState*> fwd_removed;
    std::vector<ExecutionState*> brnch_removed;

    if(fr.current) {
      if(fr.current->isIsolated()) brnch_cur = fr.current;
      else fwd_cur = fr.current;
    }
    for(auto i : fr.addedStates) {
      if(i->isIsolated()) brnch_added.push_back(i);
      else fwd_added.push_back(i);
    }
    for(auto i : fr.removedStates) {
      if(i->isIsolated()) brnch_removed.push_back(i);
      else fwd_removed.push_back(i);
    }

    branch->update(brnch_cur, brnch_added, brnch_removed);
    auto reached = branch->collectAndClearReached();
    for(auto i : reached) {
        backward->addBranch(i);
    }
    forward->update(fwd_cur, fwd_added, fwd_removed);
    reached = forward->collectAndClearReached();
    for (auto i : reached) {
      backward->addBranch(i);
    }

  } else if (std::holds_alternative<BackwardResult>(r)) {
    auto br = std::get<BackwardResult>(r);
    backward->update(br.newPob);
    initializer->addPob(br.newPob);
  } else {
    auto ir = std::get<InitResult>(r);
    branch->update(nullptr, {ir.state}, {});
  }
}

BidirectionalSearcher::BidirectionalSearcher(SearcherConfig cfg) {
  ex = cfg.executor;
  forward = new GuidedForwardSearcher(constructUserSearcher(*cfg.executor));
  for(auto target : cfg.targets) {
    cfg.initial_state->targets.insert(target);
  }
  forward->update(nullptr,{cfg.initial_state},{});
  branch = new GuidedForwardSearcher(std::unique_ptr<ForwardSearcher>(new BFSSearcher()));
  backward = new BFSBackwardSearcher(cfg.targets);
  initializer = new ForkInitializer(cfg.targets);
}

} // namespace klee
