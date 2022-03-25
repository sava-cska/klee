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

Action BidirectionalSearcher::selectAction() {
  while (true) {
    if(forward->empty() && branch->empty() && backward->empty() && initializer->empty())
      return Action(Action::Type::Terminate, nullptr, nullptr, nullptr, {});
    choice = (choice + 1) % 4;
    if (choice == 0) {
      if(forward->empty()) continue;
      auto &state = forward->selectState();
      return Action(&state);
    }
    if (choice == 1) {
      if (branch->empty()) continue;
      auto& state = branch->selectState();
      return Action(&state);
    }
    if (choice == 2) {
      if (backward->empty()) continue;
      auto a = backward->selectAction();
      return Action(Action::Type::Backward, a.second, nullptr, a.first, {});
    }
    if (choice == 3) {
      if(initializer->empty()) continue;
      auto a = initializer->selectAction();
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

    if(fr.validity_core_init.first != nullptr) {
      initializer->addValidityCoreInit(fr.validity_core_init);
    }

  } else if (std::holds_alternative<BackwardResult>(r)) {
    auto br = std::get<BackwardResult>(r);
    for(auto i : br.newPobs) {
      backward->update(i);
      initializer->addPob(i);
    }
  } else {
    auto ir = std::get<InitResult>(r);
    branch->update(nullptr, {ir.state}, {});
  }
}

BidirectionalSearcher::BidirectionalSearcher(SearcherConfig cfg) {
  ex = cfg.executor;
  forward = new GuidedForwardSearcher(constructUserSearcher(*cfg.executor), true);
  forward->update(nullptr,{cfg.initial_state},{});
  branch = new GuidedForwardSearcher(std::unique_ptr<ForwardSearcher>(new BFSSearcher()), false);
  backward = new BFSBackwardSearcher;
  initializer = new ValidityCoreInitializer;
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

} // namespace klee
