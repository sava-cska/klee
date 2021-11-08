//===-- BidirectionalSearcher.h ----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "BidirectionalSearcher.h"
#include "ExecutionState.h"
#include "klee/Core/Interpreter.h"
#include <unordered_set>
#include <vector>

#include <cstdlib>

namespace klee {

Action GuidedForwardSearcher::selectAction() {
  auto& state = searcher.selectState();
  return Action{Action::Type::Forward, &state, nullptr, nullptr};
}

std::unordered_set<ExecutionState *>
GuidedForwardSearcher::update(ForwardResult result) {
  searcher.update(result.current, result.addedStates, result.removedStates);
  return searcher.collectReached();
}

void GuidedForwardSearcher::updateTarget(KBlock *target, KBlock *from,
                                  KBlock *remove) {
  searcher.updateTarget(target,from,remove);
}

Action GuidedBranchSearcher::selectAction() {
  if(searcher.empty()) return Action{Action::Type::None, nullptr, nullptr, nullptr};
  
  auto& state = searcher.selectState();
  return Action{Action::Type::Forward, &state, nullptr, nullptr};
}

std::unordered_set<ExecutionState *>
GuidedBranchSearcher::update(ForwardResult result) {
  searcher.update(result.current, result.addedStates, result.removedStates);
  return searcher.collectReached();
}

void GuidedBranchSearcher::setTargets(ExecutionState *from,
                                      std::unordered_set<KBlock *> to) {
  for(auto i : to) {
    from->targets.insert(i);
  }
  searcher.update(nullptr, {from}, {});
}

Action BFSBackwardSearcher::selectAction() {
  if(!backpropQueue.empty()) {
    auto a = backpropQueue.front();
    backpropQueue.pop();
    return Action{Action::Type::Backward, a.second,  nullptr, a.first};
  }
  for(auto i : pobs) {
    auto loc = i->location;
    auto distmap = loc->parent->getBackwardDistance(loc);
    for(auto j: distmap) {
      if(initializedLocs.count(j.first)) continue;
      initializedLocs.insert(j.first);
      return Action{Action::Type::Init, nullptr, j.first, nullptr};
    }
    auto fdistmap = loc->parent->parent->getBackwardDistance(loc->parent);
    for(auto j : fdistmap) {
      if(initializedLocs.count(j.first->entryKBlock)) continue;
      initializedLocs.insert(j.first->entryKBlock);
      return Action{Action::Type::Init, nullptr, j.first->entryKBlock, nullptr};
    }
  }
  return Action{Action::Type::None, nullptr, nullptr, nullptr};
}

void BFSBackwardSearcher::addBranch(ExecutionState* state) {
  for(auto i : pobs) {
    if(i->location->basicBlock == state->getPCBlock()) {
      backpropQueue.push(std::make_pair(i, state));
    }
  }
}

void BFSBackwardSearcher::update(BackwardResult result) {
  pobs.insert(result.newPob);
}

Action BidirectionalSearcher::selectAction() {
  while (true) {
    int choice = rand() % 3;
    if (choice == 0) {
      return forwardSearcher->selectAction();
    }
    if (choice == 1) {
      Action a = branchSearcher->selectAction();
      if (a.type != Action::Type::None)
        return a;
    }
    if(choice == 2) {
      Action a = backwardSearcher->selectAction();
      if (a.type != Action::Type::None)
        return a;
    }
  }
  // Что-то адекватное сюда
}

} // namespace klee
