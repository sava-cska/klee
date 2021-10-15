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
#include <vector>

#include <cstdlib>

template<typename S>
auto select_random(const S &s) {
  auto r = rand() % s.size();
  auto it = std::begin(s);
  std::advance(it,r);
  return *it;
}

namespace klee {

Action GuidedForwardSearcher::selectAction() {
  auto& state = searcher.selectState();
  return Action{Action::Type::Forward, state, nullptr, nullptr, nullptr};
}

std::vector<ExecutionState *>
GuidedForwardSearcher::update(ForwardResult result) {
  // Searcher.update(...) надо возвращать больше информации
}

std::vector<ExecutionState *>
updateTargets(const std::vector<KBlock *> &addedTargets,
            const std::vector<KBlock *> &removedTargets) {
  // Searcher.addTarget() Searcher.removeTarget()
}

Action GuidedBranchSearcher::selectAction() {
  auto searcher = select_random(searchers);
  auto& state = searcher->selectState();
  BranchMetadata* metadata = new BranchMetadata(searcher);
  return Action{Action::Type::Forward, state, nullptr, nullptr, metadata};
}

std::unordered_set<ExecutionState *>
GuidedBranchSearcher::update(ForwardResult result) {
  if(auto metadata = dyn_cast<BranchMetadata>(result.metadata)) {
    auto searcher = searchers.find(metadata->searcher);
    if(searcher != searchers.end()) {
      // update this Searcher
    }
  } else assert(false);
}

void GuidedBranchSearcher::setTargets(ExecutionState *from,
                                      std::unordered_set<KBlock *> to) {
  // GuidedSearcher* searcher = new GuidedSearcher(from);
  // for(auto i : to) searcher.addTarget(i);
  // searchers.insert(searcher);
}

} // namespace klee
