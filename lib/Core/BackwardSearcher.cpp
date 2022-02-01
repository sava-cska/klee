#include "BackwardSearcher.h"
#include "ExecutionState.h"
#include "SearcherUtil.h"

namespace klee {

void BFSBackwardSearcher::addBranch(ExecutionState* state) {
  for(auto i : pobs) {
    if(i->location->basicBlock == state->getPCBlock()) {
      auto state_copy = state->copy();
      backpropQueue.push(std::make_pair(i, state_copy));
    }
  }
}

bool BFSBackwardSearcher::empty() {
  return backpropQueue.empty();
}

void BFSBackwardSearcher::update(ProofObligation* pob) {
  pobs.insert(pob);
}

std::pair<ProofObligation *, ExecutionState *>
BFSBackwardSearcher::selectAction() {
  assert(!backpropQueue.empty());
  auto ret = backpropQueue.front();
  backpropQueue.pop();
  return ret;
}

void BFSBackwardSearcher::removePob(ProofObligation* pob) {
  pobs.erase(pob);
}

};
