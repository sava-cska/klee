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
  if(!backpropQueue.empty()) return false;
  if(backpropQueue.empty()) {
    for(auto i : pobs) {
      for(auto j : emanager->states[i->location->basicBlock]) {
        if(!used.count(std::make_pair(i,j))) return false;
      }
    }
  }
  return true;
}

void BFSBackwardSearcher::update(ProofObligation* pob) {
  pobs.insert(pob);
}

std::pair<ProofObligation *, ExecutionState *>
BFSBackwardSearcher::selectAction() {
  if(backpropQueue.empty()) {
    for(auto i : pobs) {
      for(auto j : emanager->states[i->location->basicBlock]) {
        if(!used.count(std::make_pair(i,j))) {
          used.insert(std::make_pair(i,j));
          return std::make_pair(i,j);
        }
      }
    }
  }
  auto ret = backpropQueue.front();
  backpropQueue.pop();
  return ret;
}

void BFSBackwardSearcher::removePob(ProofObligation* pob) {
  pobs.erase(pob);
}

};
