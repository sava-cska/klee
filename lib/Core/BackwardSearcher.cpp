#include "BackwardSearcher.h"
#include "ExecutionState.h"
#include "SearcherUtil.h"

namespace klee {

void BFSBackwardSearcher::addBranch(ExecutionState* state) {
  for(auto i : pobs) {
    if(i->location->basicBlock == state->getPCBlock()) {
      auto state_copy = state->copy();
      backpropQueue.push_back(std::make_pair(i, state_copy));
    }
  }
}

bool BFSBackwardSearcher::empty() {
  if(!backpropQueue.empty())
    return false;
  else {
    for(auto i : pobs) {
      for(auto j : emanager->states[i->location->basicBlock]) {
        if(!used.count(std::make_pair(i, j)))
          return false;
      }
    }
  }
  return true;
}

void BFSBackwardSearcher::update(ProofObligation* pob) {
  pobs.push_back(pob);
}

std::pair<ProofObligation *, ExecutionState *>
BFSBackwardSearcher::selectAction() {
  if(backpropQueue.empty()) {
    for(auto i : pobs) {
      for(auto j : emanager->states[i->location->basicBlock]) {
        if(!used.count(std::make_pair(i, j))) {
          used.insert(std::make_pair(i, j));
          return std::make_pair(i, j);
        }
      }
    }
  }
  auto ret = backpropQueue.front();
  backpropQueue.erase(backpropQueue.begin());
  return ret;
}

void BFSBackwardSearcher::removePob(ProofObligation* pob) {
  auto ip = std::find(pobs.begin(), pobs.end(), pob);
  if (ip != pobs.end())
    pobs.erase(ip);
}

};
