#include "BackwardSearcher.h"
#include "ExecutionState.h"
#include "SearcherUtil.h"
#include "klee/Module/KInstruction.h"
#include <algorithm>
#include <cstddef>

namespace klee {

bool checkStack(ExecutionState *state, ProofObligation *pob) {
  size_t range = std::min(state->stack.size() - 1, pob->stack.size());
  auto state_it = state->stack.rbegin();
  auto pob_it = pob->stack.rbegin();
  for (size_t i = 0; i < range; i++) {
    KInstruction* state_inst = state_it->caller;
    KInstruction* pob_inst = *pob_it;
    if(state_inst != pob_inst) {
      return false;
    }
    state_it++;
    pob_it++;
  }
  return true;
}

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
  return true;
}

void BFSBackwardSearcher::update(ProofObligation* pob) {
  pobs.insert(pob);
}

std::pair<ProofObligation *, ExecutionState *>
BFSBackwardSearcher::selectAction() {
  auto ret = backpropQueue.front();
  backpropQueue.pop();
  return ret;
}

void BFSBackwardSearcher::removePob(ProofObligation* pob) {
  pobs.erase(pob);
}

};
