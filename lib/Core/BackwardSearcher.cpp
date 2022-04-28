#include "BackwardSearcher.h"
#include "ExecutionState.h"
#include "SearcherUtil.h"
#include "klee/Module/KInstruction.h"
#include <algorithm>
#include <cstddef>
#include <utility>

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


bool BFSBackwardSearcher::empty() {
  for(auto pob : pobs) {
    Target t(pob->location, pob->at_return);
    auto states = emanager->states[t];
    for(auto state : states) {
      if(!used.count(std::make_pair(pob,state)) && checkStack(state, pob)) {
        return false;
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
  for (auto pob : pobs) {
    Target t(pob->location, pob->at_return);
    auto states = emanager->states[t];
    for (auto state : states) {
      if (!used.count(std::make_pair(pob,state)) && checkStack(state, pob)) {
        used.insert(std::make_pair(pob,state));
        return std::make_pair(pob, state);
      }
    }
  }
  return std::make_pair(nullptr, nullptr);
}

void BFSBackwardSearcher::removePob(ProofObligation* pob) {
  pobs.erase(pob);
}

};
