#include "BackwardSearcher.h"
#include "ExecutionState.h"
#include "SearcherUtil.h"
#include "klee/Module/KInstruction.h"
#include <algorithm>
#include <climits>
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


bool RecencyRankedSearcher::empty() {
  for(auto pob : pobs) {
    Target t(pob->location, pob->atReturn);
    std::unordered_set<ExecutionState *> &states = emanager->at(t);
    for(auto state : states) {
      if(!used.count(std::make_pair(pob,state)) && checkStack(state, pob)) {
        return false;
      }
    }
  }
  return true;
}

void RecencyRankedSearcher::update(ProofObligation* pob) {
  pobs.push_back(pob);
}

std::pair<ProofObligation *, ExecutionState *>
RecencyRankedSearcher::selectAction() {
  for (auto pob : pobs) {
    Target t(pob->location, pob->atReturn);
    std::unordered_set<ExecutionState *> &states = emanager->at(t);
    unsigned least_used_count = UINT_MAX;
    ExecutionState *least_used_state = nullptr;
    for (auto state : states) {
      if (!used.count(std::make_pair(pob,state)) && checkStack(state, pob)) {
        if(pob->propagationCount[state] < least_used_count) {
          least_used_count = pob->propagationCount[state];
          least_used_state = state;
        }
      }
    }
    if(least_used_state) {
      used.insert(std::make_pair(pob, least_used_state));
      return std::make_pair(pob, least_used_state);
    }
  }
  return std::make_pair(nullptr, nullptr);
}

void RecencyRankedSearcher::removePob(ProofObligation* pob) {
  auto pos = std::find(pobs.begin(), pobs.end(), pob);
  if(pos != pobs.end()) {
    pobs.erase(pos);
  }
}

};
