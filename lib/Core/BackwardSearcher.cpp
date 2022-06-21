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
  size_t range = std::min(state->stack.size(), pob->stack.size());
  auto stateIt = state->stack.rbegin();
  auto pobIt = pob->stack.rbegin();

  for (size_t i = 0; i < range; ++i) {
    KInstruction *stateInst = stateIt->caller;
    KInstruction *pobInst = *pobIt;
    if (stateInst != pobInst) {
      return false;
    }
    stateIt++;
    pobIt++;
  }
  return true;
}


bool RecencyRankedSearcher::empty() {
  return propagatePobToStates.empty();
}

void RecencyRankedSearcher::update(ProofObligation* pob) {
  pobs.push_back(pob);
  Target t(pob->location);
  std::unordered_set<ExecutionState *> &states = emanager.at(t);
  for (auto state : states) {
    if (checkStack(state, pob)) {
      propagatePobToStates[pob].insert(state);
    }
  }
}

std::pair<ProofObligation *, ExecutionState *>
RecencyRankedSearcher::selectAction() {
  auto &pobStates = *propagatePobToStates.begin();
  ProofObligation *pob = pobStates.first;
  auto &states = pobStates.second;
  Target t(pob->location);
  unsigned leastUsedCount = UINT_MAX;
  ExecutionState *leastUsedState = nullptr;
  for (ExecutionState *state : states) {
    if (pob->propagationCount[state] < leastUsedCount) {
      leastUsedCount = pob->propagationCount[state];
      leastUsedState = state;
    }
  }
  assert(leastUsedState);
  states.erase(leastUsedState);
  if (states.empty())
    propagatePobToStates.erase(pob);
  return std::make_pair(pob, leastUsedState);
}


void RecencyRankedSearcher::addState(Target target, ExecutionState *state) {
  if (state->isIsolated())
    emanager.insert(target, *state->copy());

  for (auto pob : pobs) {
    Target pobsTarget(pob->location);
    if (target == pobsTarget && checkStack(state, pob)) {
        assert(state->path.getFinalBlock() == pob->path.getInitialBlock() && "Paths are not compatible.");
        propagatePobToStates[pob].insert(state->copy());
    }
  }

}

void RecencyRankedSearcher::removePob(ProofObligation* pob) {
  auto pos = std::find(pobs.begin(), pobs.end(), pob);
  if (pos != pobs.end()) {
    pobs.erase(pos);
  }
  propagatePobToStates.erase(pob);
}

};
