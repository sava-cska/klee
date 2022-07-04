// -*- C++ -*-
#pragma once

#include "ExecutionState.h"
#include "ProofObligation.h"
#include "SearcherUtil.h"

#include <map>
#include <queue>
#include <unordered_set>

namespace klee {

bool checkStack(ExecutionState *state, ProofObligation *pob);

class BackwardSearcher {
public:
  virtual ~BackwardSearcher() = default;
  virtual std::pair<ProofObligation *, ExecutionState *> selectAction() = 0;
  virtual void addState(Target target, ExecutionState *state) = 0;
  virtual void update(ProofObligation *pob) = 0;
  virtual void removePob(ProofObligation *pob) = 0;
  virtual bool empty() = 0;
};

class RecencyRankedSearcher : public BackwardSearcher {
private:
  std::vector<ProofObligation *> pobs;
  std::set<std::pair<ProofObligation *, ExecutionState *>> used;
  std::map<ProofObligation *,
           std::set<ExecutionState *, ExecutionStateIDCompare>,
           ProofObligationIDCompare>
      propagatePobToStates;
  ExecutionManager emanager;
  unsigned maxPropagations;

public:
  RecencyRankedSearcher(unsigned _maxPropagations);
  std::pair<ProofObligation *, ExecutionState *> selectAction() override;
  void addState(Target target, ExecutionState *state) override;
  void update(ProofObligation *pob) override;
  void removePob(ProofObligation *pob) override;
  bool empty() override;
};

}; // namespace klee
