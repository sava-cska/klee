// -*- C++ -*-
#pragma once

#include "ExecutionState.h"
#include "ProofObligation.h"
#include "SearcherUtil.h"

#include <queue>
#include <unordered_set>

namespace klee {

bool checkStack(ExecutionState* state, ProofObligation* pob);

class BackwardSearcher {
public:

  virtual std::pair<ProofObligation*, ExecutionState*> selectAction() = 0;

  virtual void addBranch(ExecutionState* state) = 0;
  virtual void update(ProofObligation* pob) = 0;
  virtual void removePob(ProofObligation* pob) = 0;
  virtual bool empty() = 0;
};

class BFSBackwardSearcher : public BackwardSearcher {
public:

  ExecutionManager* emanager;

  void update(ProofObligation* pob) override;

  std::pair<ProofObligation*, ExecutionState*> selectAction() override;

  void addBranch(ExecutionState* state) override;

  bool empty() override;

  void removePob(ProofObligation* pob) override;

  BFSBackwardSearcher() {
  }

private:
  std::unordered_set<ProofObligation *> pobs;
  std::queue<std::pair<ProofObligation *, ExecutionState *>> backpropQueue;
  std::set<std::pair<ProofObligation*,ExecutionState*>> used;
};

};
