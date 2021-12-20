// -*- C++ -*-
#pragma once

#include "ExecutionState.h"
#include "ProofObligation.h"
#include "SearcherUtil.h"

#include <queue>
#include <unordered_set>

namespace klee {

class BackwardSearcher {
public:

  virtual std::pair<ProofObligation*, ExecutionState*> selectAction() = 0;

  virtual void addBranch(ExecutionState* state) = 0;
  virtual void update(ProofObligation* pob) = 0;
  virtual bool empty() = 0;  
};

class BFSBackwardSearcher : BackwardSearcher {
public:

  void update(ProofObligation* pob) override;
  
  std::pair<ProofObligation*, ExecutionState*> selectAction() override;

  void addBranch(ExecutionState* state) override;

  bool empty() override;

  BFSBackwardSearcher(std::unordered_set<KBlock *> locations) {
    for (auto location : locations) {
      pobs.insert(new ProofObligation(location));
    }
  }

private:
  std::unordered_set<ProofObligation *> pobs;
  std::queue<std::pair<ProofObligation *, ExecutionState *>> backpropQueue;
};

};
