// -*- C++ -*-
#pragma once

#include "ProofObligation.h"
#include "SearcherUtil.h"

#include <queue>
#include <unordered_set>

namespace klee {

class BackwardSearcher {
public:
  
  virtual void update(ProofObligation* pob) = 0;
  virtual bool empty() = 0;
};

class BFSBackwardSearcher : BackwardSearcher {
// public:
//   // Action selectAction() override;

//   // void addBranch(ExecutionState *state) override;

//   // void update(BackwardResult) override;
//   // void remove(ProofObligation* pob) override;

//   BFSBackwardSearcher(std::unordered_set<KBlock *> locations) {
//     for (auto location : locations) {
//       pobs.insert(new ProofObligation(location));
//     }
//   }

// private:
//   std::unordered_set<ProofObligation *> pobs;
//   std::unordered_set<KBlock *> targets;
//   std::queue<std::pair<ProofObligation *, ExecutionState *>> backpropQueue;
//   std::unordered_set<KBlock *> initializedLocs;
};

};
