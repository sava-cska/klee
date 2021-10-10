//===-- BidirectionalSearcher.h ----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#pragma once
#include "ProofObligation.h"
#include "Searcher.h"
#include <unordered_set>

namespace klee {

class ExecutionState;

struct Action {
  enum class Type { Init, Forward, Backward };
  
  Type type;
  ExecutionState* state; // Forward, Backward, Init
  KBlock* location;      // Init
  ProofObligation* pob;  // Backward
};

// Так мало информации?
class ForwardSearcher {
public:
  Action selectAction();
  void update(/*?*/);

private:
  
};
  
// Слишком много концептуально разных вещей с названием Searcher...
class BranchSearcher {
public:
  Action selectAction();
  void setTargets(ExecutionState* from, std::unordered_set<KBlock*> to);
  std::unordered_set<ExecutionState*> update(/*?*/);
  
private:
  std::vector<std::pair<ExecutionState*, KBlock*>> destinations;
};

class BackwardSearcher {
public:
  Action selectAction();
  std::unordered_set<ProofObligation*> addBranch(ExecutionState* state);
  void update(/*?*/);
};


class BidirectionalSearcher {
public:
  Action selectAction();
  // updatePobs
  // updateStates
private:
  ForwardSearcher forwardSearcher;
  BranchSearcher branchSearcher;
  BackwardSearcher backwardSearcher;

};

} // namespace klee

