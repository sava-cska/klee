//===-- BidirectionalSearcher.h ---------------------------------*- C++ -*-===//
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
#include "klee/Module/KModule.h"
#include <memory>
#include <unordered_set>
#include <vector>

namespace klee {

class ExecutionState;
class BidirectionalExecutor;

struct Action {
  enum class Type { Init, Forward, Backward, None };
  enum class SearcherType { Forward, Backward, Branch };

  Type type;
  SearcherType searcher;
  ExecutionState* state; // Forward, Backward
  KBlock *location;      // Init
  ProofObligation *pob;  // Backward
  std::unordered_set<KBlock*> targets; // Init
};

struct SearcherConfig {
  ExecutionState* initial_state;
  std::unordered_set<KBlock*> targets;
  // Hack
  BidirectionalExecutor* executor;
};

struct ForwardResult {
  ExecutionState *current;
  // references to vectors?
  std::vector<ExecutionState *> addedStates;
  std::vector<ExecutionState *> removedStates;
  ForwardResult(ExecutionState *_s, std::vector<ExecutionState *> a,
                std::vector<ExecutionState *> r)
      : current(_s), addedStates(a), removedStates(r){};
};

struct BackwardResult {
  ProofObligation *newPob;
  ProofObligation *oldPob;
  BackwardResult(ProofObligation *_newPob, ProofObligation *_oldPob)
      : newPob(_newPob), oldPob(_oldPob) {}
};

class IForwardSearcher {
public:
  virtual Action selectAction() = 0;

  virtual std::unordered_set<ExecutionState *>
  update(ForwardResult) = 0;

  virtual void updateTarget(KBlock *target, KBlock *from, KBlock *remove) = 0;
};

class IBranchSearcher {
public:
  
  virtual Action selectAction() = 0;

  // Вариант Саши?
  virtual void setTargets(ExecutionState *from,
                          std::unordered_set<KBlock *> to) = 0;
  
  virtual std::unordered_set<ExecutionState *>
  update(ForwardResult) = 0;
};

class IBackwardSearcher {
public:
  virtual Action selectAction() = 0;

  virtual void addBranch(ExecutionState *state) = 0;

  virtual void update(BackwardResult) = 0;
};

class GuidedForwardSearcher : IForwardSearcher {
public:
  Action selectAction() override;

  // hotfix
  bool empty() { return searcher->empty(); }

  std::unordered_set<ExecutionState *>
  update(ForwardResult result) override;

  void updateTarget(KBlock *target, KBlock *from, KBlock *remove) override;

  GuidedForwardSearcher(GuidedSearcher* searcher) : searcher(searcher) {}

private:
  GuidedSearcher* searcher;
};

class GuidedBranchSearcher : IBranchSearcher {
public:
  Action selectAction() override;

  std::unordered_set<ExecutionState *> update(ForwardResult result) override;

  void setTargets(ExecutionState *from,
                  std::unordered_set<KBlock *> to) override;

  GuidedBranchSearcher(GuidedSearcher* searcher) : searcher(searcher) {}

private:
  GuidedSearcher* searcher;
};

class BFSBackwardSearcher : IBackwardSearcher {
public:
  Action selectAction() override;

  void addBranch(ExecutionState *state) override;

  void update(BackwardResult) override;

  BFSBackwardSearcher(std::unordered_set<KBlock *> locations) {
    for (auto location : locations) {
      pobs.insert(new ProofObligation(location));
    }
  }

private:
  std::unordered_set<ProofObligation*> pobs;
  std::unordered_set<KBlock*> targets;
  std::queue<std::pair<ProofObligation*, ExecutionState*>> backpropQueue;
  std::unordered_set<KBlock*> initializedLocs;
};

class BidirectionalSearcher {
public:
  Action selectAction();
  bool empty();

  BidirectionalSearcher(SearcherConfig);

  std::unique_ptr<GuidedForwardSearcher> forwardSearcher;
  std::unique_ptr<GuidedBranchSearcher> branchSearcher;
  std::unique_ptr<BFSBackwardSearcher> backwardSearcher;
};

} // namespace klee
