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

namespace klee {

class ExecutionState;

struct Action {
  enum class Type { Init, Forward, Backward, None };

  Type type;
  ExecutionState* state; // Forward, Backward, Init
  KBlock *location;      // Init
  ProofObligation *pob;  // Backward
};

struct ForwardResult {
  ExecutionState *current;
  std::vector<ExecutionState *> &addedStates;
  std::vector<ExecutionState *> &removedStates;
  ForwardResult(ExecutionState *_s, std::vector<ExecutionState *> &a,
                std::vector<ExecutionState *> &r)
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

class BidirectionalSearcher {
public:
  Action selectAction();

private:
  std::unique_ptr<IForwardSearcher> forwardSearcher;
  std::unique_ptr<IBranchSearcher> branchSearcher;
  std::unique_ptr<IBackwardSearcher> backwardSearcher;
};


class GuidedForwardSearcher : IForwardSearcher {
public:
  Action selectAction() override;

  std::unordered_set<ExecutionState *>
  update(ForwardResult result) override;

  void updateTarget(KBlock *target, KBlock *from, KBlock *remove) override;

private:
  GuidedSearcher searcher;
  KBlock* EP; // Хотим иногда снова стартануть из начального состояния
};

class GuidedBranchSearcher : IBranchSearcher {
public:
  Action selectAction() override;

  std::unordered_set<ExecutionState *> update(ForwardResult result) override;

  void setTargets(ExecutionState *from,
                  std::unordered_set<KBlock *> to) override;

private:
  GuidedSearcher searcher;
};

class BFSBackwardSearcher : IBackwardSearcher {
public:
  Action selectAction() override;

  void addBranch(ExecutionState *state) override;

  void update(BackwardResult) override;
  
private:
  std::unordered_set<ProofObligation*> pobs;
  std::queue<std::pair<ProofObligation*, ExecutionState*>> backpropQueue;
  std::unordered_set<KBlock*> initializedLocs;
  KModule* module;
};

} // namespace klee
