//===-- BidirectionalSearcher.h ---------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#pragma once
#include "BackwardSearcher.h"
#include "Executor.h"
#include "SearcherUtil.h"
#include "Initializer.h"
#include "ProofObligation.h"
#include "ForwardSearcher.h"
#include "klee/Module/KModule.h"
#include <memory>
#include <unordered_set>
#include <vector>
#include <variant>


namespace klee {

class IBidirectionalSearcher {
public:
  virtual Action &selectAction() = 0;
  virtual void update(ActionResult) = 0;
  virtual void closeProofObligation(ProofObligation *) = 0;
  virtual bool empty() = 0;
  virtual ~IBidirectionalSearcher() {}
};


class BidirectionalSearcher : public IBidirectionalSearcher {
public:
  Action &selectAction() override;
  void update(ActionResult) override;
  void closeProofObligation(ProofObligation *) override;
  bool empty() override;
  explicit BidirectionalSearcher(const SearcherConfig &);
  ~BidirectionalSearcher() override;

private:
  enum class StepKind { Initialize, Forward, Branch, Backward, Terminate };

  Executor *ex; // hack

  GuidedSearcher *forward;
  GuidedSearcher *branch;
  RecencyRankedSearcher *backward;
  ConflictCoreInitializer *initializer;

  std::vector<ProofObligation *> pobs;

  Ticker ticker;

  // Temporary _-_
  std::unordered_set<llvm::BasicBlock *> rootBlocks;
  std::unordered_set<llvm::BasicBlock *> reachableBlocks;

  StepKind selectStep();
  void addPob(ProofObligation *);
  void removePob(ProofObligation *);
  void answerPob(ProofObligation *);
  bool isStuck(ExecutionState &);
};

} // namespace klee
