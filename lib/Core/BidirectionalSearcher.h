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
#include "Initializer.h"
#include "SearcherUtil.h"
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
  virtual Action& selectAction() = 0;
  virtual void update(ActionResult) = 0;
  virtual void closeProofObligation(ProofObligation *) = 0;
  virtual bool empty() = 0;
};


class BidirectionalSearcher : public IBidirectionalSearcher {
public:
  Action &selectAction() override;
  void update(ActionResult) override;
  void closeProofObligation(ProofObligation *) override;
  bool empty() override;
  explicit BidirectionalSearcher(const SearcherConfig &);

private:
  enum class StepKind { Initialize, Forward, Branch, Backward, Terminate };

  Executor *ex; // hack

  GuidedSearcher *forward;
  GuidedSearcher *branch;
  BFSBackwardSearcher *backward;
  ValidityCoreInitializer *initializer;

  Ticker ticker;

  // Temporary _-_
  std::unordered_set<llvm::BasicBlock *> mainLocs;

  StepKind selectStep();
  void removePob(ProofObligation *);
  bool isStuck(ExecutionState &);
};

} // namespace klee
