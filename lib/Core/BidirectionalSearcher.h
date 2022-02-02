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

  virtual Action selectAction() = 0;
  virtual void update(ActionResult) = 0;
  virtual void closeProofObligation(ProofObligation*) = 0;

};

class ForwardBidirectionalSearcher : public IBidirectionalSearcher {
public:
  Action selectAction() override;
  void update(ActionResult) override;
  void closeProofObligation(ProofObligation*) override;

  explicit ForwardBidirectionalSearcher(SearcherConfig);

private:
  GuidedForwardSearcher* searcher;
  Executor* ex; // hack
};

class BidirectionalSearcher : public IBidirectionalSearcher {
public:
  Action selectAction() override;
  void update(ActionResult) override;
  void closeProofObligation(ProofObligation*) override;

  explicit BidirectionalSearcher(SearcherConfig);

private:

  Executor* ex; // hack
  GuidedForwardSearcher* forward;
  GuidedForwardSearcher* branch;
  BFSBackwardSearcher* backward;
  ValidityCoreInitializer* initializer;
  uint choice = 0;

  // Temporary _-_
  std::unordered_set<KBlock*> known_locs;
};

} // namespace klee
