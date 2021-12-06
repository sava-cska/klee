//===-- BidirectionalSearcher.h ---------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#pragma once
#include "Executor.h"
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
  
};

class ForwardBidirSearcher : public IBidirectionalSearcher {
public:
  Action selectAction() override;
  void update(ActionResult) override;

  ForwardBidirSearcher(SearcherConfig);
  
private:
  GuidedForwardSearcher* searcher;
  Executor* ex; // hack
};

// class BidirectionalSearcher : public IBidirectionalSearcher {
// public:
//   Action selectAction() override;
//   void update(ActionResult) override;
//   bool empty() override;

//   BidirectionalSearcher(SearcherConfig);

// private:
//   std::unique_ptr<GuidedForwardSearcher> forwardSearcher;
//   std::unique_ptr<GuidedForwardSearcher> branchSearcher;
//   std::unique_ptr<BFSBackwardSearcher> backwardSearcher;
// };

} // namespace klee
