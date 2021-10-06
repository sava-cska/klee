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

namespace klee {

class ExecutionState;

struct Action {
  enum class Type { Init, Forward, Backward };
  
  Type type;
  ExecutionState* state; // Forward, Backward
  KBlock* location;      // Init
  ProofObligation* pob;  // Backward
};

/// A BidirectionalSearcher implements an exploration strategy for the Executor by selecting
/// states for further exploration using different strategies or heuristics.
class BidirectionalSearcher {
public:
  virtual ~BidirectionalSearcher() = default;

  /// Selects a state for further exploration.
  /// \return The selected state.
  virtual Action selectState() = 0;

  // /// Notifies BidirectionalSearcher about new or deleted states.
  // /// \param current The currently selected state for exploration.
  // /// \param addedStates The newly branched states with `current` as common ancestor.
  // /// \param removedStates The states that will be terminated.
  // virtual void update(ExecutionState *current,
  //                     const std::vector<ExecutionState *> &addedStates,
  //                     const std::vector<ExecutionState *> &removedStates) = 0;

  /// \return True if no state left for exploration, False otherwise
  virtual bool empty() = 0;

  // /// Prints name of BidirectionalSearcher as a `klee_message()`.
  // // TODO: could probably made prettier or more flexible
  // virtual void printName(llvm::raw_ostream &os) = 0;
};

class SimpleBidirectionalSearcher : public BidirectionalSearcher {
public:
  virtual ~SimpleBidirectionalSearcher() = default;
  Action selectState() override;
  bool empty() override;
};

} // klee namespace
