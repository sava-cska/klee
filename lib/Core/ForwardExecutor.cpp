//===-- ForwardExecutor.cpp ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "ForwardExecutor.h"

#include "Searcher.h"

using namespace llvm;

namespace klee {

ForwardExecutor::ForwardExecutor(llvm::LLVMContext &ctx, const InterpreterOptions &opts,
                                 InterpreterHandler *ie)
    : BaseExecutor(ctx, opts, ie)
{}

void ForwardExecutor::run(ExecutionState &initialState) {
  // Delay init till now so that ticks don't accrue during optimization and such.
  timers.reset();

  states.insert(&initialState);

  if (usingSeeds) {
    seed(initialState);
  }

  searcher = constructUserSearcher(*this);

  std::vector<ExecutionState *> newStates(states.begin(), states.end());
  searcher->update(0, newStates, std::vector<ExecutionState *>());

  // main interpreter loop
  while (!states.empty() && !haltExecution) {
    ExecutionState &state = searcher->selectState();
    executeStep(state);
  }

  searcher.reset();

  doDumpStates();
  haltExecution = false;
}

} // namespace klee
