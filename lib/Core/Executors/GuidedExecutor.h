//===-- GuidedExecutor.h ----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Class to perform actual execution, hides implementation details from external
// interpreter.
//
//===----------------------------------------------------------------------===//

#pragma once

#include "BaseExecutor.h"
#include "../Tracker.h"

namespace klee {

class GuidedExecutor : public BaseExecutor {
public:
  typedef std::pair<llvm::BasicBlock*, llvm::BasicBlock*> BasicBlockPair;
  typedef std::map<llvm::BasicBlock*, std::set<ExecutionState*, ExecutionStateIDCompare> > ExecutedInterval;
  typedef std::map<llvm::BasicBlock*, std::unordered_set<llvm::BasicBlock*> > VisitedBlock;
  typedef std::map<llvm::BasicBlock*, std::unordered_set<Transition, BasicBlockPairHash>> VisitedTransition;
  struct ExecutionBlockResult {
    ExecutedInterval redundantStates;
    ExecutedInterval completedStates;
    ExecutedInterval pausedStates;
    ExecutedInterval erroneousStates;
    VisitedBlock history;
    VisitedTransition transitionHistory;
  };
  typedef std::map<llvm::BasicBlock*, ExecutionBlockResult> ExecutionResult;

private:
  std::map<llvm::Function *, std::unordered_set<ExecutionState *>> targetableStates;
  Tracker reachabilityTracker;

  ExecutionResult results;

  void addCompletedResult(ExecutionState &state);
  void addErroneousResult(ExecutionState &state);
  void addHistoryResult(ExecutionState &state);
  void addTargetable(ExecutionState &state);
  void removeTargetable(ExecutionState &state);
  bool isTargetable(ExecutionState &state);

  void targetedRun(ExecutionState &initialState, KBlock *target);
  void guidedRun(ExecutionState &initialState);

  void run(ExecutionState &initialState) override;
  void runWithTarget(ExecutionState &state, KBlock *target);
  void runGuided(ExecutionState &state);
  void runCovered(ExecutionState &state);

  void initializeRoot(ExecutionState &state, KBlock *kb);

  // pause state
  void pauseState(ExecutionState &state);
  void pauseRedundantState(ExecutionState &state);
  // unpause state
  void unpauseState(ExecutionState &state);

  void reachTarget(ExecutionState const &initialState, KBlock const &target, size_t lvl_bound);
  void goFront(ExecutionState &state, std::queue<ExecutionState *> &forwardQueue);
  void goBack(ExecutionState &state, ProofObligation &pob, std::deque<ProofObligation> &backwardQueue, ExecutionState const &entryPoint);

  void markUnblocked(ExecutionState &state, ProofObligation &pob) const;

  KBlock * getStartLocation(ExecutionState &state);
  KBlock * getLastExecutedLocation(ExecutionState &state);
  KBlock * getCurrentLocation(ExecutionState &state);
protected:
  void actionBeforeStateTerminating(ExecutionState &state, TerminateReason reason) override;

public:
  GuidedExecutor(llvm::LLVMContext &ctx, const InterpreterOptions &opts,
      InterpreterHandler *ie);

  virtual ~GuidedExecutor() = default;

  const InterpreterHandler& getHandler() {
    return *interpreterHandler;
  }

  void runMainWithTarget(llvm::Function *mainFn, llvm::BasicBlock *target,
                         int argc, char **argv, char **envp);

  bool tryBoundedExecuteStep(ExecutionState &state, unsigned bound);
  void isolatedExecuteStep(ExecutionState &state);
  bool tryExploreStep(ExecutionState &state, ExecutionState &initialState);
  void composeStep(ExecutionState &state);
  void executeReturn(ExecutionState &state, KInstruction *ki);
  KBlock *calculateCoverTarget(ExecutionState &state);
  KBlock *calculateTarget(ExecutionState &state);
  void calculateTargetedStates(llvm::BasicBlock *initialBlock,
                               ExecutedInterval &pausedStates,
                               std::map<KBlock*, std::vector<ExecutionState*>> &targetedStates);
  void initializeRoots(ExecutionState *initialState);
  void addState(ExecutionState &state);
};
  
} // namespace klee
