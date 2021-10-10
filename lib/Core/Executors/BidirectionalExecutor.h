// -*- C++ -*-
#pragma once

#include "BaseExecutor.h"
#include "../Tracker.h"

#include <queue>

namespace klee {

class BidirectionalExecutor : public BaseExecutor {
public:
  typedef std::pair<llvm::BasicBlock *, llvm::BasicBlock *> BasicBlockPair;
  typedef std::map<llvm::BasicBlock *,
                   std::set<ExecutionState *, ExecutionStateIDCompare>>
      ExecutedInterval;
  typedef std::map<llvm::BasicBlock *, std::unordered_set<llvm::BasicBlock *>>
      VisitedBlock;
  typedef std::map<llvm::BasicBlock *,
                   std::unordered_set<Transition, BasicBlockPairHash>>
      VisitedTransition;

  struct ExecutionBlockResult {
    ExecutedInterval redundantStates;
    ExecutedInterval completedStates;
    ExecutedInterval pausedStates;
    ExecutedInterval erroneousStates;
    VisitedBlock history;
    VisitedTransition transitionHistory;
  };

  typedef std::map<llvm::BasicBlock*, ExecutionBlockResult> ExecutionResult;

  
public:
  std::map<llvm::Function *, std::unordered_set<ExecutionState *>> targetableStates;
  std::unique_ptr<Tracker> reachabilityTracker;
  
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
  void pauseState(ExecutionState &state);
  void pauseRedundantState(ExecutionState &state);
  void unpauseState(ExecutionState &state);
  void silentRemove(ExecutionState &state);

  void reachTarget(ExecutionState const &initialState, KBlock const &target,
                   size_t lvl_bound);
  void goFront(ExecutionState &state,
               std::queue<ExecutionState *> &forwardQueue);
  void goBack(ExecutionState &state, ProofObligation &pob,
              std::deque<ProofObligation> &backwardQueue,
              ExecutionState const &entryPoint);

  KBlock *getStartLocation(const ExecutionState &state);
  KBlock *getLastExecutedLocation(const ExecutionState &state);
  KBlock *getCurrentLocation(const ExecutionState &state);

  void actionBeforeStateTerminating(ExecutionState &state,
                                    TerminateReason reason) override;

  BidirectionalExecutor(llvm::LLVMContext &ctx, const InterpreterOptions &opts,
                 InterpreterHandler *ie);

  virtual ~BidirectionalExecutor() = default;

  const InterpreterHandler &getHandler() { return *interpreterHandler; }

  void runMainWithTarget(llvm::Function *mainFn, llvm::BasicBlock *target,
                         int argc, char **argv, char **envp);

  bool tryBoundedExecuteStep(ExecutionState &state, unsigned bound);
  void isolatedExecuteStep(ExecutionState &state);
  bool tryExploreStep(ExecutionState &state, ExecutionState &initialState);
  void composeStep(ExecutionState &state);
  void executeReturn(ExecutionState &state, KInstruction *ki);
  KBlock *calculateCoverTarget(ExecutionState &state);
  KBlock *calculateTarget(ExecutionState &state);
  void calculateTargetedStates(
      llvm::BasicBlock *initialBlock, ExecutedInterval &pausedStates,
      std::map<KBlock *, std::vector<ExecutionState *>> &targetedStates);
  void initializeRoots(ExecutionState *initialState);
  void addState(ExecutionState &state);
};
} // namespace klee
