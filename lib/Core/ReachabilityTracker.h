#ifndef KLEE_REACHABILITYTRACKER_H
#define KLEE_REACHABILITYTRACKER_H

#include "ExecutionState.h"
#include "ProofObligation.h"

namespace klee {
class ReachabilityTracker {
public:
  ReachabilityTracker() = default;

  void createRunningStateToTarget(const ExecutionState *state,
                                  const Target &target);
  void addRunningStateToTarget(const ExecutionState *state,
                               const Target &target);
  void removeRunningStateToTarget(const ExecutionState *state,
                                  const Target &target);
  bool emptyRunningStateToTarget(const Target &target) const;

  void addWaitingStateToTarget(const ExecutionState *state,
                               const Target &target);
  void removeWaitingStateToPob(const ExecutionState *state,
                               ProofObligation *pob);

  bool isPobProcessAllStates(ProofObligation *pob) const;

  void addUnreachableRootTarget(const Target &target);
  bool checkIsRootTargetUnreachable(const Target &target) const;

  void updateFinishPob(ProofObligation *pob);

private:
  std::map<Target, std::set<const ExecutionState *>> runningStateToTarget;
  std::map<Target, std::set<const ExecutionState *>> waitingStateToTarget;
  std::map<ProofObligation *, std::set<const ExecutionState *>>
      waitingStateToPob;
  std::set<ProofObligation *> blockedPob;

  std::set<Target> unreachableRootTarget;
};
} // namespace klee

#endif
