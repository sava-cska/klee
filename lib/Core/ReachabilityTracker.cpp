#include "ReachabilityTracker.h"
#include "klee/Support/ErrorHandling.h"

namespace klee {
void ReachabilityTracker::removePob(ProofObligation *pob) {
  waitingStateToPob.erase(pob);
  blockedPob.erase(pob);
}

bool ReachabilityTracker::isPobProcessAllStates(ProofObligation *pob) const {
  return blockedPob.find(pob) != blockedPob.end();
}

void ReachabilityTracker::createRunningStateToTarget(
    const ExecutionState *state, const Target &target) {
  std::string s;
  llvm::raw_string_ostream ss(s);
  target.block->basicBlock->printAsOperand(ss, false);
  //klee_message("createRunningStateToTarget: state %s\n", ss.str().c_str());
  //klee_message("Start location: %s",
  //             Target(state->initPC->parent).print().c_str());
  // assert(runningStateToTarget[target].find(state->initPC->parent) ==
  //        runningStateToTarget[target].end());
  runningStateToTarget[target].insert(state);
  if (waitingStateToTarget.find(target) == waitingStateToTarget.end()) {
    waitingStateToTarget.insert({target, {}});
  }
}

void ReachabilityTracker::addRunningStateToTarget(const ExecutionState *state,
                                                  const Target &target) {
  std::string s;
  llvm::raw_string_ostream ss(s);
  target.block->basicBlock->printAsOperand(ss, false);
  //klee_message("addRunningStateToTarget: state %s\n", ss.str().c_str());
  //klee_message("Start location: %s",
  //             Target(state->initPC->parent).print().c_str());
  assert(runningStateToTarget.find(target) != runningStateToTarget.end() &&
         runningStateToTarget[target].find(state) ==
             runningStateToTarget[target].end());
  runningStateToTarget[target].insert(state);
}

void ReachabilityTracker::removeRunningStateToTarget(
    const ExecutionState *state, const Target &target) {
  std::string s;
  llvm::raw_string_ostream ss(s);
  target.block->basicBlock->printAsOperand(ss, false);
  //klee_message("removeRunningStateToTarget: state %s\n", ss.str().c_str());
  //klee_message("Start location: %s",
  //             Target(state->initPC->parent).print().c_str());
  assert(runningStateToTarget[target].find(state) !=
         runningStateToTarget[target].end());
  runningStateToTarget[target].erase(state);
}

bool ReachabilityTracker::emptyRunningStateToTarget(
    const Target &target) const {
  return runningStateToTarget.at(target).empty();
}

void ReachabilityTracker::addWaitingStateToTarget(const ExecutionState *state,
                                                  const Target &target) {
  std::string s;
  llvm::raw_string_ostream ss(s);
  target.block->basicBlock->printAsOperand(ss, false);
  //klee_message("addWaitingStateToTarget: state %s\n", ss.str().c_str());
  //klee_message("Start location: %s",
  //             Target(state->initPC->parent).print().c_str());
  assert(waitingStateToTarget[target].find(state) ==
         waitingStateToTarget[target].end());
  waitingStateToTarget[target].insert(state);
}

void ReachabilityTracker::removeWaitingStateToPob(const ExecutionState *state,
                                                  ProofObligation *pob) {
  //klee_message("removeWaitingStateToPob: pob %s\n", pob->print().c_str());
  //klee_message("Start location: %s",
  //             Target(state->initPC->parent).print().c_str());
  assert(waitingStateToPob[pob].find(state) == waitingStateToPob[pob].end());
  waitingStateToPob[pob].insert(state);
}

void ReachabilityTracker::addUnreachableRootTarget(const Target &target) {
  unreachableRootTarget.insert(target);
}

bool ReachabilityTracker::checkIsRootTargetUnreachable(
    const Target &target) const {
  return unreachableRootTarget.find(target) != unreachableRootTarget.end();
}

void ReachabilityTracker::updateFinishPob(ProofObligation *pob) {
  Target target = Target(pob->location);
  bool f1 = runningStateToTarget.find(target) != runningStateToTarget.end();
  bool f3 = waitingStateToTarget.find(target) != waitingStateToTarget.end();
  assert(f1 && f3);
  bool fl1 = runningStateToTarget[target].empty();
  bool fl2 = waitingStateToTarget[target] == waitingStateToPob[pob];
  if (fl1 && fl2) {
    blockedPob.insert(pob);
  }
}
} // namespace klee