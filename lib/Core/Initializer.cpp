#include "Initializer.h"
#include "Path.h"
#include "SearcherUtil.h"
#include "klee/Support/ModuleUtil.h"
#include "klee/Support/OptionCategories.h"

#include "llvm/IR/Instructions.h"
#include "klee/Support/ErrorHandling.h"

#include <algorithm>
#include <iostream>
#include <set>
#include <stack>
#include <utility>

using namespace llvm;

namespace {
llvm::cl::opt<bool> DebugInitializer(
    "debug-initializer",
    llvm::cl::desc(""),
    llvm::cl::init(false),
    llvm::cl::cat(klee::DebugCat));
}

namespace klee {

std::pair<KInstruction *, std::set<Target>>
ConflictCoreInitializer::selectAction() {
  auto action = conflictCoreInits.front();
  conflictCoreInits.pop();
  return action;
}

bool ConflictCoreInitializer::empty() {
  return conflictCoreInits.empty();
}

void ConflictCoreInitializer::addPob(ProofObligation *pob) {
  Target target(pob->location);
  if (mapTargetToInitLocations.count(target)) {
    for (auto &init : mapTargetToInitLocations[target]) {
      if (!mapInitLocationToTargets.count(init))
        continue;

      conflictCoreInits.push(std::make_pair(init, mapInitLocationToTargets[init]));
      for (auto &sideTarget : mapInitLocationToTargets[init]) {
        if (sideTarget == target)
          continue;

        auto initPos =  std::find(
          mapTargetToInitLocations[sideTarget].begin(),
          mapTargetToInitLocations[sideTarget].end(),
          init);
        mapTargetToInitLocations[sideTarget].erase(initPos);
      }
      mapInitLocationToTargets.erase(init);
    }
    mapTargetToInitLocations.erase(target);
  }
}

void ConflictCoreInitializer::removePob(ProofObligation *pob) {}

void ConflictCoreInitializer::addConflictInit(const Conflict &conflict, KBlock *target) {
  const Conflict::core_ty &core = conflict.core;
  assert(!core.empty());
  const Path &path = conflict.path;
  std::set<std::pair<KInstruction *, Target>> inits;
  KFunction *mainKF = initInst->parent->parent;
  KInstruction *current = initInst;
  std::set<KFunction *> visited;
  for (size_t pathIndex = 1; pathIndex < path.size(); pathIndex++) {
    if (path.getBlock(pathIndex)->parent != path.getBlock(pathIndex - 1)->parent) {
      if (path.getBlock(pathIndex - 1)->parent == mainKF) {
        auto dismantled = dismantle(current->parent, {path.getBlock(pathIndex - 1)});
        for (auto &blockpair : dismantled) {
          inits.insert(std::make_pair(blockpair.first == current->parent ? current : blockpair.first->instructions[0],
                                      Target(blockpair.second)));
          if (blockpair.second->getKBlockType() == KBlockType::Call) {
            KFunction* f = dyn_cast<KCallBlock>(blockpair.second)->getKFunction();
            if (f) {
              inits.insert(std::make_pair(blockpair.second->instructions[0],
                                          Target(f->entryKBlock)));
            }
          }
        }
        inits.insert(std::make_pair(path.getBlock(pathIndex - 1)->instructions[0],
                                    Target(path.getBlock(pathIndex))));
        current =
          isa<llvm::CallInst>(path.getBlock(pathIndex - 1)->instructions[0]->inst) ?
            path.getBlock(pathIndex - 1)->instructions[1] :
            path.getBlock(pathIndex)->instructions[0];
      }
      if (path.getBlock(pathIndex)->parent != mainKF &&
          !dismantledKFunctions.count(path.getBlock(pathIndex)->parent)) {
        visited.insert(path.getBlock(pathIndex)->parent);
      }
    }
    if (pathIndex == path.size() - 1) {
      if (path.getBlock(pathIndex)->parent == mainKF &&
          path.getBlock(pathIndex) != current->parent) {
        auto dismantled = dismantle(current->parent, {path.getBlock(pathIndex)});
        for (auto &blockpair : dismantled) {
          inits.insert(std::make_pair(
            blockpair.first == current->parent ? current : blockpair.first->instructions[0],
            Target(blockpair.second)));
          if (blockpair.second->getKBlockType() == KBlockType::Call) {
            KFunction *f = dyn_cast<KCallBlock>(blockpair.second)->getKFunction();
            if (f) {
              inits.insert(std::make_pair(blockpair.second->instructions[0],
                                          Target(f->entryKBlock)));
            }
          }
        }
      }
    }
  }
  for (auto kf : visited) {
    dismantledKFunctions.insert(kf);
    std::vector<KBlock *> to = kf->returnKBlocks;

    if (kf == path.getFinalBlock()->parent)
      to.push_back(path.getFinalBlock());

    auto dismantled = dismantle(kf->entryKBlock, to);
    for (auto blockpair : dismantled) {

      inits.insert(std::make_pair(blockpair.first->instructions[0],
                                  Target(blockpair.second)));

      if (blockpair.second->getKBlockType() == KBlockType::Call) {
        KFunction *f = dyn_cast<KCallBlock>(blockpair.second)->getKFunction();
        if (f) {
          inits.insert(std::make_pair(blockpair.second->instructions[0],
                                      Target(f->entryKBlock)));
        }
      }
    }
  }
  inits.insert(std::make_pair(path.getFinalBlock()->instructions[0], Target(target)));

  std::map<KInstruction *, std::set<Target>> ret;
  if (DebugInitializer) {
    llvm::errs() << "Initialization of entry points and targets: \n";
  }
  for (auto &init : inits) {
    if (!initialized[init.first].count(init.second) &&
        !(isa<KReturnBlock>(init.second.block) && init.first->parent == init.second.block)) {
      if (isa<CallInst>(init.first->inst) &&
          cast<KCallBlock>(init.first->parent)->calledFunction !=
              init.second.block->parent->function) {
        continue;
      }
      if (DebugInitializer) {
        llvm::errs() << init.first->getIRLocation() << "\n" << init.first->getSourceLocation() << "\n";
        llvm::errs() << init.second.print() << "\n" << init.second.block->instructions[0]->getSourceLocation();
        llvm::errs() << "\n";
      }
      ret[init.first].insert(init.second);
      initialized[init.first].insert(init.second);
    }
  }
  if (DebugInitializer) {
    llvm::errs() << "\n";
  }
  for (auto &initTargets : ret) {
    mapInitLocationToTargets[initTargets.first] = initTargets.second;
    for (auto &target : initTargets.second) {
      mapTargetToInitLocations[target].push_back(initTargets.first);
    }
  }
}

void ConflictCoreInitializer::setEntryPoint(KFunction *_entrypoint) {
  entrypoint = _entrypoint;
}

bool ConflictCoreInitializer::isDominatorSet(
    ProofObligation *pob, const std::set<KBlock *> &dominatorSet) const {
  bool result = !entrypoint->isReachable(pob->location, dominatorSet);
  klee_message("Result: %d\n", result);
  return result;
}

bool ConflictCoreInitializer::isTargetUnreachable(ProofObligation *pob) const {
  if (pobBlockSet.find(pob) == pobBlockSet.end()) {
    return false;
  }
  return isDominatorSet(pob, pobBlockSet.at(pob));
}

void ConflictCoreInitializer::createRunningStateToTarget(
    const ExecutionState *state, const Target &target) {
  std::string s;
  llvm::raw_string_ostream ss(s);
  target.block->basicBlock->printAsOperand(ss, false);
  klee_message("createRunningStateToTarget: state %s\n", ss.str().c_str());
  assert(runningStateToTarget[target].find(state->initPC->parent) ==
         runningStateToTarget[target].end());
  runningStateToTarget[target].insert({state->initPC->parent, {state}});
  waitingStateToTarget[target].insert({state->initPC->parent, {}});
}

void ConflictCoreInitializer::addRunningStateToTarget(
    const ExecutionState *state, const Target &target) {
  std::string s;
  llvm::raw_string_ostream ss(s);
  target.block->basicBlock->printAsOperand(ss, false);
  klee_message("addRunningStateToTarget: state %s\n", ss.str().c_str());
  assert(runningStateToTarget.find(target) != runningStateToTarget.end() &&
         runningStateToTarget[target].find(state->initPC->parent) !=
             runningStateToTarget[target].end());
  runningStateToTarget[target][state->initPC->parent].insert(state);
}

void ConflictCoreInitializer::removeRunningStateToTarget(
    const ExecutionState *state, const Target &target) {
  std::string s;
  llvm::raw_string_ostream ss(s);
  target.block->basicBlock->printAsOperand(ss, false);
  klee_message("removeRunningStateToTarget: state %s\n", ss.str().c_str());
  assert(runningStateToTarget[target][state->initPC->parent].find(state) !=
         runningStateToTarget[target][state->initPC->parent].end());
  runningStateToTarget[target][state->initPC->parent].erase(state);
}

bool ConflictCoreInitializer::emptyRunningStateToTarget(
    KBlock *stateStartBlock, const Target &target) const {
  return runningStateToTarget.at(target).at(stateStartBlock).empty();
}

void ConflictCoreInitializer::addWaitingStateToTarget(
    const ExecutionState *state, const Target &target) {
  std::string s;
  llvm::raw_string_ostream ss(s);
  target.block->basicBlock->printAsOperand(ss, false);
  klee_message("addWaitingStateToTarget: state %s\n", ss.str().c_str());
  waitingStateToTarget[target][state->initPC->parent].insert(state);
}

void ConflictCoreInitializer::removeWaitingStateToPob(
    const ExecutionState *state, ProofObligation *pob) {
  klee_message("removeWaitingStateToPob: pob %s\n", pob->print().c_str());
  waitingStateToPob[pob][state->initPC->parent].insert(state);
}

void ConflictCoreInitializer::addUnreachableRootTarget(const Target &target) {
  unreachableRootTarget.insert(target);
}

bool ConflictCoreInitializer::checkIsRootTargetUnreachable(const Target &target) const {
  return unreachableRootTarget.find(target) != unreachableRootTarget.end();
}

void ConflictCoreInitializer::updateBlockSetForPob(ProofObligation *pob) {
  Target target = Target(pob->location);
  bool f1 = runningStateToTarget.find(target) != runningStateToTarget.end();
  bool f3 = waitingStateToTarget.find(target) != waitingStateToTarget.end();
  assert(f1 && f3);
  FromStartToStates waitingStatesToPob = waitingStateToTarget[target];
  for (const auto &startAndStates : waitingStatesToPob) {
    KBlock *startBlock = startAndStates.first;
    std::set<const ExecutionState *> waitingStates = startAndStates.second;
    bool f2 = runningStateToTarget[target].find(startBlock) !=
              runningStateToTarget[target].end();
    bool f4 = waitingStateToTarget[target].find(startBlock) !=
              waitingStateToTarget[target].end();
    assert(f2 && f4);

    bool fl1 = runningStateToTarget[target][startBlock].empty();
    bool fl2 = waitingStateToTarget[target][startBlock] ==
               waitingStateToPob[pob][startBlock];
    if (fl1 && fl2) {
      pobBlockSet[pob].insert(startBlock);
    }
  }
}

}
 
