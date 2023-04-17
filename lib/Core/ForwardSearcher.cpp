//===-- Searcher.cpp ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "ForwardSearcher.h"

#include "CoreStats.h"
#include "ExecutionState.h"
#include "Executor.h"
#include "MergeHandler.h"
#include "PTree.h"
#include "StatsTracker.h"

#include "klee/ADT/DiscretePDF.h"
#include "klee/ADT/RNG.h"
#include "klee/Statistics/Statistics.h"
#include "klee/Module/InstructionInfoTable.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Support/ErrorHandling.h"
#include "klee/System/Time.h"

#include "llvm/IR/Constants.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/CommandLine.h"

#include <cassert>
#include <cmath>
#include <map>
#include <unordered_set>

using namespace klee;
using namespace llvm;


///

ExecutionState &DFSSearcher::selectState() {
  return *states.back();
}

void DFSSearcher::update(ExecutionState *current,
                         const std::vector<ExecutionState *> &addedStates,
                         const std::vector<ExecutionState *> &removedStates) {
  // insert states
  states.insert(states.end(), addedStates.begin(), addedStates.end());

  // remove states
  for (const auto state : removedStates) {
    if (state == states.back()) {
      states.pop_back();
    } else {
      auto it = std::find(states.begin(), states.end(), state);
      assert(it != states.end() && "invalid state removed");
      states.erase(it);
    }
  }
}

bool DFSSearcher::empty() {
  return states.empty();
}

void DFSSearcher::printName(llvm::raw_ostream &os) {
  os << "DFSSearcher\n";
}


///

ExecutionState &BFSSearcher::selectState() {
  return *states.front();
}

void BFSSearcher::update(ExecutionState *current,
                         const std::vector<ExecutionState *> &addedStates,
                         const std::vector<ExecutionState *> &removedStates) {
  // update current state
  // Assumption: If new states were added KLEE forked, therefore states evolved.
  // constraints were added to the current state, it evolved.
  if (current && (!addedStates.empty() || current->getPrevPCBlock() != current->getPCBlock()) &&
      std::find(removedStates.begin(), removedStates.end(), current) == removedStates.end()) {
    auto pos = std::find(states.begin(), states.end(), current);
    assert(pos != states.end());
    states.erase(pos);
    states.push_back(current);
  }

  // insert states
  states.insert(states.end(), addedStates.begin(), addedStates.end());

  // remove states
  for (const auto state : removedStates) {
    if (state == states.front()) {
      states.pop_front();
    } else {
      auto it = std::find(states.begin(), states.end(), state);
      assert(it != states.end() && "invalid state removed");
      states.erase(it);
    }
  }
}

bool BFSSearcher::empty() {
  return states.empty();
}

void BFSSearcher::printName(llvm::raw_ostream &os) {
  os << "BFSSearcher\n";
}


///

RandomSearcher::RandomSearcher(RNG &rng) : theRNG{rng} {}

ExecutionState &RandomSearcher::selectState() {
  return *states[theRNG.getInt32() % states.size()];
}

void RandomSearcher::update(ExecutionState *current,
                            const std::vector<ExecutionState *> &addedStates,
                            const std::vector<ExecutionState *> &removedStates) {
  // insert states
  states.insert(states.end(), addedStates.begin(), addedStates.end());

  // remove states
  for (const auto state : removedStates) {
    auto it = std::find(states.begin(), states.end(), state);
    assert(it != states.end() && "invalid state removed");
    states.erase(it);
  }
}

bool RandomSearcher::empty() {
  return states.empty();
}

void RandomSearcher::printName(llvm::raw_ostream &os) {
  os << "RandomSearcher\n";
}

///

TargetedSearcher::TargetedSearcher(Target _target)
    : states(std::make_unique<
             DiscretePDF<ExecutionState *, ExecutionStateIDCompare>>()),
      target(_target), distanceToTargetFunction(
                           target.block->parent->parent->getBackwardDistance(
                               target.block->parent)) {}

TargetedSearcher::~TargetedSearcher() {
  for (auto state : states_set) {
    state->removeTarget(target);
  }
}

ExecutionState &TargetedSearcher::selectState() { return *states->choose(0.0); }

bool TargetedSearcher::distanceInCallGraph(KFunction *kf, KBlock *kb,
                                           unsigned int &distance) {
  distance = UINT_MAX;
  std::map<KBlock *, unsigned> &dist = kf->getDistance(kb);

  if (kf == target.block->parent && dist.find(target.block) != dist.end()) {
    distance = 0;
    return true;
  }

  for (auto &kCallBlock : kf->kCallBlocks) {
    if (dist.find(kCallBlock) != dist.end()) {
      KFunction *calledKFunction =
          kf->parent->functionMap[kCallBlock->calledFunction];
      if (distanceToTargetFunction.find(calledKFunction) !=
              distanceToTargetFunction.end() &&
          distance > distanceToTargetFunction[calledKFunction] + 1) {
        distance = distanceToTargetFunction[calledKFunction] + 1;
      }
    }
  }
  return distance != UINT_MAX;
}

TargetedSearcher::WeightResult
TargetedSearcher::tryGetLocalWeight(const ExecutionState &es, double &weight,
                                    const std::vector<KBlock *> &localTargets) {
  unsigned int intWeight = es.steppedMemoryInstructions;
  KFunction *currentKF = es.stack.back().kf;
  KBlock *currentKB = currentKF->blockMap[es.getPCBlock()];
  KBlock *prevKB = currentKF->blockMap[es.getPrevPCBlock()];
  std::map<KBlock *, unsigned> &dist = currentKF->getDistance(currentKB);
  unsigned int localWeight = UINT_MAX;
  for (auto &end : localTargets) {
    if (dist.count(end) > 0) {
      unsigned int w = dist[end];
      localWeight = std::min(w, localWeight);
    }
  }

  if (localWeight == UINT_MAX)
    return Miss;
  if (localWeight == 0 && prevKB != currentKB)
    return Done;

  intWeight += localWeight;
  weight = intWeight * (1.0 / 4294967296.0); // number on [0,1)-real-interval
  return Continue;
}

TargetedSearcher::WeightResult
TargetedSearcher::tryGetPreTargetWeight(const ExecutionState &es,
                                        double &weight) {
  KFunction *currentKF = es.stack.back().kf;
  std::vector<KBlock *> localTargets;
  for (auto &kCallBlock : currentKF->kCallBlocks) {
    KFunction *calledKFunction =
        currentKF->parent->functionMap[kCallBlock->calledFunction];
    if (distanceToTargetFunction.count(calledKFunction) > 0) {
      localTargets.push_back(kCallBlock);
    }
  }

  if (localTargets.empty())
    return Miss;

  WeightResult res = tryGetLocalWeight(es, weight, localTargets);
  weight = 1.0 / 2.0 + weight * (1.0 / 2.0); // number on [0.5,1)-real-interval
  return res == Done ? Continue : res;
}

TargetedSearcher::WeightResult
TargetedSearcher::tryGetPostTargetWeight(const ExecutionState &es,
                                         double &weight) {
  KFunction *currentKF = es.stack.back().kf;
  std::vector<KBlock *> &localTargets = currentKF->finalKBlocks;

  if (localTargets.empty())
    return Miss;

  WeightResult res = tryGetLocalWeight(es, weight, localTargets);
  weight = 1.0 / 2.0 + weight * (1.0 / 2.0); // number on [0.5,1)-real-interval
  return res == Done ? Continue : res;
}

TargetedSearcher::WeightResult
TargetedSearcher::tryGetTargetWeight(const ExecutionState &es, double &weight) {
  std::vector<KBlock *> localTargets = {target.block};
  WeightResult res = tryGetLocalWeight(es, weight, localTargets);

  weight = weight * (1.0 / 2.0); // number on [0,0.5)-real-interval
  return res;
}

TargetedSearcher::WeightResult
TargetedSearcher::tryGetWeight(const ExecutionState &es, double &weight) {
  if (target.atReturn()) {
    if (es.prevPC->parent == target.block &&
        es.prevPC == target.block->getLastInstruction()) {
      return Done;
    } else if (es.pc->parent == target.block) {
      weight = 0;
      return Continue;
    }
  }

  BasicBlock *bb = es.getPCBlock();
  KBlock *kb = es.stack.back().kf->blockMap[bb];
  unsigned int minCallWeight = UINT_MAX, minSfNum = UINT_MAX, sfNum = 0;
  for (auto sfi = es.stack.rbegin(), sfe = es.stack.rend(); sfi != sfe; sfi++) {
    unsigned callWeight;
    if (distanceInCallGraph(sfi->kf, kb, callWeight)) {
      callWeight *= 2;
      callWeight += sfNum;
      if (callWeight < minCallWeight) {
        minCallWeight = callWeight;
        minSfNum = sfNum;
      }
    }

    if (sfi->caller) {
      kb = sfi->caller->parent;
      bb = kb->basicBlock;
    }
    sfNum++;

    if (minCallWeight < sfNum)
      break;
  }

  WeightResult res = Miss;
  if (minCallWeight == 0)
    res = tryGetTargetWeight(es, weight);
  else if (minSfNum == 0)
    res = tryGetPreTargetWeight(es, weight);
  else if (minSfNum != UINT_MAX)
    res = tryGetPostTargetWeight(es, weight);
  return res;
}

void TargetedSearcher::update(
    ExecutionState *current, const std::vector<ExecutionState *> &addedStates,
    const std::vector<ExecutionState *> &removedStates) {
  reachedOnLastUpdate.clear();
  unreachedOnLastUpdate.clear();

  double weight = 0;
  // update current
  // регать unreachedState
  if (current && std::find(removedStates.begin(), removedStates.end(),
                           current) == removedStates.end()) {
    switch (tryGetWeight(*current, weight)) {
    case Continue:
      if (states->inTree(current))
        states->update(current, weight);
      else {
        states->insert(current, weight);
        states_set.insert(current);
      }
      break;
    case Done:
      if (!states->inTree(current)) {
        states->insert(current, weight);
        states_set.insert(current);
      }
      reachedOnLastUpdate.insert(current);
      break;
    case Miss:
      //current->removeTarget(target);
      //states->remove(current);
      //states_set.erase(current);
      unreachedOnLastUpdate.insert(current);
      break;
    }
  }

  // insert states
  for (const auto state : addedStates) {
    switch (tryGetWeight(*state, weight)) {
    case Continue:
      states->insert(state, weight);
      states_set.insert(state);
      break;
    case Done:
      states->insert(state, weight);
      states_set.insert(state);
      reachedOnLastUpdate.insert(state);
      break;
    case Miss:
      //state->removeTarget(target);
      states->insert(state, weight);
      states_set.insert(state);
      unreachedOnLastUpdate.insert(state);
      break;
    }
  }

  // remove states
  for (const auto state : removedStates) {
    if (target.atReturn() &&
        state->prevPC == target.block->getLastInstruction())
      reachedOnLastUpdate.insert(state);
    else {
      switch (tryGetWeight(*state, weight)) {
      case Continue:
        unreachedOnLastUpdate.insert(state);
        break;
      case Done:
        reachedOnLastUpdate.insert(state);
        break;
      case Miss:
        unreachedOnLastUpdate.insert(state);
        break;
      }
    }
  }
}

bool TargetedSearcher::empty() { return states->empty(); }

void TargetedSearcher::printName(llvm::raw_ostream &os) {
  os << "TargetedSearcher";
}

std::set<ExecutionState *> TargetedSearcher::reached() const {
  return reachedOnLastUpdate;
}

std::set<ExecutionState *> TargetedSearcher::unreached() const {
  return unreachedOnLastUpdate;
}

void TargetedSearcher::removeUnreached() {
  for (auto state : unreachedOnLastUpdate) {
    states->remove(state);
    states_set.erase(state);
    state->removeTarget(target);
  }
}

void TargetedSearcher::removeReached() {
  for (auto state : reachedOnLastUpdate) {
    states->remove(state);
    states_set.erase(state);
    state->removeTarget(target);
  }
}


///

GuidedSearcher::GuidedSearcher(std::unique_ptr<ForwardSearcher> _baseSearcher,
                               bool _reachingEnough)
    : baseSearcher(std::move(_baseSearcher)), reachingEnough(_reachingEnough) {}

ExecutionState &GuidedSearcher::selectState() {
  unsigned size = targetedSearchers.size();
  index = (index + 1) % (size + 1);
  if (reachingEnough && index == size)
    return baseSearcher->selectState();
  else {
    index = index % size;
    auto it = targetedSearchers.begin();
    std::advance(it, index);
    Target target = it->first;
    assert(targetedSearchers.find(target) != targetedSearchers.end() &&
           !targetedSearchers[target]->empty());
    return targetedSearchers[target]->selectState();
  }
}

void GuidedSearcher::innerUpdate(
    ExecutionState *current, const std::vector<ExecutionState *> &addedStates,
    const std::vector<ExecutionState *> &removedStates) {

  std::map<Target, std::vector<ExecutionState *>> addedTStates;
  std::map<Target, std::vector<ExecutionState *>> removedTStates;
  std::set<Target> targets;

  for (auto &targetSearcher : targetedSearchers)
    targetSearcher.second->update(nullptr, {}, {});

  for (const auto state : addedStates) {
    for (auto i : state->targets) {
      targets.insert(i);
      addedTStates[i].push_back(state);
    }
  }

  for (const auto state : removedStates) {
    for (auto i : state->targets) {
      targets.insert(i);
      removedTStates[i].push_back(state);
    }
  }

  if (current)
    for (auto i : current->targets)
      targets.insert(i);

  for (auto target : targets) {
    ExecutionState *currTState =
        current && current->targets.find(target) != current->targets.end()
            ? current
            : nullptr;

    if (targetedSearchers.count(target) == 0)
      addTarget(target);
    targetedSearchers[target]->update(currTState, addedTStates[target],
                                      removedTStates[target]);

    if (targetedSearchers[target]->empty())
      targetedSearchers.erase(target);
  }

  baseSearcher->update(current, addedStates, removedStates);
}

void GuidedSearcher::update(
    ExecutionState *current, const std::vector<ExecutionState *> &addedStates,
    const std::vector<ExecutionState *> &removedStates,
    std::map<Target, std::unordered_set<ExecutionState *>> &reachedStates,
    std::map<Target, std::unordered_set<ExecutionState *>> &unreachedStates) {
  innerUpdate(current, addedStates, removedStates);
  collectReachedAndUnreached(reachedStates, unreachedStates);
  clearReached();
}

void GuidedSearcher::update(
    ExecutionState *current, const std::vector<ExecutionState *> &addedStates,
    const std::vector<ExecutionState *> &removedStates) {
  innerUpdate(current, addedStates, removedStates);
  clearReached();
}

void GuidedSearcher::collectReachedAndUnreached(
    std::map<Target, std::unordered_set<ExecutionState *>> &reachedStates,
    std::map<Target, std::unordered_set<ExecutionState *>> &unreachedStates)
    const {
  std::map<Target, std::unordered_set<ExecutionState *>> ret;
  std::vector<Target> targets;
  for (auto const &targetSearcher : targetedSearchers) {
    targets.push_back(targetSearcher.first);
  }
  for (auto target : targets) {
    auto reached = targetedSearchers.at(target)->reached();
    for (const auto &state : reached) {
      reachedStates[target].insert(state);
    }
    auto unreached = targetedSearchers.at(target)->unreached();
    for (const auto &state : unreached) {
      unreachedStates[target].insert(state);
    }
  }
}

void GuidedSearcher::clearReached() {
  std::vector<Target> targets;
  for (auto const &targetSearcher : targetedSearchers)
    targets.push_back(targetSearcher.first);
  for (auto target : targets) {
    auto reached = targetedSearchers[target]->reached();
    auto unreached = targetedSearchers[target]->unreached();
    if (!unreached.empty()) {
      targetedSearchers[target]->removeUnreached();
      if (targetedSearchers[target]->empty())
        targetedSearchers.erase(target);
    }
    if (!reached.empty()) {
      targetedSearchers[target]->removeReached();
      if (reachingEnough || targetedSearchers[target]->empty())
        targetedSearchers.erase(target);
    }
  }
}

bool GuidedSearcher::empty() {
  return reachingEnough ? baseSearcher->empty() : targetedSearchers.empty();
}

void GuidedSearcher::printName(llvm::raw_ostream &os) {
  os << "GuidedSearcher";
}

void GuidedSearcher::addTarget(Target target) {
  targetedSearchers[target] = std::make_unique<TargetedSearcher>(target);
}

void GuidedSearcher::removeTarget(const Target &target) {
  targetedSearchers.erase(target);
}

///

WeightedRandomSearcher::WeightedRandomSearcher(WeightType type, RNG &rng)
  : states(std::make_unique<DiscretePDF<ExecutionState *, ExecutionStateIDCompare>>()),
    theRNG{rng},
    type(type) {

  switch(type) {
  case Depth:
  case RP:
    updateWeights = false;
    break;
  case InstCount:
  case CPInstCount:
  case QueryCost:
  case MinDistToUncovered:
  case CoveringNew:
    updateWeights = true;
    break;
  default:
    assert(0 && "invalid weight type");
  }
}

ExecutionState &WeightedRandomSearcher::selectState() {
  return *states->choose(theRNG.getDoubleL());
}

double WeightedRandomSearcher::getWeight(ExecutionState *es) {
  switch(type) {
    default:
    case Depth:
      return es->depth;
    case RP:
      return std::pow(0.5, es->depth);
    case InstCount: {
      uint64_t count = theStatisticManager->getIndexedValue(stats::instructions,
                                                            es->pc->info->id);
      double inv = 1. / std::max((uint64_t) 1, count);
      return inv * inv;
    }
    case CPInstCount: {
      StackFrame &sf = es->stack.back();
      uint64_t count = sf.callPathNode->statistics.getValue(stats::instructions);
      double inv = 1. / std::max((uint64_t) 1, count);
      return inv;
    }
    case QueryCost:
      return (es->queryMetaData.queryCost.toSeconds() < .1)
                 ? 1.
                 : 1. / es->queryMetaData.queryCost.toSeconds();
    case CoveringNew:
    case MinDistToUncovered: {
      uint64_t md2u = computeMinDistToUncovered(es->pc,
                                                es->stack.back().minDistToUncoveredOnReturn);

      double invMD2U = 1. / (md2u ? md2u : 10000);
      if (type == CoveringNew) {
        double invCovNew = 0.;
        if (es->instsSinceCovNew)
          invCovNew = 1. / std::max(1, (int) es->instsSinceCovNew - 1000);
        return (invCovNew * invCovNew + invMD2U * invMD2U);
      } else {
        return invMD2U * invMD2U;
      }
    }
  }
}

void WeightedRandomSearcher::update(ExecutionState *current,
                                    const std::vector<ExecutionState *> &addedStates,
                                    const std::vector<ExecutionState *> &removedStates) {

  // update current
  if (current && updateWeights &&
      std::find(removedStates.begin(), removedStates.end(), current) == removedStates.end())
    states->update(current, getWeight(current));

  // insert states
  for (const auto state : addedStates)
    states->insert(state, getWeight(state));

  // remove states
  for (const auto state : removedStates)
    states->remove(state);
}

bool WeightedRandomSearcher::empty() {
  return states->empty();
}

void WeightedRandomSearcher::printName(llvm::raw_ostream &os) {
  os << "WeightedRandomSearcher::";
  switch(type) {
    case Depth              : os << "Depth\n"; return;
    case RP                 : os << "RandomPath\n"; return;
    case QueryCost          : os << "QueryCost\n"; return;
    case InstCount          : os << "InstCount\n"; return;
    case CPInstCount        : os << "CPInstCount\n"; return;
    case MinDistToUncovered : os << "MinDistToUncovered\n"; return;
    case CoveringNew        : os << "CoveringNew\n"; return;
    default                 : os << "<unknown type>\n"; return;
  }
}


///

// Check if n is a valid pointer and a node belonging to us
#define IS_OUR_NODE_VALID(n)                                                   \
  (((n).getPointer() != nullptr) && (((n).getInt() & idBitMask) != 0))

RandomPathSearcher::RandomPathSearcher(PForest &processForest, RNG &rng)
  : processForest{processForest},
    theRNG{rng},
    idBitMask{processForest.getNextId()} {};

ExecutionState &RandomPathSearcher::selectState() {
  unsigned flips=0, bits=0, range=0;
  PTreeNodePtr *root = nullptr;
  while (!root || !IS_OUR_NODE_VALID(*root))
    root = &processForest.getPTrees().at(range++ % processForest.getPTrees().size() + 1)->root;
  assert(root->getInt() & idBitMask && "Root should belong to the searcher");
  PTreeNode *n = root->getPointer();
  while (!n->state) {
    if (!IS_OUR_NODE_VALID(n->left)) {
      assert(IS_OUR_NODE_VALID(n->right) && "Both left and right nodes invalid");
      assert(n != n->right.getPointer());
      n = n->right.getPointer();
    } else if (!IS_OUR_NODE_VALID(n->right)) {
      assert(IS_OUR_NODE_VALID(n->left) && "Both right and left nodes invalid");
      assert(n != n->left.getPointer());
      n = n->left.getPointer();
    } else {
      if (bits==0) {
        flips = theRNG.getInt32();
        bits = 32;
      }
      --bits;
      n = ((flips & (1U << bits)) ? n->left : n->right).getPointer();
    }
  }

  return *n->state;
}

void RandomPathSearcher::update(ExecutionState *current,
                                const std::vector<ExecutionState *> &addedStates,
                                const std::vector<ExecutionState *> &removedStates) {
  // insert states
  for (auto &es : addedStates) {
    PTreeNode *pnode = es->ptreeNode, *parent = pnode->parent;
    PTreeNodePtr &root = processForest.getPTrees().at(pnode->getTreeID())->root;
    PTreeNodePtr *childPtr;

    childPtr = parent ? ((parent->left.getPointer() == pnode) ? &parent->left
                                                              : &parent->right)
                      : &root;
    while (pnode && !IS_OUR_NODE_VALID(*childPtr)) {
      childPtr->setInt(childPtr->getInt() | idBitMask);
      pnode = parent;
      if (pnode)
        parent = pnode->parent;

      childPtr = parent
                     ? ((parent->left.getPointer() == pnode) ? &parent->left
                                                             : &parent->right)
                     : &root;
    }
  }

  // remove states
  for (auto &es : removedStates) {
    PTreeNode *pnode = es->ptreeNode, *parent = pnode->parent;
    PTreeNodePtr &root = processForest.getPTrees().at(pnode->getTreeID())->root;

    while (pnode && !IS_OUR_NODE_VALID(pnode->left) &&
           !IS_OUR_NODE_VALID(pnode->right)) {
      auto childPtr =
          parent ? ((parent->left.getPointer() == pnode) ? &parent->left
                                                         : &parent->right)
                 : &root;
      assert(IS_OUR_NODE_VALID(*childPtr) && "Removing pTree child not ours");
      childPtr->setInt(childPtr->getInt() & ~idBitMask);
      pnode = parent;
      if (pnode)
        parent = pnode->parent;
    }
  }
}

bool RandomPathSearcher::empty() {
  bool res = true;
  for (const auto &ntree : processForest.getPTrees())
    res = res && !IS_OUR_NODE_VALID(ntree.second->root);
  return res;
}

void RandomPathSearcher::printName(llvm::raw_ostream &os) {
  os << "RandomPathSearcher\n";
}


///

MergingSearcher::MergingSearcher(ForwardSearcher *baseSearcher)
  : baseSearcher{baseSearcher} {};

void MergingSearcher::pauseState(ExecutionState &state) {
  assert(std::find(pausedStates.begin(), pausedStates.end(), &state) == pausedStates.end());
  pausedStates.push_back(&state);
  baseSearcher->update(nullptr, {}, {&state});
}

void MergingSearcher::continueState(ExecutionState &state) {
  auto it = std::find(pausedStates.begin(), pausedStates.end(), &state);
  assert(it != pausedStates.end());
  pausedStates.erase(it);
  baseSearcher->update(nullptr, {&state}, {});
}

ExecutionState& MergingSearcher::selectState() {
  assert(!baseSearcher->empty() && "base searcher is empty");

  if (!UseIncompleteMerge)
    return baseSearcher->selectState();

  // Iterate through all MergeHandlers
  for (auto cur_mergehandler: mergeGroups) {
    // Find one that has states that could be released
    if (!cur_mergehandler->hasMergedStates()) {
      continue;
    }
    // Find a state that can be prioritized
    ExecutionState *es = cur_mergehandler->getPrioritizeState();
    if (es) {
      return *es;
    } else {
      if (DebugLogIncompleteMerge){
        llvm::errs() << "Preemptively releasing states\n";
      }
      // If no state can be prioritized, they all exceeded the amount of time we
      // are willing to wait for them. Release the states that already arrived at close_merge.
      cur_mergehandler->releaseStates();
    }
  }
  // If we were not able to prioritize a merging state, just return some state
  return baseSearcher->selectState();
}

void MergingSearcher::update(ExecutionState *current,
                             const std::vector<ExecutionState *> &addedStates,
                             const std::vector<ExecutionState *> &removedStates) {
  // We have to check if the current execution state was just deleted, as to
  // not confuse the nurs searchers
  if (std::find(pausedStates.begin(), pausedStates.end(), current) == pausedStates.end()) {
    baseSearcher->update(current, addedStates, removedStates);
  }
}

bool MergingSearcher::empty() {
  return baseSearcher->empty();
}

void MergingSearcher::printName(llvm::raw_ostream &os) {
  os << "MergingSearcher\n";
}


///

BatchingSearcher::BatchingSearcher(ForwardSearcher *baseSearcher, time::Span timeBudget, unsigned instructionBudget)
  : baseSearcher{baseSearcher},
    timeBudget{timeBudget},
    instructionBudget{instructionBudget} {};

ExecutionState &BatchingSearcher::selectState() {
  if (!lastState ||
      (((timeBudget.toSeconds() > 0) &&
        (time::getWallTime() - lastStartTime) > timeBudget)) ||
      ((instructionBudget > 0) &&
       (stats::instructions - lastStartInstructions) > instructionBudget)) {
    if (lastState) {
      time::Span delta = time::getWallTime() - lastStartTime;
      auto t = timeBudget;
      t *= 1.1;
      if (delta > t) {
        klee_message("increased time budget from %f to %f\n", timeBudget.toSeconds(), delta.toSeconds());
        timeBudget = delta;
      }
    }
    lastState = &baseSearcher->selectState();
    lastStartTime = time::getWallTime();
    lastStartInstructions = stats::instructions;
    return *lastState;
  } else {
    return *lastState;
  }
}

void BatchingSearcher::update(ExecutionState *current,
                              const std::vector<ExecutionState *> &addedStates,
                              const std::vector<ExecutionState *> &removedStates) {
  // drop memoized state if it is marked for deletion
  if (std::find(removedStates.begin(), removedStates.end(), lastState) != removedStates.end())
    lastState = nullptr;
  // update underlying searcher
  baseSearcher->update(current, addedStates, removedStates);
}

bool BatchingSearcher::empty() {
  return baseSearcher->empty();
}

void BatchingSearcher::printName(llvm::raw_ostream &os) {
  os << "<BatchingSearcher> timeBudget: " << timeBudget
     << ", instructionBudget: " << instructionBudget
     << ", baseSearcher:\n";
  baseSearcher->printName(os);
  os << "</BatchingSearcher>\n";
}


///

IterativeDeepeningTimeSearcher::IterativeDeepeningTimeSearcher(ForwardSearcher *baseSearcher)
  : baseSearcher{baseSearcher} {};

ExecutionState &IterativeDeepeningTimeSearcher::selectState() {
  ExecutionState &res = baseSearcher->selectState();
  startTime = time::getWallTime();
  return res;
}

void IterativeDeepeningTimeSearcher::update(ExecutionState *current,
                                            const std::vector<ExecutionState *> &addedStates,
                                            const std::vector<ExecutionState *> &removedStates) {

  const auto elapsed = time::getWallTime() - startTime;

  // update underlying searcher (filter paused states unknown to underlying searcher)
  if (!removedStates.empty()) {
    std::vector<ExecutionState *> alt = removedStates;
    for (const auto state : removedStates) {
      auto it = pausedStates.find(state);
      if (it != pausedStates.end()) {
        pausedStates.erase(it);
        alt.erase(std::remove(alt.begin(), alt.end(), state), alt.end());
      }
    }
    baseSearcher->update(current, addedStates, alt);
  } else {
    baseSearcher->update(current, addedStates, removedStates);
  }

  // update current: pause if time exceeded
  if (current &&
      std::find(removedStates.begin(), removedStates.end(), current) == removedStates.end() &&
      elapsed > time) {
    pausedStates.insert(current);
    baseSearcher->update(nullptr, {}, {current});
  }

  // no states left in underlying searcher: fill with paused states
  if (baseSearcher->empty()) {
    time *= 2U;
    klee_message("increased time budget to %f\n", time.toSeconds());
    std::vector<ExecutionState *> ps(pausedStates.begin(), pausedStates.end());
    baseSearcher->update(nullptr, ps, std::vector<ExecutionState *>());
    pausedStates.clear();
  }
}

bool IterativeDeepeningTimeSearcher::empty() {
  return baseSearcher->empty() && pausedStates.empty();
}

void IterativeDeepeningTimeSearcher::printName(llvm::raw_ostream &os) {
  os << "IterativeDeepeningTimeSearcher\n";
}


///

InterleavedSearcher::InterleavedSearcher(const std::vector<ForwardSearcher*> &_searchers) {
  searchers.reserve(_searchers.size());
  for (auto searcher : _searchers)
    searchers.emplace_back(searcher);
}

ExecutionState &InterleavedSearcher::selectState() {
  ForwardSearcher *s = searchers[--index].get();
  if (index == 0) index = searchers.size();
  return s->selectState();
}

void InterleavedSearcher::update(ExecutionState *current,
                                 const std::vector<ExecutionState *> &addedStates,
                                 const std::vector<ExecutionState *> &removedStates) {

  // update underlying searchers
  for (auto &searcher : searchers)
    searcher->update(current, addedStates, removedStates);
}

bool InterleavedSearcher::empty() {
  return searchers[0]->empty();
}

void InterleavedSearcher::printName(llvm::raw_ostream &os) {
  os << "<InterleavedSearcher> containing " << searchers.size() << " searchers:\n";
  for (const auto &searcher : searchers)
    searcher->printName(os);
  os << "</InterleavedSearcher>\n";
}
