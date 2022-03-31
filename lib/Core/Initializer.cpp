#include "Initializer.h"
#include "ExecutionState.h"
#include "ProofObligation.h"
#include <iostream>
#include <utility>

namespace klee {

bool SDInitializer::empty() {
  for(auto i : pobs) {
    auto distmap = i->parent->getSortedBackwardDistance(i);
    for(auto j: distmap) {
      if(initializedLocs.count(j.first)) continue;
      return false;
    }
    auto fdistmap = i->parent->parent->getSortedBackwardDistance(i->parent);
    for(auto j : fdistmap) {
      if(initializedLocs.count(j.first->entryKBlock)) continue;
      return false;
    }
  }
  return true;
}

std::pair<KBlock*, std::unordered_set<KBlock*>> SDInitializer::selectAction() {
  for(auto i : pobs) {
    auto distmap = i->parent->getSortedBackwardDistance(i);
    for(auto j: distmap) {
      if(initializedLocs.count(j.first)) continue;
      initializedLocs.insert(j.first);
      return std::make_pair(j.first, pobs);
    }
    auto fdistmap = i->parent->parent->getSortedBackwardDistance(i->parent);
    for(auto j : fdistmap) {
      if(initializedLocs.count(j.first->entryKBlock)) continue;
      initializedLocs.insert(j.first->entryKBlock);
      return std::make_pair(j.first->entryKBlock, pobs);
    }
  }
  assert(0);
}

void SDInitializer::addPob(ProofObligation* pob) {
  pobs.insert(pob->location);
}

void SDInitializer::removePob(ProofObligation* pob) {
  pobs.erase(pob->location);
}

void SDInitializer::addValidityCoreInit(std::pair<ExecutionState *, KBlock *> v) {}

bool ForkInitializer::empty() {
  for(auto i : pobs) {
    auto distmap = i->parent->getSortedBackwardDistance(i);
    for(auto j: distmap) {
      if(j.second == 0) continue;
      if(!j.first->basicBlock->hasNPredecessorsOrMore(2)) continue;
      if(initializedLocs.count(j.first)) continue;
      return false;
    }
    auto fdistmap = i->parent->parent->getSortedBackwardDistance(i->parent);
    for(auto j : fdistmap) {
      if(initializedLocs.count(j.first->entryKBlock)) continue;
      return false;
    }
  }
  return true;
}

std::pair<KBlock*, std::unordered_set<KBlock*>> ForkInitializer::selectAction() {
  for(auto i : pobs) {
    auto distmap = i->parent->getSortedBackwardDistance(i);
    for(auto j: distmap) {
      if(j.second == 0) continue;
      if(!j.first->basicBlock->hasNPredecessorsOrMore(2)) continue;
      if(initializedLocs.count(j.first)) continue;
      initializedLocs.insert(j.first);
      return std::make_pair(j.first, pobs);
    }
    auto fdistmap = i->parent->parent->getSortedBackwardDistance(i->parent);
    for(auto j : fdistmap) {
      if(initializedLocs.count(j.first->entryKBlock)) continue;
      initializedLocs.insert(j.first->entryKBlock);
      return std::make_pair(j.first->entryKBlock, pobs);
    }
  }
  assert(0);
}

void ForkInitializer::addPob(ProofObligation *pob) {
  pobs.insert(pob->location);
}

void ForkInitializer::removePob(ProofObligation *pob) {
  pobs.erase(pob->location);
}

void ForkInitializer::addValidityCoreInit(std::pair<ExecutionState *, KBlock *> v) {}

std::pair<KBlock *, std::unordered_set<KBlock *>>
ValidityCoreInitializer::selectAction() {
  std::pair<KBlock *, KBlock *> v = validityCoreInits.front();
  validityCoreInits.pop();
  return std::make_pair(v.first, std::unordered_set({ v.second }));
}

bool ValidityCoreInitializer::empty() {
  return validityCoreInits.empty();
}

void ValidityCoreInitializer::addPob(ProofObligation *pob) {}
  
void ValidityCoreInitializer::removePob(ProofObligation *pob) {}

void ValidityCoreInitializer::addValidityCoreInit(std::pair<ExecutionState *, KBlock *> v) {
  auto state = v.first;
  SolverQueryMetaData metaData = state->queryMetaData;
  assert(!metaData.queryValidityCores.empty());
  SolverQueryMetaData::core_ty core = metaData.queryValidityCores.back().second;
  assert(!core.empty());
  std::vector<std::pair<KBlock *, KBlock *>> initTargets;

  KFunction *kf = state->initPC->parent->parent;
  KBlock *init = state->initPC->parent;
  KBlock *front = core.front().second->parent;
  front = front->parent->getNearestJoinBlock(front);
  KBlock *back = core.back().second->parent;
  back = back->parent->getNearestJoinBlock(back);
  KBlock *target = v.second;

  initTargets.push_back({ back, target });

  auto distance = kf->getDistance(front);
  std::unordered_set<llvm::BasicBlock *> joinBlocks;
  std::vector<KBlock *> stack;
  stack.push_back(back);
  while (!stack.empty()) {
    auto block = stack.back();
    stack.pop_back();
    for (auto const &pred : predecessors(block->basicBlock)) {
      auto predJoin = kf->getNearestJoinBlock(kf->blockMap[pred]);
      if (distance.count(predJoin) && !joinBlocks.count(predJoin->basicBlock)) {
        joinBlocks.insert(predJoin->basicBlock);
        stack.insert(stack.begin(), predJoin);
      }
      initTargets.insert(initTargets.begin(), { predJoin, block });
    }
  }

  initTargets.insert(initTargets.begin(), { init, front });

  while (!initTargets.empty()) {
    auto initTarget = initTargets.back();
    initTargets.pop_back();
    if (!initializedLocs.count(initTarget.first->basicBlock) ||
        !initializedLocs[initTarget.first->basicBlock].count(initTarget.second->basicBlock)) {
      initializedLocs[initTarget.first->basicBlock].insert(initTarget.second->basicBlock);
      validityCoreInits.push(std::make_pair(initTarget.first, initTarget.second));
    }
  }
}

};
