#include "Initializer.h"
#include "ExecutionState.h"
#include "ProofObligation.h"
#include "klee/Module/KInstruction.h"

#include <utility>

namespace klee {

std::pair<KInstruction *, std::set<Target>>
ValidityCoreInitializer::selectAction() {
  auto v = validityCoreInits.front();
  validityCoreInits.pop();
  return std::make_pair(v.first, std::set({ v.second }));
}

bool ValidityCoreInitializer::empty() {
  return validityCoreInits.empty();
}

void ValidityCoreInitializer::addPob(ProofObligation *pob) {}
  
void ValidityCoreInitializer::removePob(ProofObligation *pob) {}

void ValidityCoreInitializer::addValidityCoreInit(std::pair<ExecutionState *, KBlock *> v) {
  // auto state = v.first;
  // SolverQueryMetaData metaData = state->queryMetaData;
  // assert(!metaData.queryValidityCores.empty());
  // SolverQueryMetaData::core_ty core = metaData.queryValidityCores.back().second;
  // assert(!core.empty());
  // std::vector<std::pair<KBlock *, KBlock *>> initTargets;

  // KFunction *kf = state->initPC->parent->parent;
  // KBlock *init = state->initPC->parent;
  // KBlock *front = core.front().second->parent;
  // front = front->parent->getNearestJoinBlock(front);
  // KBlock *back = core.back().second->parent;
  // back = back->parent->getNearestJoinBlock(back);
  // KBlock *target = v.second;

  // initTargets.push_back({ back, target });

  // auto distance = kf->getDistance(front);
  // std::unordered_set<llvm::BasicBlock *> joinBlocks;
  // std::vector<KBlock *> stack;
  // stack.push_back(back);
  // while (!stack.empty()) {
  //   auto block = stack.back();
  //   stack.pop_back();
  //   for (auto const &pred : predecessors(block->basicBlock)) {
  //     auto predJoin = kf->getNearestJoinBlock(kf->blockMap[pred]);
  //     if (distance.count(predJoin) && !joinBlocks.count(predJoin->basicBlock)) {
  //       joinBlocks.insert(predJoin->basicBlock);
  //       stack.insert(stack.begin(), predJoin);
  //     }
  //     initTargets.insert(initTargets.begin(), { predJoin, block });
  //   }
  // }

  // initTargets.insert(initTargets.begin(), { init, front });

  // while (!initTargets.empty()) {
  //   auto initTarget = initTargets.back();
  //   initTargets.pop_back();
  //   if (!initializedLocs.count(initTarget.first->basicBlock) ||
  //       !initializedLocs[initTarget.first->basicBlock].count(initTarget.second->basicBlock)) {
  //     initializedLocs[initTarget.first->basicBlock].insert(initTarget.second->basicBlock);
  //     validityCoreInits.push(std::make_pair(initTarget.first, initTarget.second));
  //   }
  // }
}

};
