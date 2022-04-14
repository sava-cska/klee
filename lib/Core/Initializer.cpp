#include "Initializer.h"
#include "ExecutionState.h"
#include "Path.h"
#include "ProofObligation.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Solver/Solver.h"
#include "klee/Support/ModuleUtil.h"

#include <iostream>
#include <set>
#include <stack>
#include <utility>

namespace klee {

std::pair<KInstruction *, std::set<Target>>
ValidityCoreInitializer::selectAction() {
  auto v = validityCoreInits.front();
  validityCoreInits.pop();
  return std::make_pair(v.first, v.second);
}

bool ValidityCoreInitializer::empty() {
  return validityCoreInits.empty();
}

void ValidityCoreInitializer::addPob(ProofObligation *pob) {}
  
void ValidityCoreInitializer::removePob(ProofObligation *pob) {}

void ValidityCoreInitializer::addValidityCoreInit(std::pair<Path, SolverQueryMetaData::core_ty> v, KBlock* b) {
  SolverQueryMetaData::core_ty core = v.second;
  assert(!core.empty());
  Path path = v.first;
  std::set<std::pair<KInstruction*, Target>> inits;
  std::stack<KBlock*> stack;
  size_t path_index = 0;
  size_t front = *(core.front().second);

  inits.insert(std::make_pair(initInst, Target(path.getBlock(front), false)));

  while(path_index != front) {
    path_index++;
    if(path.getBlock(path_index)->parent != path.getBlock(path_index-1)->parent) {
      if(path.getBlock(path_index-1)->getKBlockType() == KBlockType::Call) {
        stack.push(path.getBlock(path_index-1));
      } else {
        stack.pop();
      }
    }
  }

  KBlock* current = path.getBlock(front);
  bool after_return = false;
  while(true) {
    while (path_index != path.size() - 1 &&
           path.getBlock(path_index)->parent == current->parent) {
      path_index++;
    }
    if (path.getBlock(path_index - 1)->getKBlockType() == KBlockType::Call) {
      auto dismantled = dismantle(current, {path.getBlock(path_index - 1)});
      for (auto blockpair : dismantled) {
        if (blockpair.first == current && after_return) {
          assert(current->getKBlockType() == KBlockType::Call);
          inits.insert(std::make_pair(current->instructions[1],
                                      Target(blockpair.second, false)));
        } else {
          inits.insert(std::make_pair(current->instructions[0],
                                      Target(blockpair.second, false)));
        }
      }
      stack.push(path.getBlock(path_index - 1));
      current = path.getBlock(path_index);
      after_return = false;
      if(path_index == path.size() - 1) {
        break;
      }
    } else {
      if(path.getBlock(path_index)->parent != current->parent) {
        auto dismantled = dismantle(current, current->parent->finalKBlocks);
        for (auto blockpair : dismantled) {
          if (blockpair.first == current && after_return) {
            assert(current->getKBlockType() == KBlockType::Call);
            inits.insert(std::make_pair(current->instructions[1],
                                        Target(blockpair.second, true)));
          } else {
            inits.insert(std::make_pair(current->instructions[0],
                                        Target(blockpair.second, true)));
          }
        }
        current = stack.top();
        stack.pop();
        after_return = true;
      }
      if(path_index == path.size() - 1) {
        assert(path.getBlock(path_index)->parent == current->parent);
        auto dismantled = dismantle(current, {path.getBlock(path_index)});
        for (auto blockpair : dismantled) {
          if (blockpair.first == current && after_return) {
            assert(current->getKBlockType() == KBlockType::Call);
            inits.insert(std::make_pair(current->instructions[1],
                                        Target(blockpair.second, false)));
          } else {
            inits.insert(std::make_pair(current->instructions[0],
                                        Target(blockpair.second, false)));
          }
        }
        break;
      }
    }
  }
  
  inits.insert(std::make_pair(path.getFinalBlock()->instructions[0], Target(b, false)));
  std::map<KInstruction*, std::set<Target>> ret;
  
  for (auto init : inits) {
    if(!initialized[init.first].count(init.second)) {
      ret[init.first].insert(init.second);
      initialized[init.first].insert(init.second);
    }
  }
  for(auto i : ret) {
    validityCoreInits.push(i);
  }
}
  
};
