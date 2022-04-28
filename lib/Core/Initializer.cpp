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
  std::vector<KBlock*> targets;
  bool after_return = false;
  bool at_return = false;
  bool done = false;

  while(!done) {
    while (path_index != path.size() - 1 &&
           path.getBlock(path_index)->parent == current->parent) {
      path_index++;
    }

    if (path.getBlock(path_index - 1)->getKBlockType() == KBlockType::Call) {
      targets = {path.getBlock(path_index - 1)};
      at_return = false;
      if(path_index == path.size() - 1) {
        done = true;
      }
    } else if (path.getBlock(path_index)->parent != current->parent) {
      targets = current->parent->returnKBlocks;

      at_return = true;
    } else if (path_index == path.size() - 1) {
      targets = {path.getBlock(path_index)};
      at_return = false;
      done = true;
    }

    auto dismantled = dismantle(current, targets);
    for (auto blockpair: dismantled) {
      bool instruction_index = (blockpair.first == current && after_return) ? 1 : 0;
      inits.insert(std::make_pair(blockpair.first->instructions[instruction_index],
                                  Target(blockpair.second, at_return)));
    }

    if (path.getBlock(path_index - 1)->getKBlockType() == KBlockType::Call) {
      inits.insert(
          std::make_pair(path.getBlock(path_index - 1)->instructions[0],
                         Target(path.getBlock(path_index), false)));
      stack.push(path.getBlock(path_index - 1));
      current = path.getBlock(path_index);
      after_return = false;
    } else if(path.getBlock(path_index)->parent != current->parent) {
      current = stack.top();
      stack.pop();
      after_return = true;
    }
  }

  inits.insert(std::make_pair(path.getFinalBlock()->instructions[0], Target(b, false)));
  std::map<KInstruction*, std::set<Target>> ret;

  // std::cout << "Initializing: " << std::endl;

  for (auto init : inits) {
    if (!initialized[init.first].count(init.second) &&
        !(!init.second.at_end && init.first->parent == init.second.targetBlock)) {
      // std::cout << init.first->getIRLocation() << std::endl;
      // std::cout << init.second.print() << std::endl << std::endl;
      ret[init.first].insert(init.second);
      initialized[init.first].insert(init.second);
    }
  }
  for(auto i : ret) {
    validityCoreInits.push(i);
  }
}

};
