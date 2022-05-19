#include "Initializer.h"
#include "ExecutionState.h"
#include "Path.h"
#include "ProofObligation.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Solver/Solver.h"
#include "klee/Support/ModuleUtil.h"

#include <algorithm>
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

// void ValidityCoreInitializer::addValidityCoreInit(std::pair<Path, SolverQueryMetaData::core_ty> v, KBlock* b) {
//   SolverQueryMetaData::core_ty core = v.second;
//   assert(!core.empty());
//   Path path = v.first;
//   std::set<std::pair<KInstruction*, Target>> inits;
//   std::stack<KBlock*> stack;
//   size_t path_index = 0;
//   size_t front = *(core.front().second);

//   inits.insert(std::make_pair(initInst, Target(path.getBlock(front), false)));

//   while(path_index != front) {
//     path_index++;
//     if(path.getBlock(path_index)->parent != path.getBlock(path_index-1)->parent) {
//       if(path.getBlock(path_index-1)->getKBlockType() == KBlockType::Call) {
//         stack.push(path.getBlock(path_index-1));
//       } else {
//         stack.pop();
//       }
//     }
//   }

//   KBlock* current = path.getBlock(front);
//   std::vector<KBlock*> targets;
//   bool after_return = false;
//   bool at_return = false;
//   bool done = false;

//   while(!done) {
//     while (path_index != path.size() - 1 &&
//            path.getBlock(path_index)->parent == current->parent) {
//       path_index++;
//     }

//     if (path.getBlock(path_index - 1)->getKBlockType() == KBlockType::Call) {
//       targets = {path.getBlock(path_index - 1)};
//       at_return = false;
//       if(path_index == path.size() - 1) {
//         done = true;
//       }
//     } else if (path.getBlock(path_index)->parent != current->parent) {
//       targets = current->parent->returnKBlocks;

//       at_return = true;
//     } else if (path_index == path.size() - 1) {
//       targets = {path.getBlock(path_index)};
//       at_return = false;
//       done = true;
//     }

//     auto dismantled = dismantle(current, targets);
//     for (auto blockpair: dismantled) {
//       bool instruction_index = (blockpair.first == current && after_return) ? 1 : 0;
//       bool target_at_return =
//           (at_return && std::find(targets.begin(), targets.end(),
//                                   blockpair.second) != targets.end()
//                ? true
//                : false);
//       inits.insert(std::make_pair(blockpair.first->instructions[instruction_index],
//                                   Target(blockpair.second, target_at_return)));
//     }

//     if (path.getBlock(path_index - 1)->getKBlockType() == KBlockType::Call) {
//       inits.insert(
//           std::make_pair(path.getBlock(path_index - 1)->instructions[0],
//                          Target(path.getBlock(path_index), false)));
//       stack.push(path.getBlock(path_index - 1));
//       current = path.getBlock(path_index);
//       after_return = false;
//     } else if(path.getBlock(path_index)->parent != current->parent) {
//       current = stack.top();
//       stack.pop();
//       after_return = true;
//     }
//   }

//   inits.insert(std::make_pair(path.getFinalBlock()->instructions[0], Target(b, false)));
//   std::map<KInstruction*, std::set<Target>> ret;

//   // std::cout << "Initializing: " << std::endl;

//   for (auto init : inits) {
//     if (!initialized[init.first].count(init.second) &&
//         !(!init.second.at_end && init.first->parent == init.second.targetBlock)) {
//       // std::cout << init.first->getIRLocation() << std::endl;
//       // std::cout << init.second.print() << std::endl << std::endl;
//       ret[init.first].insert(init.second);
//       initialized[init.first].insert(init.second);
//     }
//   }
//   for(auto i : ret) {
//     validityCoreInits.push(i);
//   }
// }

// New strategy (WIP)
void ValidityCoreInitializer::addValidityCoreInit(std::pair<Path, SolverQueryMetaData::core_ty> v, KBlock* b) {
  SolverQueryMetaData::core_ty core = v.second;
  assert(!core.empty());
  Path path = v.first;
  std::set<std::pair<KInstruction*, Target>> inits;
  KFunction* main_fn = initInst->parent->parent;
  KInstruction* current = initInst;
  std::set<KFunction*> visited;
  for(size_t path_index = 1; path_index < path.size(); path_index++) {
    if(path.getBlock(path_index)->parent != path.getBlock(path_index - 1)->parent) {
      if(path.getBlock(path_index - 1)->parent == main_fn) {
        auto dismantled = dismantle(current->parent, {path.getBlock(path_index - 1)});
        for(auto blockpair : dismantled) {
          inits.insert(std::make_pair(blockpair.first == current->parent ? current : blockpair.first->instructions[0],
                                      Target(blockpair.second, false)));
          if(blockpair.second->getKBlockType() == KBlockType::Call) {
            KFunction* f = dyn_cast<KCallBlock>(blockpair.second)->getKFunction();
            if (f) {
              inits.insert(std::make_pair(blockpair.second->instructions[0], Target(f->entryKBlock,false)));
            }
          }
        }
        inits.insert(std::make_pair(path.getBlock(path_index-1)->instructions[0], Target(path.getBlock(path_index),false)));
        current = path.getBlock(path_index - 1)->instructions[1];
      }
      if(path.getBlock(path_index)->parent != main_fn &&
         !dismantled_fns.count(path.getBlock(path_index)->parent)) {
        visited.insert(path.getBlock(path_index)->parent);
      }
    }
    if(path_index == path.size() - 1) {
      if(path.getBlock(path_index)->parent == main_fn &&
         path.getBlock(path_index) != current->parent) {
        auto dismantled = dismantle(current->parent, {path.getBlock(path_index)});
        for(auto blockpair : dismantled) {
          inits.insert(std::make_pair(blockpair.first == current->parent ? current : blockpair.first->instructions[0],
                                      Target(blockpair.second, false)));
          if(blockpair.second->getKBlockType() == KBlockType::Call) {
            KFunction* f = dyn_cast<KCallBlock>(blockpair.second)->getKFunction();
            if (f) {
              inits.insert(std::make_pair(blockpair.second->instructions[0], Target(f->entryKBlock,false)));
            }
          }
        }
      }
    }
  }
  for (auto fn : visited) {
    dismantled_fns.insert(fn);
    auto dismantled = dismantle(fn->entryKBlock, fn->returnKBlocks);
    for (auto blockpair : dismantled) {
      bool at_return =
          std::find(fn->returnKBlocks.begin(), fn->returnKBlocks.end(),
                    blockpair.second) != fn->returnKBlocks.end();

      inits.insert(std::make_pair(blockpair.first->instructions[0],
                                  Target(blockpair.second, at_return)));

      if (blockpair.second->getKBlockType() == KBlockType::Call) {
        KFunction *f =
            dyn_cast<KCallBlock>(blockpair.second)->getKFunction();
        if (f) {
          inits.insert(std::make_pair(blockpair.second->instructions[0],
                                      Target(f->entryKBlock, false)));
        }
      }
    }
  }
  inits.insert(std::make_pair(path.getFinalBlock()->instructions[0], Target(b, false)));

  std::map<KInstruction *, std::set<Target>> ret;
  llvm::errs() << "Initialization of entry points and targets: \n";
  for (auto init : inits) {
    if (!initialized[init.first].count(init.second) &&
        !(!init.second.at_end && init.first->parent == init.second.targetBlock)) {
      llvm::errs() << init.first->getIRLocation() << "\n" << init.first->getSourceLocation() << "\n";
      llvm::errs() << init.second.print() << "\n" << init.second.targetBlock->instructions[0]->getSourceLocation() << "\n";
      ret[init.first].insert(init.second);
      initialized[init.first].insert(init.second);
    }
  }
  for(auto i : ret) {
    validityCoreInits.push(i);
  }
}

};
 
