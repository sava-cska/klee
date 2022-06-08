#include "Initializer.h"
#include "ExecutionState.h"
#include "Path.h"
#include "ProofObligation.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Solver/Solver.h"
#include "klee/Support/ModuleUtil.h"
#include "klee/Support/OptionCategories.h"

#include <algorithm>
#include <iostream>
#include <set>
#include <stack>
#include <utility>

namespace {
llvm::cl::opt<bool> DebugInitializer(
    "debug-initializer",
    llvm::cl::desc(""),
    llvm::cl::init(false),
    llvm::cl::cat(klee::DebugCat));
}

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
      bool atReturn =
          std::find(fn->returnKBlocks.begin(), fn->returnKBlocks.end(),
                    blockpair.second) != fn->returnKBlocks.end();

      inits.insert(std::make_pair(blockpair.first->instructions[0],
                                  Target(blockpair.second, atReturn)));

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
  if (DebugInitializer) {
    llvm::errs() << "Initialization of entry points and targets: \n";
  }
  for (auto init : inits) {
    if (!initialized[init.first].count(init.second) &&
        !(!init.second.atReturn && init.first->parent == init.second.targetBlock)) {
      if (DebugInitializer) {
        llvm::errs() << init.first->getIRLocation() << "\n" << init.first->getSourceLocation() << "\n";
        llvm::errs() << init.second.print() << "\n" << init.second.targetBlock->instructions[0]->getSourceLocation();
        llvm::errs() << "\n";
      }
      ret[init.first].insert(init.second);
      initialized[init.first].insert(init.second);
    }
  }
  if (DebugInitializer) {
    llvm::errs() << "\n";
  }
  for(auto i : ret) {
    validityCoreInits.push(i);
  }
}

};
 
