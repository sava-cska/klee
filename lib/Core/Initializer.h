// -*- C++ -*-
#pragma once

#include "ExecutionState.h"
#include "ProofObligation.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include <set>
#include <queue>

namespace klee {
class Initializer {
public:
  
  virtual std::pair<KInstruction*, std::set<Target>> selectAction() = 0;
  virtual bool empty() = 0;
  virtual void addPob(ProofObligation* pob) = 0;
  virtual void removePob(ProofObligation* pob) = 0;
  virtual void addValidityCoreInit(std::pair<ExecutionState*,KBlock*>) = 0;

};

class ValidityCoreInitializer: public Initializer {
public:
  std::pair<KInstruction *, std::set<Target>> selectAction() override;
  bool empty() override;
  void addPob(ProofObligation *pob) override;
  void removePob(ProofObligation* pob) override;
  void addValidityCoreInit(std::pair<ExecutionState *,KBlock *>) override;

  ValidityCoreInitializer() {};

private:
  std::queue<std::pair<KInstruction *, Target>> validityCoreInits;
  std::unordered_map<llvm::BasicBlock *, std::unordered_set<llvm::BasicBlock *>> initializedLocs;
};

};
