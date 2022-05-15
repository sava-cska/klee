// -*- C++ -*-
#pragma once

#include "ExecutionState.h"
#include "ProofObligation.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Solver/Solver.h"
#include <set>
#include <queue>

namespace klee {
class Initializer {
public:
  
  virtual std::pair<KInstruction*, std::set<Target>> selectAction() = 0;
  virtual bool empty() = 0;
  virtual void addPob(ProofObligation* pob) = 0;
  virtual void removePob(ProofObligation* pob) = 0;
  virtual void addValidityCoreInit(std::pair<Path,SolverQueryMetaData::core_ty>, KBlock*) = 0;

};

class ValidityCoreInitializer: public Initializer {
public:
  std::pair<KInstruction *, std::set<Target>> selectAction() override;
  bool empty() override;
  void addPob(ProofObligation *pob) override;
  void removePob(ProofObligation* pob) override;
  void addValidityCoreInit(std::pair<Path,SolverQueryMetaData::core_ty>, KBlock*) override;

  ValidityCoreInitializer(KInstruction* initInst) : initInst(initInst) {};

private:
  KInstruction* initInst;
  std::queue<std::pair<KInstruction *, std::set<Target>>> validityCoreInits;
  std::map<KInstruction*, std::set<Target>> initialized;
  std::set<KFunction*> dismantled_fns;
};

};
