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
struct Conflict;

class Initializer {
public:
  virtual ~Initializer() {}
  virtual std::pair<KInstruction *, std::set<Target>> selectAction() = 0;
  virtual bool empty() = 0;
  virtual void addPob(ProofObligation *pob) = 0;
  virtual void removePob(ProofObligation *pob) = 0;
  virtual void addConflictInit(const Conflict &, KBlock *) = 0;

};

class ConflictCoreInitializer: public Initializer {
public:
  std::pair<KInstruction *, std::set<Target>> selectAction() override;
  bool empty() override;
  void addPob(ProofObligation *pob) override;
  void removePob(ProofObligation *pob) override;
  void addConflictInit(const Conflict &, KBlock *) override;

  explicit ConflictCoreInitializer(KInstruction *initInst) : initInst(initInst) {};
  ~ConflictCoreInitializer() override {}

private:
  KInstruction *initInst;
  std::map<KInstruction *, std::set<Target>> mapInitLocationToTargets;
  std::map<Target, std::vector<KInstruction *>> mapTargetToInitLocations;
  std::queue<std::pair<KInstruction *, std::set<Target>>> conflictCoreInits;
  std::map<KInstruction *, std::set<Target>> initialized;
  std::set<KFunction *> dismantledKFunctions;
};

};
