// -*- C++ -*-
#pragma once

#include "ProofObligation.h"
#include "klee/Module/KModule.h"
#include <unordered_set>

namespace klee {
class Initializer {
public:
  
  virtual std::pair<KBlock*, std::unordered_set<KBlock*>> selectAction() = 0;
  virtual bool empty() = 0;
  virtual void addPob(ProofObligation* pob) = 0; 
};

class SDInitializer : public Initializer {
public:
  std::pair<KBlock*, std::unordered_set<KBlock*>> selectAction() override;
  bool empty() override;
  void addPob(ProofObligation* pob) override;

  SDInitializer(std::unordered_set<KBlock *> targets) : pobs(targets) {}

private:
  std::unordered_set<KBlock *> pobs;
  std::unordered_set<KBlock *> initializedLocs;
};

class ForkInitializer : public Initializer {
public:
  std::pair<KBlock *, std::unordered_set<KBlock *>> selectAction() override;
  bool empty() override;
  void addPob(ProofObligation *pob) override;

  ForkInitializer(std::unordered_set<KBlock *> targets) : pobs(targets) {}

private:
  std::unordered_set<KBlock *> pobs;
  std::unordered_set<KBlock *> initializedLocs;
};

};
