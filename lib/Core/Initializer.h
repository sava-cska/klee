// -*- C++ -*-
#pragma once

#include "ProofObligation.h"
#include "klee/Module/KModule.h"
#include <unordered_set>
#include <queue>

namespace klee {
class Initializer {
public:
  
  virtual std::pair<KBlock*, std::unordered_set<KBlock*>> selectAction() = 0;
  virtual bool empty() = 0;
  virtual void addPob(ProofObligation* pob) = 0;
  virtual void addValidityCoreInit(std::pair<KBlock*,KBlock*>);
};

class SDInitializer : public Initializer {
public:
  std::pair<KBlock*, std::unordered_set<KBlock*>> selectAction() override;
  bool empty() override;
  void addPob(ProofObligation* pob) override;
  void addValidityCoreInit(std::pair<KBlock*,KBlock*>) override;

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
  void addValidityCoreInit(std::pair<KBlock*,KBlock*>) override;

  ForkInitializer(std::unordered_set<KBlock *> targets) : pobs(targets) {}

private:
  std::unordered_set<KBlock *> pobs;
  std::unordered_set<KBlock *> initializedLocs;
};

class ValidityCoreInitializer: public Initializer {
public:
  std::pair<KBlock *, std::unordered_set<KBlock *>> selectAction() override;
  bool empty() override;
  void addPob(ProofObligation *pob) override;
  void addValidityCoreInit(std::pair<KBlock*,KBlock*>) override;

  ValidityCoreInitializer(std::unordered_set<KBlock *> targets) {};
  
private:
  std::queue<std::pair<KBlock*, KBlock*>> validity_core_inits;
  
};

};
