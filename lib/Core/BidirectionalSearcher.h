//===-- BidirectionalSearcher.h -------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===--------------------------------------------------------------===//

#pragma once
#include "ProofObligation.h"
#include "Searcher.h"
#include "klee/Module/KModule.h"
#include <memory>
#include <unordered_set>

namespace klee {

class ExecutionState;

struct ActionMetadata {
  enum class ActionMetadataType {
    Base,
    Branch
  };  
  static bool classof(const ActionMetadata*) { return true; }
  virtual ActionMetadataType getType() const {
    return ActionMetadataType::Base;
  }
};

struct BranchMetadata : ActionMetadata {
  static bool classof(const BranchMetadata*) { return true; }
  static bool classof(const ActionMetadata *_m) {
    return _m->getType() == ActionMetadataType::Branch;
  }
  ActionMetadataType getType() const override {
    return ActionMetadataType::Branch;
  }

  GuidedSearcher* searcher;
  BranchMetadata(GuidedSearcher* _s) : searcher(_s) {}
};

struct Action {
  enum class Type { Init, Forward, Backward };

  Type type;
  ExecutionState &state; // Forward, Backward, Init
  KBlock *location;      // Init
  ProofObligation *pob;  // Backward
  ActionMetadata* metadata;
  // Метаданные? Они будут просто формироваться и передаваться
  // обратно к Searchers, легче чем хранить в каждом Searcher
  // дополнительные структуры. Можно использовать базовый
  // класс и LLVM RTTI.
  // (Привести пример с Branch, про выбранный Guided)
};

struct ActionResult {
  ActionMetadata* metadata;
  ActionResult(ActionMetadata* _m) : metadata(_m) {};
};

struct ForwardResult : ActionResult {
  ExecutionState* current;
  const std::vector<ExecutionState *> &addedStates;
  const std::vector<ExecutionState *> &removedStates;
  ForwardResult(ActionMetadata* _m, ExecutionState* _s,
                const std::vector<ExecutionState *> &a,
                const std::vector<ExecutionState *> &r)
    : ActionResult(_m), current(_s), addedStates(a), removedStates(r) {};
};

struct BackwardResult : ActionResult {
  ProofObligation* newPob;
  ProofObligation* oldPob;
  BackwardResult(ActionMetadata *_m, ProofObligation *_newPob,
                 ProofObligation *_oldPob)
      : ActionResult(_m), newPob(_newPob), oldPob(_oldPob) {}
};

// Можно комбинировать фичи и строить тактики через
// через множественное наследование (возможно, виртуальное)

class IForwardSearcher {
public:
  virtual Action selectAction() = 0;

  virtual std::vector<ExecutionState *>
  // Что за removedStates если removed максимум current?
  update(ForwardResult) = 0;

  virtual std::vector<ExecutionState *>
  updateTargets(const std::vector<KBlock *> &addedTargets,
              const std::vector<KBlock *> &removedTargets) = 0;
};

class IBranchSearcher {
public:
  
  virtual Action selectAction() = 0;

  // Вариант Саши?
  virtual void setTargets(ExecutionState *from,
                          std::unordered_set<KBlock *> to) = 0;
  
  virtual std::unordered_set<ExecutionState *>
  update(ForwardResult) = 0;
};

class IBackwardSearcher {
public:
  virtual Action selectAction() = 0;

  virtual std::unordered_set<ProofObligation *>
  addBranch(ExecutionState *state) = 0;

  virtual void update(BackwardResult) = 0;
};



class BidirectionalSearcher {
public:
  Action selectAction();

private:
  std::unique_ptr<IForwardSearcher> forwardSearcher;
  std::unique_ptr<IBranchSearcher> branchSearcher;
  std::unique_ptr<IBackwardSearcher> backwardSearcher;
};


class GuidedForwardSearcher : IForwardSearcher {
public:
  Action selectAction() override;

  std::vector<ExecutionState *>
  update(ForwardResult result) override;

  std::vector<ExecutionState *>
  updateTargets(const std::vector<KBlock *> &addedTargets,
              const std::vector<KBlock *> &removedTargets) override;

private:
  // Нужно держать +- все состояния? Вдруг прошли через то куда пришел
  // BackwardSearcher
  GuidedSearcher searcher;
};

class GuidedBranchSearcher : IBranchSearcher {
public:
  Action selectAction() override;

  virtual std::unordered_set<ExecutionState *>
  update(ForwardResult result) override;

  virtual void setTargets(ExecutionState *from,
                          std::unordered_set<KBlock *> to) override;

private:
  // Нужно передавать владение ExecutionState to BranchSearcher
  std::set<GuidedSearcher*> searchers;
  
};

} // namespace klee
