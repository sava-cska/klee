// -*- C++ -*-
#pragma once

#include "ExecutionState.h"
#include "Initializer.h"
#include "Path.h"
#include "ProofObligation.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include <optional>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace klee {

class ExecutionState;
class Executor;

struct Action {
  enum class Kind { Initialize, Forward, Branch, Backward, Terminate };

  Action() {}

  virtual Kind getKind() const = 0;

  static bool classof(const Action *) { return true; }
};

struct TerminateAction : public Action {
  TerminateAction() {}

  Kind getKind() const { return Kind::Terminate; }
  static bool classof(const Action *A) {
    return A->getKind() == Kind::Terminate;
  }
  static bool classof(const TerminateAction *) { return true; }
};

struct ForwardAction : public Action {
  ExecutionState *state;

  ForwardAction(ExecutionState *_state) : state(_state) {}

  Kind getKind() const { return Kind::Forward; }
  static bool classof(const Action *A) {
    return A->getKind() == Kind::Forward || A->getKind() == Kind::Branch;
  }
  static bool classof(const ForwardAction *) { return true; }
};

struct BranchAction : public ForwardAction {
  ExecutionState *state;

  BranchAction(ExecutionState *_state) : ForwardAction(_state) {}

  Kind getKind() const { return Kind::Branch; }
  static bool classof(const Action *A) {
    return A->getKind() == Kind::Branch;
  }
  static bool classof(const BranchAction *) { return true; }
};

struct BackwardAction : public Action {
  ExecutionState *state;
  ProofObligation *pob;

  BackwardAction(ExecutionState *_state, ProofObligation *_pob)
    : state(_state), pob(_pob) {}

  Kind getKind() const { return Kind::Backward; }
  static bool classof(const Action *A) {
    return A->getKind() == Kind::Backward;
  }
  static bool classof(const BackwardAction *) { return true; }
};

struct InitializeAction : public Action {
  KInstruction *location;
  std::set<Target> targets;

  InitializeAction(KInstruction *_location, std::set<Target> _targets)
    : location(_location), targets(_targets) {}

  Kind getKind() const { return Kind::Initialize; }
  static bool classof(const Action *A) {
    return A->getKind() == Kind::Initialize;
  }
  static bool classof(const InitializeAction *) { return true; }
};

struct SearcherConfig {
  ExecutionState *initial_state;
  Executor *executor;
};

struct ForwardResult {
  ExecutionState *current;
  const std::vector<ExecutionState *> &addedStates;
  const std::vector<ExecutionState *> &removedStates;

  struct ValidityCore {
    std::pair<Path, SolverQueryMetaData::core_ty> core;
    KBlock *target;

    ValidityCore(std::pair<Path, SolverQueryMetaData::core_ty> core, KBlock* target) :
      core(core), target(target) {}

    ValidityCore(Path path, SolverQueryMetaData::core_ty core, KBlock* target) :
      core(std::make_pair(path,core)), target(target) {}
  };

  std::optional<ValidityCore> validityCore;

  ForwardResult(ExecutionState *_s, const std::vector<ExecutionState *> &a,
                const std::vector<ExecutionState *> &r)
    : current(_s), addedStates(a), removedStates(r), validityCore(std::nullopt) {};
};

struct BranchResult {
  ExecutionState *current;
  const std::vector<ExecutionState *> &addedStates;
  const std::vector<ExecutionState *> &removedStates;

  BranchResult(ExecutionState *_s, const std::vector<ExecutionState *> &a,
               const std::vector<ExecutionState *> &r)
    : current(_s), addedStates(a), removedStates(r) {};
};

struct BackwardResult {
  std::vector<ProofObligation*> newPobs;
  ProofObligation *oldPob;

  BackwardResult(std::vector<ProofObligation*> _newPobs, ProofObligation *_oldPob)
    : newPobs(_newPobs), oldPob(_oldPob) {}
};

struct InitializeResult {
  KInstruction *location;
  ExecutionState &state;

  InitializeResult(KInstruction *_loc, ExecutionState &es) :
    location(_loc), state(es) {}
};

struct TerminateResult {};

using ActionResult = std::variant<ForwardResult, BranchResult, BackwardResult, InitializeResult, TerminateResult>;

class Ticker {
  std::vector<unsigned> ticks;
  size_t index = 0;
  unsigned counter = 0;

public:
  Ticker(std::vector<unsigned> ticks) : ticks(ticks) {}

  unsigned getCurrent() {
    unsigned current = index;
    counter += 1;
    if (counter == ticks[index]) {
      index = (index + 1) % ticks.size();
      counter = 0;
    }
    return current;
  }

  void moveToNext() {
    if (counter != 0) {
      index = (index + 1) % ticks.size();
      counter = 0;
    }
  }
};

}
