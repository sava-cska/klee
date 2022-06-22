// -*- C++ -*-
#pragma once

#include "ExecutionState.h"
#include "Path.h"
#include "ProofObligation.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace klee {

class ExecutionState;
class Executor;

struct Action {
  enum class Kind { Initialize, Forward, Branch, Backward, Terminate };

  Action() = default;
  virtual ~Action() = default;

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


struct Conflict {
  using constraint_ty = std::pair<ref<Expr>, KInstruction *>;
  using core_ty = std::vector<constraint_ty>;

  Path path;
  core_ty core;

  Conflict() = default;
  Conflict(const Path &_path, const core_ty &_core) :
    path(_path), core(_core) {}
};

struct TargetedConflict {
  Conflict conflict;
  KBlock *target;

  TargetedConflict(Conflict &_conflict, KBlock *_target) :
    conflict(_conflict), target(_target) {}
};


struct ActionResult {
  enum class Kind { Initialize, Forward, Branch, Backward, Terminate };

  ActionResult() = default;
  virtual ~ActionResult() = default;

  virtual Kind getKind() const = 0;

  static bool classof(const ActionResult *) { return true; }
};

struct ForwardResult : ActionResult {
  ExecutionState *current;
  const std::vector<ExecutionState *> &addedStates;
  const std::vector<ExecutionState *> &removedStates;
  std::optional<TargetedConflict> targetedConflict;

  ForwardResult(ExecutionState *_s, const std::vector<ExecutionState *> &a,
                const std::vector<ExecutionState *> &r)

    : current(_s), addedStates(a), removedStates(r), targetedConflict(std::nullopt) {};

  Kind getKind() const { return Kind::Forward; }
  static bool classof(const ActionResult *A) {
    return A->getKind() == Kind::Forward;
  }
  static bool classof(const ForwardResult *) { return true; }
};

struct BranchResult : ActionResult {
  ExecutionState *current;
  const std::vector<ExecutionState *> &addedStates;
  const std::vector<ExecutionState *> &removedStates;

  BranchResult(ExecutionState *_s, const std::vector<ExecutionState *> &a,
               const std::vector<ExecutionState *> &r)
    : current(_s), addedStates(a), removedStates(r) {};

  Kind getKind() const { return Kind::Branch; }
  static bool classof(const ActionResult *A) {
    return A->getKind() == Kind::Branch;
  }
  static bool classof(const BranchResult *) { return true; }
};

struct BackwardResult : ActionResult {
  std::vector<ProofObligation*> newPobs;
  ProofObligation *oldPob;

  BackwardResult(std::vector<ProofObligation*> _newPobs, ProofObligation *_oldPob)
    : newPobs(_newPobs), oldPob(_oldPob) {}

  Kind getKind() const { return Kind::Backward; }
  static bool classof(const ActionResult *A) {
    return A->getKind() == Kind::Backward;
  }
  static bool classof(const BackwardResult *) { return true; }
};

struct InitializeResult : ActionResult {
  KInstruction *location;
  ExecutionState &state;

  InitializeResult(KInstruction *_loc, ExecutionState &es) :
    location(_loc), state(es) {}

  Kind getKind() const { return Kind::Initialize; }
  static bool classof(const ActionResult *A) {
    return A->getKind() == Kind::Initialize;
  }
  static bool classof(const InitializeResult *) { return true; }
};

struct TerminateResult : ActionResult {
  Kind getKind() const { return Kind::Terminate; }
  static bool classof(const ActionResult *A) {
    return A->getKind() == Kind::Terminate;
  }
  static bool classof(const TerminateResult *) { return true; }
};

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
