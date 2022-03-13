// -*- C++ -*-
#pragma once

#include "ProofObligation.h"
#include "klee/Module/KModule.h"
#include <unordered_set>
#include <variant>

namespace klee {

class ExecutionState;
class Executor;

struct Action {
  enum class Kind { Initialize, Forward, Backward, Terminate };

  Action() {}

  virtual Kind getKind() const = 0;

  static bool classof(const Action *) { return true; }
};

struct TerminateAction : Action {
  ExecutionState *state;

  TerminateAction() {}

  Kind getKind() const { return Kind::Terminate; }
  static bool classof(const Action *A) {
    return A->getKind() == Kind::Terminate;
  }
  static bool classof(const TerminateAction *) { return true; }
};

struct ForwardAction : Action {
  ExecutionState *state;

  ForwardAction(ExecutionState *_state) : state(_state) {}

  Kind getKind() const { return Kind::Forward; }
  static bool classof(const Action *A) {
    return A->getKind() == Kind::Forward;
  }
  static bool classof(const ForwardAction *) { return true; }
};

struct BackwardAction : Action {
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

struct InitializeAction : Action {
  KBlock *location;
  std::unordered_set<KBlock *> targets;
  bool makePobAtTargets;

  InitializeAction(KBlock *_location, std::unordered_set<KBlock *> _targets, bool _makePobAtTargets)
    : location(_location), targets(_targets), makePobAtTargets(_makePobAtTargets) {}

  Kind getKind() const { return Kind::Initialize; }
  static bool classof(const Action *A) {
    return A->getKind() == Kind::Initialize;
  }
  static bool classof(const InitializeAction *) { return true; }
};

struct SearcherConfig {
  ExecutionState *initial_state;
  std::unordered_set<KBlock *> targets;
  // Hack
  Executor *executor;
};

struct ForwardResult {
  ExecutionState *current;
  // references to vectors?
  std::vector<ExecutionState *> addedStates;
  std::vector<ExecutionState *> removedStates;
  // _-_ In the future probably do not use references
  // _-_ That's quite ugly, refactor later
  std::pair<ExecutionState *, KBlock *> validityCoreInit;
  ForwardResult(ExecutionState *_s, std::vector<ExecutionState *> &a,
                std::vector<ExecutionState *> &r)
      : current(_s), addedStates(a), removedStates(r){};
  // Way too easy to mistake for ...
  ForwardResult(ExecutionState *_s)
      : current(_s), addedStates({}), removedStates({}){};
};

struct BackwardResult {
  ProofObligation *newPob;
  ProofObligation *oldPob;
  // _-_ That's quite ugly, refactor later
  std::pair<KBlock*, KBlock*> validityCoreInit;
  KFunction* validity_core_function;
  BackwardResult(ProofObligation *_newPob, ProofObligation *_oldPob)
    : newPob(_newPob), oldPob(_oldPob), validityCoreInit(),
      validity_core_function(nullptr) {}
};

struct InitializeResult {
  KBlock* location;
  ExecutionState* state;
  std::unordered_set<ProofObligation*> pobs;
  InitializeResult(KBlock *_loc, ExecutionState *es, std::unordered_set<ProofObligation*> pobs) :
    location(_loc), state(es), pobs(pobs) {}
};

using ActionResult = std::variant<ForwardResult, BackwardResult, InitializeResult>;

}
