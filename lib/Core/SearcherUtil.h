// -*- C++ -*-
#pragma once

#include "ProofObligation.h"
#include "klee/Module/KModule.h"
#include <variant>

namespace klee {

class ExecutionState;
class Executor;

struct Action {
  enum class Type { Init, Forward, Backward };

  Type type;
  ExecutionState* state; // Forward, Backward
  KBlock *location;      // Init
  ProofObligation *pob;  // Backward
  std::unordered_set<KBlock*> targets; // Init

  Action(ExecutionState *es)
      : type(Action::Type::Forward), state(es), location(nullptr), pob(nullptr),
        targets({}) {}
};

struct SearcherConfig {
  ExecutionState* initial_state;
  std::unordered_set<KBlock*> targets;
  // Hack
  Executor* executor;
};

struct ForwardResult {
  ExecutionState *current;
  // references to vectors?
  std::vector<ExecutionState *> addedStates;
  std::vector<ExecutionState *> removedStates;
  // _-_ In the future probably do not use references
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
  BackwardResult(ProofObligation *_newPob, ProofObligation *_oldPob)
      : newPob(_newPob), oldPob(_oldPob) {}
};

using ActionResult = std::variant<ForwardResult, BackwardResult>;

}
