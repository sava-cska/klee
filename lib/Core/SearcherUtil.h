// -*- C++ -*-
#pragma once

#include "ExecutionState.h"
#include "ProofObligation.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include <unordered_set>
#include <variant>

namespace klee {

class ExecutionState;
class Executor;

struct Action {
  enum class Type { Init, Forward, Backward, Terminate };

  Type type;
  ExecutionState *state;    // Forward, Backward
  KInstruction *location;   // Init
  ProofObligation *pob;     // Backward
  std::set<Target> targets; // Init

  explicit Action(ExecutionState *es)
      : type(Action::Type::Forward), state(es), location(nullptr), pob(nullptr),
        targets({}) {}

  Action(Type t, ExecutionState *es, KInstruction *loc, ProofObligation *pob,
         std::set<Target> targets)
    : type(t), state(es), location(loc), pob(pob), targets(targets)
  {}
};

struct SearcherConfig {
  ExecutionState *initial_state;
  Executor *executor;
};

struct ForwardResult {
  ExecutionState *current;
  std::vector<ExecutionState *> addedStates;
  std::vector<ExecutionState *> removedStates;
  std::pair<ExecutionState*, KBlock*> validity_core_init;
  ForwardResult(ExecutionState *_s, std::vector<ExecutionState *> &a,
                std::vector<ExecutionState *> &r)
      : current(_s), addedStates(a), removedStates(r){};

  ForwardResult(ExecutionState *_s)
      : current(_s), addedStates({}), removedStates({}){};
};

struct BackwardResult {
  std::vector<ProofObligation*> newPobs;
  ProofObligation *oldPob;

  BackwardResult(std::vector<ProofObligation*> _newPobs, ProofObligation *_oldPob)
    : newPobs(_newPobs), oldPob(_oldPob) {}
};

struct InitResult {
  KInstruction* location;
  ExecutionState* state;

  InitResult(KInstruction *_loc, ExecutionState *es) :
    location(_loc), state(es) {}
};

using ActionResult = std::variant<ForwardResult, BackwardResult, InitResult>;

}
