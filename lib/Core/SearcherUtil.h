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
  enum class Type { Init, Forward, Backward, Terminate };

  Type type;
  ExecutionState* state; // Forward, Backward
  KBlock *location;      // Init
  ProofObligation *pob;  // Backward
  std::unordered_set<KBlock*> targets; // Init
  bool makePobAtTargets; // Init

  Action(ExecutionState *es)
      : type(Action::Type::Forward), state(es), location(nullptr), pob(nullptr),
        targets({}), makePobAtTargets(false) {}

  Action(Type t, ExecutionState *es, KBlock *loc, ProofObligation *pob,
         std::unordered_set<KBlock *> targets, bool makePobAtTargets)
    : type(t), state(es), location(loc), pob(pob), targets(targets),
      makePobAtTargets(makePobAtTargets) {}
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
  std::pair<KBlock*, KBlock*> validity_core_init;
  KFunction* validity_core_function;
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
  std::pair<KBlock*, KBlock*> validity_core_init;
  KFunction* validity_core_function;
  BackwardResult(ProofObligation *_newPob, ProofObligation *_oldPob)
    : newPob(_newPob), oldPob(_oldPob), validity_core_init(),
      validity_core_function(nullptr) {}
};

struct InitResult {
  KBlock* location;
  ExecutionState* state;
  std::unordered_set<ProofObligation*> pobs;
  InitResult(KBlock *_loc, ExecutionState *es, std::unordered_set<ProofObligation*> pobs) :
    location(_loc), state(es), pobs(pobs) {}
};

using ActionResult = std::variant<ForwardResult, BackwardResult, InitResult>;

}
