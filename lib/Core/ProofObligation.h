#pragma once
#include "klee/Expr/Constraints.h"
#include "klee/Module/KModule.h"

#include <unordered_set>

namespace klee {

class ExecutionState;

class ProofObligation {

  ProofObligation *parent;
  std::unordered_set<ProofObligation *> children;

  std::unordered_set<ExecutionState *> unblocked;
  std::unordered_set<llvm::BasicBlock *> blocking_locs;

public:
  KBlock* location;
  ConstraintSet condition;
  size_t lvl;
  bool answered;

  ProofObligation(KBlock *location, ProofObligation *parent, size_t lvl)
      : parent(parent), location(location), lvl(lvl), answered(false) {}

  ProofObligation(KBlock* location)
      : ProofObligation(location, nullptr, 0) {}

  ProofObligation makeChild(KBlock * location, size_t lvl,
                               ConstraintSet &&condition) {
    ProofObligation child(location, this, lvl);
    child.condition = std::move(condition);
    return child;
  }

  void addAsUnblocked(ExecutionState &state);

  void block(ExecutionState &state);

  bool isUnreachable() const noexcept {
    return unblocked.empty() && children.empty();
  }

  bool isOriginPob() const noexcept { return !parent; }

  ProofObligation *propagateUnreachability() {
    auto current_pob = this;
    while (isUnreachable() && parent) {
      parent->children.erase(current_pob);
      current_pob->answered = true;
      current_pob = parent;
    }
    return current_pob;
  }

  ProofObligation *propagateReachability() {
    auto current_pob = this;
    while (current_pob->parent)
      current_pob = current_pob->parent;
    unblockTree(*current_pob);
    return current_pob;
  }

private:
  void unblockTree(ProofObligation &node);

  void unblock(ProofObligation &node);
};

} // namespace klee
