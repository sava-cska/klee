#pragma once

#include "Memory.h"

#include "klee/Expr/Constraints.h"
#include "klee/Module/KModule.h"
#include "llvm/Support/raw_ostream.h"

#include <sstream>
#include <string>
#include <unordered_set>

namespace klee {

class ExecutionState;
class MemoryObject;

class ProofObligation {
public:
  static size_t counter;
  ProofObligation *parent;
  std::unordered_set<ProofObligation *> children;

  std::unordered_set<ExecutionState *> unblocked;
  std::unordered_set<llvm::BasicBlock *> blocking_locs;

public:
  KBlock* location;
  ConstraintSet condition;
  std::vector<std::pair<ref<const MemoryObject>, const Array *>> symbolics;
  size_t lvl;
  bool answered;

  size_t id;

  ProofObligation(KBlock *_location, ProofObligation *_parent, size_t _lvl)
    : parent(_parent), location(_location), lvl(_lvl), answered(false), id(counter++) {}

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

  std::string print() {
    std::string ret;
    std::string s;
    llvm::raw_string_ostream ss(s);
    location->basicBlock->printAsOperand(ss, false);
    ret += "Proof Obligation at " + ss.str();
    ret += " id: " + std::to_string(id) + '\n';
    ret += "The conditions are:\n";
    if(condition.empty()) ret += "None\n";
    for(auto i : condition) {
      ret += i->toString() + " at " + condition.get_location(i)->getSourceLocation() + '\n';
    }
    ret += "Children: ";
    if(children.empty()) ret += "None";
    for(auto i : children) {
      ret += std::to_string(i->id) + " ";
    }
    return ret;
  }

private:
  void unblockTree(ProofObligation &node);

  void unblock(ProofObligation &node);
};

} // namespace klee
