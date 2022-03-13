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
  std::unordered_set<llvm::BasicBlock *> blockingLocs;

public:
  KBlock *location;
  ConstraintSet condition;
  std::vector<std::pair<ref<const MemoryObject>, const Array *>> symbolics;
  size_t lvl;
  bool answered;

  size_t id;

  ProofObligation(KBlock *_location, ProofObligation *_parent, size_t _lvl)
    : parent(_parent), location(_location), lvl(_lvl), answered(false), id(counter++) {}

  ProofObligation(KBlock* location) : ProofObligation(location, nullptr, 0) {}

  ProofObligation makeChild(KBlock * location, size_t lvl, ConstraintSet &&condition);
  void block(ExecutionState &state);
  bool isUnreachable() const noexcept;
  bool isOriginPob() const noexcept;
  void unblockTree(ProofObligation &node);
  void unblock(ProofObligation &node);
  ProofObligation *propagateUnreachability();
  ProofObligation *propagateReachability();
  std::string print();
};

} // namespace klee
