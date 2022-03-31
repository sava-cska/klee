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

public:
  KBlock *root;
  KBlock *location;
  ConstraintSet condition;
  std::vector<std::pair<ref<const MemoryObject>, const Array *>> symbolics;
  size_t lvl;
  size_t id;

  ProofObligation(KBlock *_location, ProofObligation *_parent, size_t _lvl)
    : parent(_parent), location(_location), lvl(_lvl), id(counter++) {
      root = parent ? parent->root : location;
      if (parent) {
        parent->children.insert(this);
      }
    }

  ProofObligation(KBlock* location) : ProofObligation(location, nullptr, 0) {}

  ~ProofObligation();

  void addCondition(ref<Expr> e, KInstruction *loc, bool *sat = 0);
  std::string print();
};

} // namespace klee
