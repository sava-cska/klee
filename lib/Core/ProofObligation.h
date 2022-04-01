#pragma once

#include "Memory.h"

#include "klee/Expr/Constraints.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "llvm/Support/raw_ostream.h"

#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace klee {

class ExecutionState;
class MemoryObject;

class ProofObligation {
private:
  static size_t counter;
  size_t id;

public:
  ProofObligation *parent;
  std::unordered_set<ProofObligation *> children;
  std::vector<KInstruction*> stack;

  KBlock* location;
  ConstraintSet condition;

  // Indicates that this proof obligation was pushed from the outer stack frame, and so,
  // it is actually at the return statement of the current basic block.
  bool at_return;

  std::vector<std::pair<ref<const MemoryObject>, const Array *>> symbolics;

  ProofObligation(KBlock *_location, ProofObligation *_parent, bool at_return = false)
    : id(counter++), parent(_parent), stack(parent->stack), location(_location), at_return(at_return) {
    parent->children.insert(this);
  }

  explicit ProofObligation(ProofObligation *pob)
      : id(counter++), parent(pob->parent), stack(pob->stack),
        location(pob->location), condition(pob->condition),
        at_return(pob->at_return) {
    parent->children.insert(this);
  }

  ~ProofObligation() {
    if(parent) {
      parent->children.erase(this);
    }
  }

  bool isOriginPob() const noexcept { return !parent; }

  void addCondition(ref<Expr> e, KInstruction *loc, bool *sat = 0);
  std::string print();
};

} // namespace klee
