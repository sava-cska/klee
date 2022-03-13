#include "ProofObligation.h"
#include "ExecutionState.h"

namespace klee {

size_t ProofObligation::counter = 0;

ProofObligation ProofObligation::makeChild(KBlock * location, size_t lvl,
                              ConstraintSet &&condition) {
  ProofObligation child(location, this, lvl);
  child.condition = std::move(condition);
  return child;
}

void ProofObligation::block(ExecutionState & state) {
  auto it = unblocked.find(&state);
  assert(it != unblocked.end());
  unblocked.erase(it);
  blockingLocs.insert(state.getInitPCBlock());
}

bool ProofObligation::isUnreachable() const noexcept {
  return unblocked.empty() && children.empty();
}

bool ProofObligation::isOriginPob() const noexcept { return !parent; }

void ProofObligation::unblockTree(ProofObligation &node) {
  for (auto child : node.children) {
    unblock(*child);
    unblockTree(*child);
    node.answered = true;
  }
}

void ProofObligation::unblock(ProofObligation & node) {
  node.unblocked.clear();
}

ProofObligation *ProofObligation::propagateReachability() {
  auto current_pob = this;
  while (current_pob->parent)
    current_pob = current_pob->parent;
  unblockTree(*current_pob);
  return current_pob;
}

std::string ProofObligation::print() {
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

} // namespace klee
