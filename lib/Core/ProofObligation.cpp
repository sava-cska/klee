#include "ProofObligation.h"
#include "ExecutionState.h"

namespace klee {

size_t ProofObligation::counter = 0;

void ProofObligation::addAsUnblocked(ExecutionState & state) {
  unblocked.insert(&state);
}

void ProofObligation::block(ExecutionState & state) {
  auto it = unblocked.find(&state);
  assert(it != unblocked.end());
  unblocked.erase(it);
  blocking_locs.insert(state.getInitPCBlock());
}

void ProofObligation::unblockTree(ProofObligation & node) {
  for (auto child : node.children) {
    unblock(node);
    unblockTree(node);
    node.answered = true;
  }
}

void ProofObligation::unblock(ProofObligation & node) {
  // for (auto state : node.unblocked)
  //   state->unblock(node);
  node.unblocked.clear();
}

// ProofObligation* PobManager::makePob(KBlock* location) {
//   if(mainPobClosed[location] == true) {
//     return nullptr;
//   }
//   if(mainPobs.count(location)) {
//     return mainPobs[location];
//   }
//   auto pob = new ProofObligation(location);
//   pobs.insert(pob);
//   mainPobs[location] = pob;
//   return pob;
// }

// ProofObligation* PobManager::makePob(KBlock* location, ProofObligation* parent) {
  
// }
  
} // namespace klee
