#include "Initializer.h"
#include "ExecutionState.h"
#include "ProofObligation.h"
#include "klee/Module/KInstruction.h"

#include <utility>

namespace klee {

std::pair<KInstruction*, std::set<Target>> ValidityCoreInitializer::selectAction() {
  auto v = validity_core_inits.front();
  validity_core_inits.pop();
  return v;
}

bool ValidityCoreInitializer::empty() {
  return validity_core_inits.empty();
}

void ValidityCoreInitializer::addPob(ProofObligation *pob) {}

void ValidityCoreInitializer::removePob(ProofObligation* pob) {}

void ValidityCoreInitializer::addValidityCoreInit(std::pair<ExecutionState*,KBlock*> v) {
  auto state = v.first;
  SolverQueryMetaData metaData = state->queryMetaData;
  assert(!metaData.queryValidityCores.empty());
  SolverQueryMetaData::core_ty core = metaData.queryValidityCores.back().second;
  assert(!core.empty());
  // Fix init
  // validity_core_inits.push(std::make_pair(core.front().second->parent, core.back().second->parent));
  // validity_core_inits.push(std::make_pair(core.back().second->parent, v.second));
  // validity_core_inits.push(std::make_pair(state->initPC->parent,core.front().second->parent));
}

};
