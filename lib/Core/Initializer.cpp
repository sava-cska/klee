#include "Initializer.h"
#include "ProofObligation.h"
#include <utility>

namespace klee {

bool SDInitializer::empty() {
  for(auto i : pobs) {
    auto distmap = i->parent->getSortedBackwardDistance(i);
    for(auto j: distmap) {
      if(initializedLocs.count(j.first)) continue;
      return false;
    }
    auto fdistmap = i->parent->parent->getSortedBackwardDistance(i->parent);
    for(auto j : fdistmap) {
      if(initializedLocs.count(j.first->entryKBlock)) continue;
      return false;
    }
  }
  return true;
}

std::pair<KBlock*, std::unordered_set<KBlock*>> SDInitializer::selectAction() {
  for(auto i : pobs) {
    auto distmap = i->parent->getSortedBackwardDistance(i);
    for(auto j: distmap) {
      if(initializedLocs.count(j.first)) continue;
      initializedLocs.insert(j.first);
      return std::make_pair(j.first, pobs);
    }
    auto fdistmap = i->parent->parent->getSortedBackwardDistance(i->parent);
    for(auto j : fdistmap) {
      if(initializedLocs.count(j.first->entryKBlock)) continue;
      initializedLocs.insert(j.first->entryKBlock);
      return std::make_pair(j.first->entryKBlock, pobs);
    }
  }
  assert(0);
}

void SDInitializer::addPob(ProofObligation* pob) {
  pobs.insert(pob->location);
}

void SDInitializer::removePob(ProofObligation* pob) {
  pobs.erase(pob->location);
}

void SDInitializer::addValidityCoreInit(std::pair<KBlock*,KBlock*> v, KFunction* f) {}

bool SDInitializer::pobsAtTargets() { return false; }


bool ForkInitializer::empty() {
  for(auto i : pobs) {
    auto distmap = i->parent->getSortedBackwardDistance(i);
    for(auto j: distmap) {
      if(j.second == 0) continue;
      if(!j.first->basicBlock->hasNPredecessorsOrMore(2)) continue;
      if(initializedLocs.count(j.first)) continue;
      return false;
    }
    auto fdistmap = i->parent->parent->getSortedBackwardDistance(i->parent);
    for(auto j : fdistmap) {
      if(initializedLocs.count(j.first->entryKBlock)) continue;
      return false;
    }
  }
  return true;
}

std::pair<KBlock*, std::unordered_set<KBlock*>> ForkInitializer::selectAction() {
  for(auto i : pobs) {
    auto distmap = i->parent->getSortedBackwardDistance(i);
    for(auto j: distmap) {
      if(j.second == 0) continue;
      if(!j.first->basicBlock->hasNPredecessorsOrMore(2)) continue;
      if(initializedLocs.count(j.first)) continue;
      initializedLocs.insert(j.first);
      return std::make_pair(j.first, pobs);
    }
    auto fdistmap = i->parent->parent->getSortedBackwardDistance(i->parent);
    for(auto j : fdistmap) {
      if(initializedLocs.count(j.first->entryKBlock)) continue;
      initializedLocs.insert(j.first->entryKBlock);
      return std::make_pair(j.first->entryKBlock, pobs);
    }
  }
  assert(0);
}

void ForkInitializer::addPob(ProofObligation* pob) {
  pobs.insert(pob->location);
}

void ForkInitializer::removePob(ProofObligation* pob) {
  pobs.erase(pob->location);
}

void ForkInitializer::addValidityCoreInit(std::pair<KBlock*,KBlock*> v, KFunction* f) {}

std::pair<KBlock *, std::unordered_set<KBlock *>> ValidityCoreInitializer::selectAction() {
  std::pair<KBlock*,KBlock*> v = validity_core_inits.front();
  validity_core_inits.pop();
  return std::make_pair(v.first, std::unordered_set({v.second}));
}

bool ForkInitializer::pobsAtTargets() { return false; }

bool ValidityCoreInitializer::empty() {
  return validity_core_inits.empty();
}

void ValidityCoreInitializer::addPob(ProofObligation *pob) {}
  
void ValidityCoreInitializer::removePob(ProofObligation* pob) {}

void ValidityCoreInitializer::addValidityCoreInit(std::pair<KBlock*,KBlock*> v, KFunction* f) {
  if(v.first->parent == f) {
   if(!knownLocs.count(v.first)) {
     validity_core_inits.push(v);
      knownLocs.insert(v.first);
    }
  } else {
    if(!knownLocs.count(f->entryKBlock)) {
      validity_core_inits.push(std::make_pair(f->entryKBlock,v.second));
      knownLocs.insert(f->entryKBlock);
    }
  }
}

bool ValidityCoreInitializer::pobsAtTargets() { return true; }

};
