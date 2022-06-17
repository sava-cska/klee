// -*- C++ -*-
#pragma once

#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Module/Cell.h"
#include <cstddef>
#include <functional>

namespace klee {

template <class T>
void hash_combine(std::size_t &seed, const T &v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

};

using namespace klee;

template<>
struct std::hash<KInstruction> {
  std::size_t operator()(KInstruction const & ki) const noexcept {
    // KModule* module = ki.parent->parent->parent;

    std::size_t seed = 0;
    hash_combine(seed, ki.toString());
    // hash_combine(seed, ki.inst->getOpcode());
    // for (unsigned i = 0; i < ki.inst->getNumOperands(); ++i) {
    //   int vnumber = ki.operands[i];
    //   if (vnumber < 0) {
    //     unsigned index = - vnumber - 2;
    //     auto constant = module->constantTable[index];
    //     hash_combine(seed, constant.value->computeHash());
    //   } else {
    //     hash_combine(seed, vnumber);
    //   }
    // }
    // hash_combine(seed, ki.dest);

    return seed;
  }
};

template<>
struct std::hash<KFunction> {
  std::size_t operator()(KFunction const & kf) const noexcept {
    std::size_t seed = 0;

    for (unsigned i = 0; i < kf.numInstructions; ++i) {
      hash_combine(seed, *(kf.instructions[i]));
    }

    return seed;
  }
};
