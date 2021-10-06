//===-- BidirectionalSearcher.h ----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#pragma once
#include "BidirectionalSearcher.h"

namespace klee {

Action SimpleBidirectionalSearcher::selectState() {
  static_assert(false && "TODO");
}

bool SimpleBidirectionalSearcher::empty() {
  static_assert(false && "TODO");
  return true;
}

} // klee namespace
