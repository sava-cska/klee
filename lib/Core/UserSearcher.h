//===-- UserSearcher.h ------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#pragma once

#include <memory>

namespace klee {
  class BaseExecutor;
  class Searcher;

  // XXX gross, should be on demand?
  bool userSearcherRequiresMD2U();

  void initializeSearchOptions();

  std::unique_ptr<Searcher> constructUserSearcher(BaseExecutor &executor);
}
