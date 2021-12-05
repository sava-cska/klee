//===-- BidirectionalSearcher.h ----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "BidirectionalSearcher.h"
#include "Executor.h"
#include "ExecutionState.h"
#include "MergeHandler.h"
#include "ProofObligation.h"
#include "SearcherUtil.h"
#include "UserSearcher.h"
#include "klee/Core/Interpreter.h"
#include "klee/Module/KModule.h"
#include "klee/Support/ErrorHandling.h"
#include <llvm/ADT/StringExtras.h>
#include <memory>
#include <unordered_set>
#include <variant>
#include <vector>

#include <cstdlib>

namespace klee {

Action ForwardBidirSearcher::selectAction() {
  auto& state = searcher->selectState();
  if (state.targets.empty() &&
      state.multilevel.count(state.getPCBlock()) > 0 /* maxcycles - 1 */) {
    KBlock* target = ex->calculateCoverTarget(state);
    if(target) {
      state.targets.insert(target);
      ex->updateStates(&state);
    } else {
      ex->pauseState(state);
      ex->updateStates(nullptr);
    }
  }
  return Action(&state);
}

bool ForwardBidirSearcher::empty() {
  return searcher->empty();
}

void ForwardBidirSearcher::update(ActionResult r) {
  if(std::holds_alternative<ForwardResult>(r)) {
    auto fr = std::get<ForwardResult>(r);
    searcher->update(fr.current, fr.addedStates, fr.removedStates);
  }
}

ForwardBidirSearcher::ForwardBidirSearcher(SearcherConfig cfg) {
  searcher = new GuidedForwardSearcher(
      constructUserSearcher(*(Executor *)(cfg.executor)));
  for(auto target : cfg.targets) {
    cfg.initial_state->targets.insert(target);
  }
  searcher->update(nullptr,{cfg.initial_state},{});
  ex = cfg.executor;
}

} // namespace klee
