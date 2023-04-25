#ifndef KLEE_COMPOSER_H
#define KLEE_COMPOSER_H

#include "Executor.h"
#include "ProofObligation.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Module/KModule.h"
#include "klee/System/Time.h"


#include "ExecutionState.h"
#include "Memory.h"
#include "TimingSolver.h"
#include "../lib/Core/Executor.h"

#include <chrono>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace klee {
class ComposeVisitor;

class Composer {
  friend class klee::ComposeVisitor;

  Composer(ExecutionState &_state) : state(_state), copy(*state.copy()), composeID(++id) {}
  ~Composer();

private:
  static uint64_t id;
  static std::map<const ExecutionState *,
                  std::map<const MemoryObject *, ref<Expr>>,
                  ExecutionStateIDCompare>
      globalReadCache;
  static std::map<const ExecutionState *, ExprHashMap<ref<Expr>>,
                  ExecutionStateIDCompare>
      globalDerefCache;

  ExecutionState &state;
  ExecutionState &copy;
  uint64_t composeID;

  bool tryRebuild(const ref<Expr>, ref<Expr> &);

public:
  static Executor *executor;
  static bool tryRebuild(const ref<Expr>, ExecutionState &, ref<Expr> &);
  static bool tryRebuild(const ProofObligation *, const ProofObligation &,
                         ExecutionState &, ProofObligation &,
                         Conflict::core_ty &, ExprHashMap<ref<Expr>> &);
};

class ComposeVisitor : public ExprVisitor {
  friend class klee::Composer;

public:
  ComposeVisitor() = delete;
  explicit ComposeVisitor(Composer &_caller, int diff, uint64_t _id)
      : ExprVisitor(false), caller(_caller), diffLevel(diff), id(_id) {
    usedExternalVisitorHash = true;
    delete visited;
    visited = &globalVisited[&caller.state];
  }

  ~ComposeVisitor() {
    globalVisited.erase(&caller.state);
  }

private:
  ref<Expr> faultyPtr;
  static std::map<const ExecutionState *, visited_ty, ExecutionStateIDCompare>
      globalVisited;

  bool tryDeref(ref<Expr> ptr, unsigned size, ref<Expr> &result);
  ExprVisitor::Action visitRead(const ReadExpr &) override;
  ExprVisitor::Action visitConcat(const ConcatExpr &concat) override;
  ExprVisitor::Action visitSelect(const SelectExpr &) override;
  void shareUpdates(ref<ObjectState>, const ReadExpr &);
  ref<Expr> processObject(const MemoryObject *object, const Array *array);
  ref<Expr> processRead(const ReadExpr &);
  ref<Expr> processOrderedRead(const ConcatExpr & ce, const ReadExpr &re);
  ref<Expr> processSelect(const SelectExpr &);
  ref<Expr> reindexArray(const Array *array);
  Composer &caller;
  int diffLevel;
  uint64_t id;
};
} // namespace klee

#endif // KLEE_COMPOSITION_H
