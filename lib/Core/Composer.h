#ifndef KLEE_COMPOSER_H
#define KLEE_COMPOSER_H

#include "Executors/BidirectionalExecutor.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Module/KModule.h"
#include "klee/System/Time.h"


#include "ExecutionState.h"
#include "Memory.h"
#include "TimingSolver.h"
#include "../lib/Core/Executors/BaseExecutor.h"

#include <chrono>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace klee {
class ComposeVisitor;

class Composer {
  friend class klee::ComposeVisitor;

  Composer(ExecutionState *S1, ExecutionState *S2) : S1(S1), S2(S2) {
    assert(S1);
  }

  ~Composer() = default;

  void compose(ExecutionState *state, ExecutionState *&result);

  bool tryRebuild(const ref<Expr>, ref<Expr> &);

public:
  static void compose(ExecutionState *, ExecutionState *, ExecutionState *&);
  static bool tryRebuild(const ref<Expr>, ExecutionState *, ref<Expr> &);

private:
  static std::map<const ExecutionState *,
                  std::map<const MemoryObject *, ref<Expr>>,
                  ExecutionStateIDCompare>
      globalReadCache;
  static std::map<const ExecutionState *, ExprHashMap<ref<Expr>>,
                  ExecutionStateIDCompare>
      globalDerefCache;

  ExecutionState *S1;
  ExecutionState *S2;

public:
  static BidirectionalExecutor *executor;
  };

  class ComposeVisitor : public ExprVisitor {
    friend class klee::Composer;

  public:
    ComposeVisitor() = delete;
    explicit ComposeVisitor(Composer *_caller, int diff)
        : ExprVisitor(false), caller(_caller), diffLevel(diff) {
      assert(caller && caller->S1);
      visited = &globalVisited[caller->S1];
    }

  private:
    ref<Expr> faultyPtr;
    static std::map<const ExecutionState *, visited_ty, ExecutionStateIDCompare>
        globalVisited;

    bool tryDeref(ref<Expr> ptr, unsigned size, ref<Expr> &result);
    ExprVisitor::Action visitRead(const ReadExpr &);
    ExprVisitor::Action visitSelect(const SelectExpr &);
    ref<Expr> shareUpdates(ref<ObjectState>, const ReadExpr &);
    ref<Expr> processRead(const ReadExpr &);
    ref<Expr> processSelect(const SelectExpr &);
    ref<Expr> reindexRead(const ReadExpr &read);
    Composer *caller;
    int diffLevel;
  };
} // namespace klee

#endif // KLEE_COMPOSITION_H
