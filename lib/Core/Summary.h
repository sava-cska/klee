// -*- C++ -*-
#pragma once

#include "Path.h"
#include "ProofObligation.h"
#include <map>
#include <vector>
#include <unordered_map>

namespace klee {
  struct Lemma {
    std::vector<Path> paths;
    std::vector<ref<Expr>> constraints;

    ref<Expr> getAsExpr();
  };

  class Summary {
  private:
    llvm::raw_fd_ostream *summaryFile;
  public:
    std::unordered_map<klee::KBlock *, std::unordered_map<ProofObligation *, Lemma>> lemmas;

  public:
    explicit Summary(llvm::raw_fd_ostream *_summaryFile)
        : summaryFile(_summaryFile) {}
    ~Summary() = default;
    void summarize(const Path &path, ProofObligation *pob,
                   const SolverQueryMetaData &metadata,
                   ExprHashMap<ref<Expr>> &rebuildMap);
  };
}; // namespace klee
