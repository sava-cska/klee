// -*- C++ -*-
#pragma once

#include "Path.h"
#include "ProofObligation.h"
#include <map>
#include <vector>

namespace klee {
  struct Lemma {
    std::vector<Path> paths;
    std::vector<ref<Expr>> constraints;

    ref<Expr> getAsExpr();
  };

  class Summary {
  private:
    std::unique_ptr<llvm::raw_fd_ostream> &summaryFile;
  public:
    std::map<ProofObligation*, Lemma> lemmas;

  public:
    explicit Summary(std::unique_ptr<llvm::raw_fd_ostream> &_summaryFile)
        : summaryFile(_summaryFile) {}
    void summarize(const Path &path, ProofObligation *pob,
                   const SolverQueryMetaData &metadata,
                   ExprHashMap<ref<Expr>> &rebuildMap);
  };
}; // namespace klee
