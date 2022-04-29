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
  public:
    std::map<ProofObligation*, Lemma> lemmas;

  public:
    void summarize(const Path &path, ProofObligation *pob,
                   const SolverQueryMetaData &metadata,
                   ExprHashMap<ref<Expr>> &rebuildMap);
  };
}; // namespace klee
