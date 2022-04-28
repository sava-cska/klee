// -*- C++ -*-
#pragma once

#include "ExecutionState.h"
#include "Path.h"
#include "ProofObligation.h"
#include "klee/ADT/Ref.h"
#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Solver/Solver.h"
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
    void summarize(const Path& path, ProofObligation *pob,
                   const SolverQueryMetaData &metadata);
  };
}; // namespace klee
