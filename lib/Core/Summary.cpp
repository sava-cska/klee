#include "Summary.h"
#include "klee/ADT/Ref.h"
#include "klee/Expr/Expr.h"
#include "klee/Support/ErrorHandling.h"
#include <iostream>

using namespace klee;

ref<Expr> Lemma::getAsExpr() {
  ref<Expr> expr = ConstantExpr::create(0, Expr::Bool);
  for(auto e : constraints) {
    expr = OrExpr::create(expr, e);
  }
  return expr;
}

void Summary::summarize(const Path& path, ProofObligation *pob,
                        const SolverQueryMetaData &metaData,
                        ExprHashMap<ref<Expr>> &rebuildMap) {
  if(!metaData.queryValidityCore) {
    return;
  }

  (*summaryFile) << "Summary for pob at: " << pob->location->getIRLocation() << (pob->at_return ? "(at return)" : "") << "\n";

  (*summaryFile) << "Pob at "
                 << (pob->at_return
                         ? pob->location->getFirstInstruction()->getSourceLocation()
                         : pob->location->getLastInstruction()->getSourceLocation())
                 << "\n";

  auto core = *metaData.queryValidityCore;
  auto& lemma = lemmas[pob];
  (*summaryFile) << "Constraints are:\n";
  for(auto &constraint : core) {
    if(rebuildMap.count(constraint.first)) {
      lemma.constraints.push_back(Expr::createIsZero(rebuildMap.at(constraint.first)));
      (*summaryFile) << lemma.constraints.back()->toString() << "\n";
    }
  }

  (*summaryFile) << "State Path is:\n";
  (*summaryFile) << path.print() << "\n";
  (*summaryFile) << "Pob Path is:\n";
  (*summaryFile) << pob->path.print() << "\n";
  (*summaryFile) << "\n";

  lemma.paths.push_back(path);
}
