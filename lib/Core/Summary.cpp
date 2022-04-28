#include "Summary.h"
#include "klee/ADT/Ref.h"
#include "klee/Expr/Expr.h"
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
                        const SolverQueryMetaData &metadata) {
  if(!metadata.queryValidityCore) {
    return;
  }
  std::cout << "Summary for pob at: " << pob->location->getIRLocation() << std::endl;
  auto core = *metadata.queryValidityCore;
  auto& lemma = lemmas[pob];
  std::cout << "Constraints are:" << std::endl;
  for(auto constraint : core) {
    if(metadata.rebuildMap.count(constraint.first)) {
      lemma.constraints.push_back(Expr::createIsZero(metadata.rebuildMap.at(constraint.first)));
      std::cout << lemma.constraints.back()->print() << std::endl;
    }
  }
  std::cout << "Path is:" << std::endl;
  std::cout << path.print() << std::endl;
  lemma.paths.push_back(path);
}
