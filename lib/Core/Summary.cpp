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
                        const SolverQueryMetaData &metaData,
                        ExprHashMap<ref<Expr>> &rebuildMap) {
  if(!metaData.queryValidityCore) {
    return;
  }

  std::cout << "Summary for pob at: " << pob->location->getIRLocation() << (pob->at_return ? "(at return)" : "") << std::endl;

  std::cout << "(Pob at "
            << (pob->at_return
                    ? pob->location->getFirstInstruction()->getSourceLocation()
                    : pob->location->getLastInstruction()->getSourceLocation())
            << std::endl;

  auto core = *metaData.queryValidityCore;
  auto& lemma = lemmas[pob];
  std::cout << "Constraints are:" << std::endl;
  for(auto constraint : core) {
    if(rebuildMap.count(constraint.first)) {
      lemma.constraints.push_back(Expr::createIsZero(rebuildMap.at(constraint.first)));
      std::cout << lemma.constraints.back()->print() << std::endl;
    }
  }
  std::cout << "State Path is:" << std::endl;
  std::cout << path.print() << std::endl;
  std::cout << "Pob Path is:" << std::endl;
  std::cout << pob->path.print() << std::endl;
  std::cout << std::endl;
  lemma.paths.push_back(path);
}
