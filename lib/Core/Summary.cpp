#include "Summary.h"
#include "CoreStats.h"
#include "klee/ADT/Ref.h"
#include "klee/Expr/Expr.h"
#include "klee/Module/KModule.h"
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

#ifndef divider
#define divider(n) std::string(n, '-') + "\n"
#endif

void Summary::summarize(const Path& path, ProofObligation *pob,
                        const SolverQueryMetaData &metaData,
                        ExprHashMap<ref<Expr>> &rebuildMap) {
  if(!metaData.queryValidityCore) {
    return;
  }
  std::string label;
  llvm::raw_string_ostream label_stream(label);

  label_stream << "Add lemma for pob at: " << pob->location->getIRLocation() << (pob->at_return ? "(at return)" : "") << "\n";

  label_stream << "Pob at "
                 << (pob->at_return
                         ? pob->location->getFirstInstruction()->getSourceLocation()
                         : pob->location->getLastInstruction()->getSourceLocation())
                 << "\n";

  auto core = *metaData.queryValidityCore;
  auto &locationLemmas = lemmas[pob->location];
  if (locationLemmas.empty())
    ++stats::summarizedLocationCount;
  auto &lemma = locationLemmas[pob];
  label_stream << "Constraints are:\n";
  for(auto &constraint : core) {
    if(rebuildMap.count(constraint.first)) {
      lemma.constraints.push_back(Expr::createIsZero(rebuildMap.at(constraint.first)));
      label_stream << lemma.constraints.back()->toString() << "\n";
    }
  }

  label_stream << "State Path is:\n";
  label_stream << path.toString() << "\n";
  label_stream << "Pob Path is:\n";
  label_stream << pob->path.toString() << "\n";
  label_stream << "\n";

  (*summaryFile) << label_stream.str();

  if (std::find(lemma.paths.begin(), lemma.paths.end(), path) == lemma.paths.end()) {
    lemma.paths.push_back(path);

    llvm::errs() << label_stream.str();

    llvm::errs() << "Summary for pob at " << pob->location->getIRLocation() << "\n";

    llvm::errs() << "Paths:\n";
    for (auto &path : lemma.paths) {
      llvm::errs() << path.toString() << "\n";
    }
    ExprHashSet summary;
    for (auto &expr : lemma.constraints) {
      summary.insert(expr);
    }
    llvm::errs() << "Lemma:\n";
    llvm::errs() << "\n" << divider(30);
    for (auto &expr : summary) {
      llvm::errs() << divider(30);
      llvm::errs() << expr << "\n";
      llvm::errs() << divider(30);
    }
    llvm::errs() << divider(30);
    llvm::errs() << "\n";
  }
}
