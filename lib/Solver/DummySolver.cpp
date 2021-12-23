//===-- DummySolver.cpp ---------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Solver/Solver.h"
#include "klee/Solver/SolverImpl.h"
#include "klee/Solver/SolverStats.h"

namespace klee {

class DummySolverImpl : public SolverImpl {
public:
  DummySolverImpl();

  bool computeValidity(const Query &, Solver::Validity &result, SolverQueryMetaData &metaData);
  bool computeTruth(const Query &, bool &isValid, SolverQueryMetaData &metaData);
  bool computeValue(const Query &, ref<Expr> &result, SolverQueryMetaData &metaData);
  bool computeInitialValues(const Query &,
                            const std::vector<const Array *> &objects,
                            std::vector<std::vector<unsigned char> > &values,
                            bool &hasSolution,
                            SolverQueryMetaData &metaData);
  SolverRunStatus getOperationStatusCode();
};

DummySolverImpl::DummySolverImpl() {}

bool DummySolverImpl::computeValidity(const Query &, Solver::Validity &result, SolverQueryMetaData &metaData) {
  ++stats::queries;
  // FIXME: We should have stats::queriesFail;
  return false;
}

bool DummySolverImpl::computeTruth(const Query &, bool &isValid, SolverQueryMetaData &metaData) {
  ++stats::queries;
  // FIXME: We should have stats::queriesFail;
  return false;
}

bool DummySolverImpl::computeValue(const Query &, ref<Expr> &result, SolverQueryMetaData &metaData) {
  ++stats::queries;
  ++stats::queryCounterexamples;
  return false;
}

bool DummySolverImpl::computeInitialValues(
    const Query &, const std::vector<const Array *> &objects,
    std::vector<std::vector<unsigned char> > &values, bool &hasSolution,
    SolverQueryMetaData &metaData) {
  ++stats::queries;
  ++stats::queryCounterexamples;
  return false;
}

SolverImpl::SolverRunStatus DummySolverImpl::getOperationStatusCode() {
  return SOLVER_RUN_STATUS_FAILURE;
}

Solver *createDummySolver() { return new Solver(new DummySolverImpl()); }
}
