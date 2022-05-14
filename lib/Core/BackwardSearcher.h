// -*- C++ -*-
#pragma once

#include "ExecutionState.h"
#include "ProofObligation.h"
#include "SearcherUtil.h"

#include <queue>
#include <unordered_set>

namespace klee {

bool checkStack(ExecutionState* state, ProofObligation* pob);

class BackwardSearcher {
public:
  virtual ~BackwardSearcher() = default;
  virtual std::pair<ProofObligation*, ExecutionState*> selectAction() = 0;
  virtual void update(ProofObligation* pob) = 0;
  virtual void removePob(ProofObligation* pob) = 0;
  virtual bool empty() = 0;
};

class RecencyRankedSearcher : public BackwardSearcher {
private:
  std::vector<ProofObligation *> pobs;
  std::set<std::pair<ProofObligation *, ExecutionState *>> used;

public:
  ExecutionManager *emanager;

public:
  explicit RecencyRankedSearcher(ExecutionManager *_emanager)
    : emanager(_emanager) {}
  void update(ProofObligation* pob) override;
  std::pair<ProofObligation*, ExecutionState*> selectAction() override;
  bool empty() override;
  void removePob(ProofObligation* pob) override;
};

};
