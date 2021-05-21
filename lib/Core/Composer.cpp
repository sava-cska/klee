//
// Created by fomius2000 on 29.04.20.
//
#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"

#include "klee/Module/Cell.h"
#include "klee/Module/KModule.h"
#include "klee/Expr/Constraints.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Expr/Expr.h"
#include "ExecutionState.h"
#include "AddressSpace.h"
#include "Executors/BaseExecutor.h"
#include "Composer.h"

#include <stack>
#include <cassert>
#include <algorithm>

using namespace klee;

typedef std::pair<ref<const MemoryObject>, const Array *> symb;


//---------ComposeVisitor---------//
ExprVisitor::Action ComposeVisitor::visitRead(const ReadExpr & read) {
    return Action::changeTo(processRead(read));
}
//~~~~~~~~~ComposeVisitor~~~~~~~~~//


//---------Composer---------//
ref<Expr> Composer::rebuild(const ref<Expr> & expr) {
    ComposeVisitor visitor(this);
    return visitor.visit(expr);
}

bool Composer::addComposedConstraints(ExecutionState & result,
                time::Span timeout, TimingSolver *solver)
{
    ConstraintSet toCheck = result.constraints;
    for (auto & expr : S2->constraints) {
        auto newExpr = rebuild(expr);
        if(!result.tryAddConstraint(newExpr, timeout, solver)) {
            return false;
        }
    }
    // check if previous constraints became false
    for (auto & expr : toCheck) {
        auto simple = ConstraintManager::simplifyExpr(result.constraints, expr);
        if( simple->getWidth() == Expr::Bool &&
            simple->isFalse()) {
            return false;
        }
    }
    return true;
}

void Composer::compose(const ExecutionState *S1,
                       const ExecutionState *S2,
                       std::vector<ExecutionState*> &result) {
  assert(S1 || S2);
  if(!S1 || S1->isEmpty()) {
    result.push_back(S2->copy());
    return;
  } else if(!S2 || S2->isEmpty()) {
    result.push_back(S1->copy());
    return;
  }

    TimingSolver *solver = executor->getSolver();
    assert(solver);
    Composer composer(S1, S2);
    
    ExecutionState *copy = S1->copy();
    composer.compose(copy, result);
}

ref<Expr> Composer::rebuild(const ref<Expr> expr, 
                            const ExecutionState *S1) {
    Composer composer(S1, nullptr);
    return composer.rebuild(expr);
}
//~~~~~~~~~Composer~~~~~~~~~//


const MemoryObject *klee::extractObject(const ref<Expr> expr) {
    if(expr.isNull()) return nullptr;
    if(isa<ReadExpr>(expr.get())) {
        const ReadExpr *read = cast<const ReadExpr>(expr.get());
        return read->updates.root->binding;
    }
    if(isa<ConcatExpr>(expr.get())) {
        const ConcatExpr *conc = cast<const ConcatExpr>(expr.get());
        const auto leftRet = extractObject(conc->getLeft());
        const auto rightRet = extractObject(conc->getRight());
        assert(leftRet == rightRet && "can just return 0 otherwise");
        return leftRet;
    }
    return nullptr;
}

ref<Expr> klee::branchResolvePointer( const ObjectPair &op, 
                                            const ref<Expr> pointer, 
                                            ref<Expr> falseCase,
                                            Expr::Width expectedW ) {
    ref<Expr> address = op.first->getBaseExpr();
    ref<Expr> eq = EqExpr::create(pointer, address);
    ref<Expr> trueCase = op.second->read(0, 8*op.second->size);
    trueCase = SExtExpr::create(trueCase, expectedW);
    falseCase = SExtExpr::create(falseCase, expectedW);
    return SelectExpr::create(eq, trueCase, falseCase);
}
