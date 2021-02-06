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
#include "Composer.h"

#include <stack>
#include <cassert>
#include <algorithm>

using namespace klee;

typedef std::pair<ref<const MemoryObject>, const Array *> symb;


//---------ComposeVisitor---------//
ExprVisitor::Action ComposeVisitor::visitRead(const ReadExpr & read)
{
    return Action::changeTo(processRead(read));
}
//~~~~~~~~~ComposeVisitor~~~~~~~~~//



//---------Composer---------//
ref<Expr> Composer::rebuild(const ref<Expr> & expr) const
{
    ComposeVisitor visitor(this);
    return visitor.visit(expr);
}

bool Composer::addComposedConstraints(ExecutionState & result,
                time::Span timeout, TimingSolver *solver) const
{
    ConstraintSet toCheck = result.constraints;
    for (auto & expr : S2->constraints)
    {
        auto newExpr = rebuild(expr);
        if(!result.tryAddConstraint(newExpr, timeout, solver)) {
            return false;
        }
    }
    // check if previous constraints became false
    for (auto & expr : toCheck)
    {
        auto simple = ConstraintManager::simplifyExpr(result.constraints, expr);
        if( simple->getWidth() == Expr::Bool &&
            simple->isFalse())
        {
            return false;
        }
    }
    return true;
}

ExecutionState *Composer::compose(
        const ExecutionState *S1,
        const ExecutionState *S2 )
{
    assert(S1 || S2);
    if(!S1) {
        return new ExecutionState(*S2);
    } else if(!S2) {
        return new ExecutionState(*S1);
    }
    if(S1->isEmpty()) {
        return new ExecutionState(*S2);
    } else if(S2->isEmpty()) { 
        return new ExecutionState(*S1);
    }

    bool possible = true;
    ExecutionState *state = new ExecutionState(*S1);
    const Composer composer(S1, S2);
    composer.compose(state, possible);
    if(!possible) {
        delete state;
        return nullptr;
    }
    return state;
}

bool Composer::update(
        ExecutionState *S1,
        const ExecutionState *S2 )
{
    assert(S1);
    if(!S2 || S2->isEmpty()) {
        // \S1 is not changed
        return true;
    }
    assert(!S1->isEmpty() && "trying to update an empty state");

    bool possible = true;
    const ExecutionState *context = new ExecutionState(*S1);
    PTreeNode *ptn = S1->ptreeNode;
    const Composer composer(context, S2);
    composer.compose(S1, possible);
    S1->ptreeNode = ptn;
    delete context;
    return possible;
}

ref<Expr> Composer::rebuild(const ref<Expr> expr, 
                            const ExecutionState *S1)
{
    Composer composer(S1, nullptr);
    return composer.rebuild(expr);
}
//~~~~~~~~~Composer~~~~~~~~~//