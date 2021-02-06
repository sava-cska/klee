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
#include "Executor.h"
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


//---------LImap---------//
LImap::LImap(   const ExecutionState *S1,
                const ExecutionState *S2,
                TimingSolver *solver)
{
    assert(solver);
    for (const auto & op : S2->addressSpace.objects) {
        const auto & object = op.first;
        if(object->isLazyInstantiated()) {
            ref<Expr> LI = object->lazyInstantiatedSource;
            LI = Composer::rebuild(LI, S1);
            ResolutionList rl;
            S1->addressSpace.resolve(*S1, solver, LI, rl);
            source[object] = rl;
        }
    }
    for(auto & op : source) {
        iterator[op.first] = op.second.begin();
    }
}

bool LImap::nextExact() 
{
    assert(iterator.size() == source.size());
    if(source.empty()) return false;
    auto component = iterator.rbegin();     // both \iterator and \source have 
    auto componentList = source.rbegin();   // same order of elements
    for(; component != iterator.rend(); ++component, ++componentList) {
        if(++component->second != componentList->second.end()) {
            return true;
        }
        component->second = componentList->second.begin();
    }
    // all components of iterator are back elements
    return false;
}
//~~~~~~~~~LImap~~~~~~~~~//


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

void Composer::compose(
        const ExecutionState *S1,
        const ExecutionState *S2,
        std::vector<ExecutionState*> &result )
{
    assert(S1 || S2);
    result.clear();
    if(!S1) {
        result.push_back(new ExecutionState(*S2));
    } else if(!S2) {
        result.push_back(new ExecutionState(*S1));
    }
    if(S1->isEmpty()) {
        result.push_back(new ExecutionState(*S2));
    } else if(S2->isEmpty()) { 
        result.push_back(new ExecutionState(*S1));
    }
    if(!result.empty()) return;

    TimingSolver *solver = executor->getSolver();
    assert(solver);
    LImap liMap(S1, S2, solver);
    const Composer composer(S1, S2, liMap.getExact());
    
    bool possible = true;
    do { // updates iterator inside liMap
        ExecutionState *state = new ExecutionState(*S1);
        composer.compose(state, possible);
        if(!possible) {
            delete state;
            continue;
        }
        state->setID();
        result.push_back(state);
    } while(liMap.nextExact());
}

ref<Expr> Composer::rebuild(const ref<Expr> expr, 
                            const ExecutionState *S1)
{
    Composer composer(S1, nullptr, nullptr);
    return composer.rebuild(expr);
}
//~~~~~~~~~Composer~~~~~~~~~//