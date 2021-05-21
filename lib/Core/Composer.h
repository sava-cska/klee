
#ifndef KLEE_COMPOSER_H
#define KLEE_COMPOSER_H

#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Module/KModule.h"
#include "klee/System/Time.h"

#include "ExecutionState.h"
#include "Memory.h"
#include "TimingSolver.h"

#include <map>
#include <vector>
#include <chrono>
#include <string>
#include <sstream>

namespace klee {
    class BidirectionalExecutor;
    class ComposeVisitor;

    const MemoryObject *extractObject(const ref<Expr> read);

    ref<Expr> branchResolvePointer(const ObjectPair &op,
                                   const ref<Expr> pointer,
                                   ref<Expr> falseCase,
                                   Expr::Width expectedWidth );


    class Composer {
        friend class klee::ComposeVisitor;

        /// @param curLImap a map between LI objects from \S2 and their possible 
        /// copies from S1
        Composer(const ExecutionState *S1, const ExecutionState *S2) :
            S1(S1), S2(S2) {
            assert(S1);
        }
        ~Composer() = default;


        /// @brief returns a composition of \state and \S2 in context \S1
        /// @param possible is false if the resulting state has unsat path constraints
        /// otherwise true
        void compose(ExecutionState *state, std::vector<ExecutionState*> &result);


        /// @brief adds composed constraints from S2 to @param acceptor and checks sat
        /// @return false if new constraints are unsat, otherwise true
        bool addComposedConstraints(ExecutionState & acceptor,
                                    time::Span timeout = time::Span::null,
                                    TimingSolver * solver = nullptr);


        /// @brief rebuilds @param e in \S1 context
        ref<Expr> rebuild(const ref<Expr> & e);

    public:

        static void compose(const ExecutionState *,
                            const ExecutionState *,
                            std::vector<ExecutionState*> &result);

        // static bool update( ExecutionState * S1,
        //                     const ExecutionState * S2);

        static ref<Expr> rebuild(const ref<Expr>, 
                                 const ExecutionState *);

    private:
        const ExecutionState *S1;
        const ExecutionState *S2;
        std::map<const MemoryObject*, ref<Expr> > liCache; 
    public:
        static BidirectionalExecutor *executor;
    };



    class ComposeVisitor : public ExprVisitor {
    public:
        ComposeVisitor() = delete;

        explicit ComposeVisitor(Composer *_caller) : ExprVisitor(false), caller(_caller) {}

    private:

        const ObjectState *getOS(const ExecutionState *, const MemoryObject *);

        ExprVisitor::Action visitRead(const ReadExpr &);

        ref<Expr> shareUpdates(ObjectState &, const ReadExpr &);

        ref<Expr> processRead(const ReadExpr &);

        Composer *caller;
    };
}


#endif //KLEE_COMPOSITION_H
