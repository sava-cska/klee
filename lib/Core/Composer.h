
#ifndef KLEE_COMPOSER_H
#define KLEE_COMPOSER_H

#include "ExecutionState.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Module/KModule.h"
#include "klee/System/Time.h"
#include "TimingSolver.h"

#include <chrono>
#include <string>
#include <sstream>

namespace klee {
    class ComposeVisitor;

    class Composer {
        friend class klee::ComposeVisitor;

        Composer(const ExecutionState *S1, const ExecutionState *S2) :
                S1(S1), S2(S2) 
        {
            assert(S1 && S2);
        }
        ~Composer() = default;

        /// @brief returns a composition of \state and \S2 in context \S1
        void compose(ExecutionState *state, bool & possible) const;

        /// @brief adds composed constraints from S2 to acceptor and checks sat
        bool addComposedConstraints(ExecutionState & acceptor,
                    time::Span timeout = time::Span::null, 
                    TimingSolver * solver = nullptr) const;

        /// @brief rebuilds \e in S1 context
        ref<Expr> rebuild(const ref<Expr> & e) const;

    public:

        static ExecutionState *compose( const ExecutionState *, 
                                        const ExecutionState *);

        static bool update( ExecutionState * S1,
                            const ExecutionState * S2);

        static ref<Expr> rebuild(const ref<Expr>, 
                                 const ExecutionState *);

    private:
        const ExecutionState *S1;
        const ExecutionState *S2;
    public:
        static Executor *executor;
    };



    class ComposeVisitor : public ExprVisitor {
    public:
        ComposeVisitor() = delete;

        explicit ComposeVisitor(const Composer *_caller) : ExprVisitor(false), caller(_caller) {}
    
    private:

        const ObjectState *getOS(const ExecutionState *, const MemoryObject *);

        ExprVisitor::Action visitRead(const ReadExpr &);

        ref<Expr> shareUpdates(ObjectState &, const ReadExpr &);

        ref<Expr> processRead(const ReadExpr &);

        const Composer *caller;
    };
}


#endif //KLEE_COMPOSITION_H
