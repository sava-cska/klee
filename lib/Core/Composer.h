
#ifndef KLEE_COMPOSER_H
#define KLEE_COMPOSER_H

#include "ExecutionState.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Module/KModule.h"
#include "klee/System/Time.h"
#include "Memory.h"
#include "TimingSolver.h"

#include <map>
#include <vector>
#include <chrono>
#include <string>
#include <sstream>

namespace klee {
    class ComposeVisitor;

    // each (LI object from S2) -> (possible object from S1)
    typedef std::map<const MemoryObject*, ResolutionList::iterator > ExactLImap;
    // each (LI object from S2) -> set<possible objects from S1>
    typedef std::map<const MemoryObject*, ResolutionList > GeneralLImap;
    // a wrapper for GeneralLImap with a comfortable iterator
    struct LImap
    {
        /// \invariant all components of iterator are not ends
        LImap() = delete;
        LImap(  const ExecutionState *S1,
                const ExecutionState *S2,
                TimingSolver *solver);
        
        /// @brief iterates all exact combinations of GeneralLImap lexicographically
        /// @return false if the last combination had been reached previously, true otherwise
        bool nextExact();
        const ExactLImap *getExact() { return &iterator; }

    private:
        ExactLImap iterator;
        GeneralLImap source;
    };


    class Composer {
        friend class klee::ComposeVisitor;

        /// @param curLImap a map between LI objects from \S2 and their possible 
        /// copies from S1
        Composer(   const ExecutionState *S1, 
                    const ExecutionState *S2, 
                    const ExactLImap *curLImap) :
            S1(S1), S2(S2), curLImap(curLImap)
        {
            assert(S1);
        }
        ~Composer() = default;


        /// @brief returns a composition of \state and \S2 in context \S1
        /// @param possible is false if the resulting state has unsat path constraints
        /// otherwise true
        void compose(ExecutionState *state, bool & possible) const;


        /// @brief adds composed constraints from S2 to @param acceptor and checks sat
        /// @return false if new constraints are unsat, otherwise true
        bool addComposedConstraints(ExecutionState & acceptor,
                    time::Span timeout = time::Span::null, 
                    TimingSolver * solver = nullptr) const;


        /// @brief rebuilds @param e in \S1 context
        ref<Expr> rebuild(const ref<Expr> & e) const;

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
        const ExactLImap *curLImap;
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
