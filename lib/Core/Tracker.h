#pragma once

#include "ExecutionState.h"
#include "klee/Module/KModule.h"

#include <vector>
#include <unordered_set>

namespace klee {

class DominationTree {
public:
    using IdType = KBlock::IndexType;
    using Graph = std::vector<std::vector<IdType>>;

    DominationTree(KBlock const & start_location);

    KBlock const * getParent(KBlock const & location) const {
        auto const & parent = parents.at(location.id);
        if (parent == IdType(-1))
            return nullptr;
        return KBlock::getMemById(parent);
    }

    std::vector<IdType> const & getChildrenIds(KBlock const & location) const {
        return children.at(location.id);
    }

    KBlock const & getLocationById(IdType const & id) const {
        return *KBlock::getMemById(id);
    }

private:
    std::vector<IdType> parents;
    Graph children;
};


class Tracker {
    using MarkedStates = std::unordered_set<ExecutionState*>;

    DominationTree domTree;
    std::vector<MarkedStates> locationIdToMarkedStates;

    void updateMarkedStatesForLocation(ExecutionState & state, std::vector<ExecutionState *> childrenStates, KBlock const & location);

    MarkedStates::const_iterator findFirstReachably(KBlock const & location, MarkedStates const & states);

    bool canReach(ExecutionState & state, KBlock const & location);

public:
    Tracker(KBlock const & start_location);

    void notifyDueToExecution(ExecutionState & state, std::vector<ExecutionState *> childrenStates, KBlock const & executedLocation);

    bool isReachably(KBlock const & location);

    void reindex(ExecutionState & state, KBlock const & location);
};

} // namespace klee
