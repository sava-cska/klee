#include "ExecutionState.h"
#include "klee/Module/KModule.h"

#include <vector>
#include <unordered_set>

namespace klee {

class DominationTree {
    using IdType = KBlock::IndexType;
    struct Node {
        std::vector<IdType> children;
        IdType parent;
        KBlock const & location;
    };
public:

    KBlock const * getParent(KBlock const & location) {
        auto const & node = locationIdToNode.at(location.id);
        if (node->parent == IdType(-1))
            return nullptr;
        return &node->location;
    }

    std::vector<IdType> const & getChildrenIds(KBlock const & location) {
        return locationIdToNode.at(location.id)->children;
    }

    KBlock const & getLocationById(IdType const & id) {
        return locationIdToNode.at(id)->location;
    }

private:
    std::vector<Node *> locationIdToNode;
};


class Tracker {
    using MarkedStates = std::unordered_set<ExecutionState*>;
    DominationTree domTree;

    std::vector<MarkedStates> locationIdToMarkedStates;

    void updateMarkedStatesForLocation(ExecutionState & state, std::vector<ExecutionState *> childrenStates, KBlock const & location);
    
    MarkedStates::const_iterator findFirstReachably(KBlock const & location, MarkedStates const & states);

public:

    void notifyDueToExecution(ExecutionState & state, std::vector<ExecutionState *> childrenStates, KBlock const & executedLocation);

    bool canReach(ExecutionState & state, KBlock const & location);

    bool isReachably(KBlock const & location);

    void reindex(ExecutionState & state, KBlock const & location);
};

} // namespace klee
