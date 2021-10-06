#include "Tracker.h"

namespace klee {

void Tracker::updateMarkedStatesForLocation(ExecutionState & state, std::vector<ExecutionState *> childrenStates, KBlock const & location) {
    auto & markedStates = locationIdToMarkedStates[location.id];
    if (canReach(state, location))
        markedStates.insert(&state);
    for (auto const & childState : childrenStates) {
        if (canReach(*childState, location))
            markedStates.insert(childState);
    }
}

void Tracker::notifyDueToExecution(ExecutionState & state, std::vector<ExecutionState *> childrenStates, KBlock const & executedLocation) {
    updateMarkedStatesForLocation(state, childrenStates, executedLocation);
    auto const & childrenIds = domTree.getChildrenIds(executedLocation);
    for (auto const & id : childrenIds) {
        auto const & child = domTree.getLocationById(id);
        updateMarkedStatesForLocation(state, childrenStates, child);
    }

    for (auto const & childState : childrenStates)
        reindex(*childState, executedLocation);
}

bool Tracker::canReach(ExecutionState & state, KBlock const & location) {
  // static_assert(false && "TODO");
    return false;
}

bool Tracker::isReachably(KBlock const & location) {
    auto* loc = &location;
    while (loc) {
        auto & states = locationIdToMarkedStates[loc->id];
        auto it = findFirstReachably(*loc, states);
        if (it != states.cend()) {
            states.erase(states.begin(), it);
            return true;
        }
        states.clear();
        loc = domTree.getParent(*loc);
    }
    return false;
}

auto Tracker::findFirstReachably(KBlock const & location, MarkedStates const & states) -> MarkedStates::const_iterator {
    auto it = states.cbegin();
    for (; it != states.cend(); ++it) {
        if (canReach(**it, location))
            return it;
    }
    return it;
}

void Tracker::reindex(ExecutionState & state, KBlock const & location) {
    auto* loc = &location;
    while (loc) {
        if (!canReach(state, *loc))
            return;
        locationIdToMarkedStates[loc->id].insert(&state);
        loc = domTree.getParent(*loc);
    }
}

} // namespace klee
