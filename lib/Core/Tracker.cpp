#include "Tracker.h"
#include <cassert>

namespace klee {
namespace {

using namespace llvm;
using Graph = typename DominationTree::Graph;

void debugPrint(char const * name, Graph const & graph) {
    dbgs() << name << ":\n";
    for (size_t v = 0; v < graph.size(); ++v) {
        dbgs() << v << " -> ";
        for (auto const & u : graph[v])
            dbgs() << u << ' ';
        dbgs() << '\n';
    }
}

void print(KBlock const & loc) {
    dbgs() << "----[ block id is " << loc.id << " ]----";
    loc.basicBlock->print(dbgs());
    dbgs() << "----[ end of " << loc.id << " block ]----\n";
}

void dfs(Graph & cfg, std::vector<bool> & used, KBlock const * loc) {
    used[loc->id] = true;
    for (BasicBlock* next_bb : successors(loc->basicBlock)) {
        KBlock* next_loc = loc->parent->blockMap[next_bb];
        cfg[loc->id].push_back(next_loc->id);
        if (!used[next_loc->id])
            dfs(cfg, used, next_loc);
    }

    if (loc->getKBlockType() == KBlockType::Call) {
        auto const & func = dynamic_cast<KCallBlock const *>(loc)->calledFunction;
        auto kmodule = loc->parent->parent;
        auto kfunc_iterator = kmodule->functionMap.find(func);
        if (kfunc_iterator == kmodule->functionMap.end()) {
            dbgs() << "Found function call that is outside of our module:\n";
            print(*loc);
        } else {
            KFunction * kfunc = kfunc_iterator->second;
            if (!used[kfunc->entryKBlock->id])
                dfs(cfg, used, kfunc->entryKBlock);

            // add edges (loc -> function entry block) and (function return block -> loc)
            cfg[loc->id].push_back(kfunc->entryKBlock->id);
            for (auto const & ret_loc : kfunc->finalKBlocks)
                cfg[ret_loc->id].push_back(loc->id);
        }
    }
}

Graph buildExplicitCFG(KBlock const & start_location) {
    std::vector<bool> used(KBlock::getTotalObjects(), false);
    Graph cfg(KBlock::getTotalObjects());
    dfs(cfg, used, &start_location);
    return cfg;
}

class DFSNumerator {
    using IdType = KBlock::IndexType;
    std::vector<IdType> oldToNew;
    std::vector<IdType> newToOld;
public:
    std::vector<IdType> parents;
    Graph reversedGraph;
    IdType freeId = 0;

    DFSNumerator(Graph const & g, IdType entryPoint)
        : oldToNew(g.size(), IdType(-1))
        , newToOld(g.size(), IdType(-1))
        , parents(g.size(), IdType(-1))
        , reversedGraph(g.size())
    {
        dfs(g, entryPoint, freeId);
        reversedGraph.resize(freeId);
    }

    bool IsReachably(IdType oldId) const noexcept {
        assert(isValid(oldId));
        return isValid(oldToNew[oldId]);
    }

    IdType GetNewId(IdType oldId) const noexcept {
        assert(isValid(oldId));
        assert(isValid(oldToNew[oldId]));
        return oldToNew[oldId];
    }

    IdType GetOldId(IdType newId) const noexcept {
        assert(isValid(newId));
        assert(isValid(newToOld[newId]));
        return newToOld[newId];
    }

private:
    bool isValid(IdType id) const noexcept {
        return id != IdType(-1);
    }

    void dfs(Graph const & g, IdType curVert, IdType & freeId) {
        oldToNew[curVert] = freeId;
        newToOld[freeId] = curVert;
        ++freeId;
        for (auto const & nextVert : g[curVert]) {
            if (!IsReachably(nextVert)) {
                dfs(g, nextVert, freeId);
                parents[oldToNew[nextVert]] = oldToNew[curVert];
            }
            reversedGraph[oldToNew[nextVert]].push_back(oldToNew[curVert]);
        }
    }

};

class DominatorTreeBuilder {
    using IdType = KBlock::IndexType;
    std::vector<IdType> parents;
    std::vector<IdType> sdom;
    std::vector<IdType> minSdomParents; // minSdomParents[i] is a parent with minimal sdom over all parents exclude root
    Graph const & reversedGraph;
    std::vector<IdType> const & dfsParents;
    IdType subRoot; // help value for Link method
public:
    DominatorTreeBuilder(Graph const & reversedGraph, std::vector<IdType> const & dfsParents)
        : parents(reversedGraph.size())
        , sdom(reversedGraph.size())
        , minSdomParents(reversedGraph.size())
        , reversedGraph(reversedGraph)
        , dfsParents(dfsParents)
    {
        for (IdType i = 0; i < reversedGraph.size(); ++i)
            minSdomParents[i] = sdom[i] = parents[i] = i;
    }

    std::vector<IdType> MakeDominators() {
        auto n = reversedGraph.size();
        std::vector<IdType> dom(n);
        Graph sdomTree(n);

        for (IdType i = 0; i < n; ++i)
            dom[i] = i;

        for (IdType v = n - 1; v > 0 /* we do not need to consider the root */; --v) {
            for (auto const & w : reversedGraph[v])
                sdom[v] = std::min(sdom[v], sdom[Eval(w)]);

            sdomTree[sdom[v]].push_back(v);
            // calculate dominators for sdomTree[v]
            for (auto const & u : sdomTree[v]) {
                // minimal sdom over all vertices in the path from sdom[u] to u
                auto minSdomParent = Eval(u);
                if (sdom[minSdomParent] == sdom[u]) {
                    dom[u] = sdom[u];
                } else {
                    // store the minimum sdom parent to update dom quickly
                    dom[u] = minSdomParent;
                }
            }

            Link(dfsParents[v], v);
        }

        // there is no smaller dominator than root
        for (auto const & u : sdomTree[0])
            dom[u] = 0;

        for (IdType i = 1; i < n; ++i) {
            if (dom[i] != sdom[i]) {
                // dom[i] holds parent with minimal sdom, and, therefore, dom
                dom[i] = dom[dom[i]];
            }
        }

        return dom;
    }


private:
    bool IsRoot(IdType a) const noexcept {
        return parents[a] == a;
    }

    void Link(IdType newRoot, IdType newChild) noexcept  {
        assert(parents[newChild] == newChild);
        assert(parents[newRoot] == newRoot);
        parents[newChild] = newRoot;
    }

    IdType Eval(IdType a) noexcept {
        if (IsRoot(a))
            return a;
        return EvalExcludeRoot(a);
    }

    IdType EvalExcludeRoot(IdType a) noexcept {
        if (IsRoot(parents[a]))
            return subRoot = a;

        IdType minSdomParent = EvalExcludeRoot(parents[a]);
        if (sdom[minSdomParent] < sdom[minSdomParents[a]])
            minSdomParents[a] = minSdomParent;

        // path compressing
        parents[a] = subRoot;

        return minSdomParents[a];
    }

};

} // namespace

DominationTree::DominationTree(KBlock const & start_location)
    : parents(KBlock::getTotalObjects(), IdType(-1))
    , children(KBlock::getTotalObjects())
{
    auto cfg = buildExplicitCFG(start_location);
    DFSNumerator numerator(cfg, start_location.id);

    DominatorTreeBuilder builder(numerator.reversedGraph, numerator.parents);

    auto dom = builder.MakeDominators();

    for (IdType i = 1; i < numerator.freeId; ++i) {
        auto v = numerator.GetOldId(i);
        auto domV = numerator.GetOldId(dom[i]);
        parents[v] = domV;
        children[domV].push_back(v);
    }

    /* uncomment to see result cfg and dom tree */
    // dbgs() << "entry point id: " << start_location.id << '\n';
    // debugPrint("cfg", cfg);
    // debugPrint("explicit dom tree", children);
}

Tracker::Tracker(KBlock const & start_location)
    : domTree(start_location)
    , locationIdToMarkedStates(KBlock::getTotalObjects())
{}

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
