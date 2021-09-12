#pragma once
#include <cstdint>
#include <vector>
#include <cassert>

namespace klee {

template <class T, std::uint32_t startId = 0>
struct Indexer {
    using IndexType = std::uint32_t;

    const IndexType id;
    static IndexType const & getTotalObjects() noexcept {
        return getFreeId();
    }

    Indexer(Indexer const &) : Indexer() {}
    Indexer & operator = (Indexer const &) = delete;

    Indexer(Indexer &&) = default;
    Indexer & operator = (Indexer &&) = default;

protected:
    Indexer() : id(getFreeId()++) {}
    Indexer(IndexType id) : id(id) {}

private:
    struct GlobalIdHolder {
        IndexType id = startId;
    };

    static IndexType & getFreeId() noexcept {
        static GlobalIdHolder freeId; // will be new at every different T value
        return freeId.id;
    }
};

template <class T>
struct IndexToMemMapper {
    IndexToMemMapper(T * mem, typename Indexer<T>::IndexType id) {
        auto & mapper = getMapper();
        assert(id == mapper.size());
        mapper.push_back(mem);
    }

    static T * getMemById(typename Indexer<T>::IndexType id) noexcept {
        assert(id < getMapper().size());
        return getMapper()[id];
    }

private:
    static std::vector<T *> & getMapper() noexcept {
        static std::vector<T *> mapper; // will be new at every different T value
        return mapper;
    }
};

} // namespace klee
