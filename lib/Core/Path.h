// -*- C++ -*-
#pragma once
#include "klee/Module/KModule.h"
#include "llvm/IR/Instructions.h"
#include <functional>
#include <string>
#include <vector>

namespace klee {

struct Path {
  std::vector<KBlock*> path_;

  KBlock* getInitialBlock() const;
  KBlock* getFinalBlock() const;

  KBlock* getBlock(size_t index) const;

  void append(KBlock* bb) {
    path_.push_back(bb);
  }

  size_t getCurrentIndex() const {
    return path_.size() - 1;
  }

  size_t size() const {
    return path_.size();
  }

  bool empty() const {
    return path_.empty();
  }

  std::string print() const;

  friend bool operator==(const Path& lhs, const Path& rhs) {
    return lhs.path_ == rhs.path_;
  }

  friend Path merge(const Path& lhs, const Path& rhs);

  Path() = default;
  explicit Path(std::vector<KBlock*> path) : path_(path) {}
};
};
