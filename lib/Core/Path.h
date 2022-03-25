// -*- C++ -*-
#pragma once
#include "llvm/IR/Instructions.h"
#include <string>
#include <vector>

struct Path {
  std::vector<llvm::BasicBlock*> path_;

  llvm::BasicBlock* getInitialBlock() const;
  llvm::BasicBlock* getFinalBlock() const;

  llvm::BasicBlock* getBlock(size_t index) const;

  void append(llvm::BasicBlock* bb) {
    path_.push_back(bb);
  }

  size_t getCurrentIndex() const {
    return path_.size() - 1;
  }

  std::string print() const;

  friend bool operator==(const Path& lhs, const Path& rhs) {
    return lhs.path_ == rhs.path_;
  }

  friend Path merge(const Path& lhs, const Path& rhs);

  Path() = default;
  explicit Path(std::vector<llvm::BasicBlock*> path) : path_(path) {}
};
