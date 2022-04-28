#include "Path.h"
#include "klee/Module/KModule.h"

#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/raw_ostream.h"
#include <string>
#include <vector>

using namespace klee;

KBlock* Path::getInitialBlock() const {
  return path_.front();
}

KBlock* Path::getFinalBlock() const {
  return path_.back();
}

KBlock* Path::getBlock(size_t index) const {
  return path_[index];
}

std::string Path::print() const {
  std::string repr = "Path: ";
  for(auto i : path_) {
    std::string label;
    llvm::raw_string_ostream label_stream(label);
    i->basicBlock->printAsOperand(label_stream);
    repr += label_stream.str() + " ";
  }
  repr.pop_back();
  return repr;
}

Path merge(const Path& lhs, const Path& rhs) {
  if(lhs.empty()) {
    return rhs;
  }
  if(rhs.empty()) {
    return lhs;
  }
  assert(lhs.getFinalBlock() == rhs.getInitialBlock() && "Paths are not compatible.");
  std::vector<KBlock*> path;
  path.reserve(lhs.path_.size() + rhs.path_.size() - 1);
  path.insert(path.end(), lhs.path_.begin(), lhs.path_.end());
  path.insert(path.end(), rhs.path_.begin() + 1, rhs.path_.end());
  return Path(std::move(path));
}
