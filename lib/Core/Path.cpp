#include "Path.h"
#include "klee/Module/KModule.h"

#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/raw_ostream.h"
#include <string>
#include <vector>

using namespace klee;

KBlock* Path::getInitialBlock() const {
  return path.front();
}

KBlock* Path::getFinalBlock() const {
  return path.back();
}

KBlock* Path::getBlock(size_t index) const {
  return path[index];
}

std::string Path::toString() const {
  unsigned stackCount = 0;
  std::string repr = "";
  for (size_t i = 0; i<path.size(); i++) {
    if (i == 0 || (path[i-1]->parent != path[i]->parent &&
                   path[i-1]->getKBlockType() == KBlockType::Call)) {
      repr += "(";
      repr += path[i]->parent->function->getName();
      repr += ": ";
      stackCount++;
    }
    std::string label;
    llvm::raw_string_ostream label_stream(label);
    path[i]->basicBlock->printAsOperand(label_stream);
    repr += label_stream.str().erase(0, 6) + " ";
    if (i == path.size() - 1 || (path[i]->parent != path[i + 1]->parent &&
                                 path[i]->getKBlockType() != KBlockType::Call)) {
      repr.pop_back();
      repr += ") ";
      if (stackCount == 0) {
        std::string tmp = "(";
        tmp += path[i]->parent->function->getName();
        tmp += ": ";
        repr = tmp + repr;
      } else {
        stackCount--;
      }
    }
  }
  repr.pop_back();
  repr += std::string(")", stackCount);
  return repr;
}


Path klee::concat(const Path& lhs, const Path& rhs) {
  if(lhs.empty()) {
    return rhs;
  }
  if(rhs.empty()) {
    return lhs;
  }
  assert(lhs.getFinalBlock() == rhs.getInitialBlock() && "Paths are not compatible.");
  std::vector<KBlock*> path;
  path.reserve(lhs.path.size() + rhs.path.size() - 1);
  path.insert(path.end(), lhs.path.begin(), lhs.path.end());
  path.insert(path.end(), rhs.path.begin() + 1, rhs.path.end());
  return Path(std::move(path));
}
