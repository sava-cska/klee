#include "klee/Core/Interpreter.h"

#include "Executors/ForwardExecutor.h"
#include "Executors/BidirectionalExecutor.h"

namespace klee {

Interpreter *Interpreter::create(llvm::LLVMContext &ctx, const InterpreterOptions &opts,
                                 InterpreterHandler *ih, ExecutionKind executionKind) {
  Interpreter* res = nullptr;
  switch (executionKind) {
    case ExecutionKind::Default : res = new ForwardExecutor(ctx, opts, ih); break;
    case ExecutionKind::Guided  : res = new BidirectionalExecutor(ctx, opts, ih); break;
    default                     : assert(false && "unreachable");
  }
  return res;
}

} // namespace klee
