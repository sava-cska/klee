//===-- ForwardExecutor.h ----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Class to perform actual execution, hides implementation details from external
// interpreter.
//
//===----------------------------------------------------------------------===//

#pragma once

#include "BaseExecutor.h"

namespace klee {

class ForwardExecutor : public BaseExecutor {

public:
  ForwardExecutor(llvm::LLVMContext &ctx, const InterpreterOptions &opts,
               InterpreterHandler *ie);

  virtual ~ForwardExecutor() = default;

private:
  void run(ExecutionState &initialState) override;
};

} // namespace klee
