//===-- KInstruction.cpp --------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include <string>

using namespace llvm;
using namespace klee;

/***/

KInstruction::KInstruction(const KInstruction& ki):
  inst(ki.inst),
  info(ki.info),
  operands(ki.operands),
  dest(ki.dest) {}

KInstruction::~KInstruction() {
  delete[] operands;
}

KGEPInstruction::KGEPInstruction(const KGEPInstruction& ki):
  KInstruction(ki),
  indices(ki.indices),
  offset(ki.offset) {}

std::string KInstruction::getSourceLocation() const {
  if (!info->file.empty())
    return info->file + ":" + std::to_string(info->line) + " " +
           std::to_string(info->column);
  else return "[no debug info]";
}

std::string KInstruction::getIRLocation() const {
  size_t count = 0;
  while (parent->instructions[count] != this) {
    count++;
  }
  std::string repr = "Instruction " + std::to_string(count) + " in BasicBlock ";
  std::string label;
  llvm::raw_string_ostream label_stream(label);
  parent->basicBlock->printAsOperand(label_stream);
  repr += label_stream.str().substr(6);
  repr += " in function ";
  repr += parent->parent->function->getName();
  repr += " (";
  repr += inst->getOpcodeName();
  repr += ")";
  return repr;
}

bool KInstruction::isCallOrInvokeInst() {
  return inst->getOpcode() == llvm::Instruction::Call || inst->getOpcode() == llvm::Instruction::Invoke;
}
