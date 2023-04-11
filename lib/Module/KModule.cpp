//===-- KModule.cpp -------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "llvm/Support/Casting.h"
#include <functional>
#define DEBUG_TYPE "KModule"

#include "Passes.h"

#include "klee/Config/Version.h"
#include "klee/Core/Interpreter.h"
#include "klee/Support/OptionCategories.h"
#include "klee/Module/Cell.h"
#include "klee/Module/InstructionInfoTable.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Support/Debug.h"
#include "klee/Support/ErrorHandling.h"
#include "klee/Support/ModuleUtil.h"
#include "klee/Support/Hashing.h"

#if LLVM_VERSION_CODE >= LLVM_VERSION(4, 0)
#include "llvm/Bitcode/BitcodeWriter.h"
#else
#include "llvm/Bitcode/ReaderWriter.h"
#endif
#if LLVM_VERSION_CODE < LLVM_VERSION(8, 0)
#include "llvm/IR/CallSite.h"
#endif
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/ValueSymbolTable.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Linker/Linker.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/raw_os_ostream.h"
#include "llvm/Transforms/Scalar.h"
#if LLVM_VERSION_CODE >= LLVM_VERSION(8, 0)
#include "llvm/Transforms/Scalar/Scalarizer.h"
#endif
#include "llvm/Transforms/Utils/Cloning.h"
#if LLVM_VERSION_CODE >= LLVM_VERSION(7, 0)
#include "llvm/Transforms/Utils.h"
#endif

#include <sstream>

using namespace llvm;
using namespace klee;

namespace klee {
cl::OptionCategory
    ModuleCat("Module-related options",
              "These options affect the compile-time processing of the code.");
}

namespace {
  enum SwitchImplType {
    eSwitchTypeSimple,
    eSwitchTypeLLVM,
    eSwitchTypeInternal
  };

  cl::opt<bool>
  OutputSource("output-source",
               cl::desc("Write the assembly for the final transformed source (default=true)"),
               cl::init(true),
	       cl::cat(ModuleCat));

  cl::opt<bool>
  OutputModule("output-module",
               cl::desc("Write the bitcode for the final transformed module"),
               cl::init(false),
	       cl::cat(ModuleCat));

  cl::opt<SwitchImplType>
  SwitchType("switch-type", cl::desc("Select the implementation of switch (default=internal)"),
             cl::values(clEnumValN(eSwitchTypeSimple, "simple", 
                                   "lower to ordered branches"),
                        clEnumValN(eSwitchTypeLLVM, "llvm", 
                                   "lower using LLVM"),
                        clEnumValN(eSwitchTypeInternal, "internal", 
                                   "execute switch internally")
                        KLEE_LLVM_CL_VAL_END),
             cl::init(eSwitchTypeInternal),
	     cl::cat(ModuleCat));
  
  cl::opt<bool>
  DebugPrintEscapingFunctions("debug-print-escaping-functions", 
                              cl::desc("Print functions whose address is taken (default=false)"),
			      cl::cat(ModuleCat));

  // Don't run VerifierPass when checking module
  cl::opt<bool>
  DontVerify("disable-verify",
             cl::desc("Do not verify the module integrity (default=false)"),
             cl::init(false), cl::cat(klee::ModuleCat));

  cl::opt<bool>
      OptimiseKLEECall("klee-call-optimisation",
                       cl::desc("Allow optimization of functions that "
                                "contain KLEE calls (default=true)"),
                       cl::init(true), cl::cat(ModuleCat));

  cl::opt<bool>
      SplitCalls("split-calls",
                 cl::desc("Split each call in own basic block (default=true)"),
                 cl::init(true), cl::cat(klee::ModuleCat));

  cl::opt<bool> SplitReturns(
      "split-returns",
      cl::desc("Split each return in own basic block (default=true)"),
      cl::init(true), cl::cat(klee::ModuleCat));
}

/***/

namespace llvm {
extern void Optimize(Module *, llvm::ArrayRef<const char *> preservedFunctions);
}

// what a hack
static Function *getStubFunctionForCtorList(Module *m,
                                            GlobalVariable *gv, 
                                            std::string name) {
  assert(!gv->isDeclaration() && !gv->hasInternalLinkage() &&
         "do not support old LLVM style constructor/destructor lists");

  std::vector<Type *> nullary;

  Function *fn = Function::Create(FunctionType::get(Type::getVoidTy(m->getContext()),
						    nullary, false),
				  GlobalVariable::InternalLinkage, 
				  name,
                              m);
  BasicBlock *bb = BasicBlock::Create(m->getContext(), "entry", fn);
  llvm::IRBuilder<> Builder(bb);

  // From lli:
  // Should be an array of '{ int, void ()* }' structs.  The first value is
  // the init priority, which we ignore.
  auto arr = dyn_cast<ConstantArray>(gv->getInitializer());
  if (arr) {
    for (unsigned i=0; i<arr->getNumOperands(); i++) {
      auto cs = cast<ConstantStruct>(arr->getOperand(i));
      // There is a third element in global_ctor elements (``i8 @data``).
#if LLVM_VERSION_CODE >= LLVM_VERSION(9, 0)
      assert(cs->getNumOperands() == 3 &&
             "unexpected element in ctor initializer list");
#else
      // before LLVM 9.0, the third operand was optional
      assert((cs->getNumOperands() == 2 || cs->getNumOperands() == 3) &&
             "unexpected element in ctor initializer list");
#endif
      auto fp = cs->getOperand(1);
      if (!fp->isNullValue()) {
        if (auto ce = dyn_cast<llvm::ConstantExpr>(fp))
          fp = ce->getOperand(0);

        if (auto f = dyn_cast<Function>(fp)) {
          Builder.CreateCall(f);
        } else {
          assert(0 && "unable to get function pointer from ctor initializer list");
        }
      }
    }
  }

  Builder.CreateRetVoid();

  return fn;
}

static void
injectStaticConstructorsAndDestructors(Module *m,
                                       llvm::StringRef entryFunction) {
  GlobalVariable *ctors = m->getNamedGlobal("llvm.global_ctors");
  GlobalVariable *dtors = m->getNamedGlobal("llvm.global_dtors");

  if (!ctors && !dtors)
    return;

  Function *mainFn = m->getFunction(entryFunction);
  if (!mainFn)
    klee_error("Entry function '%s' not found in module.",
               entryFunction.str().c_str());

  if (ctors) {
    if (mainFn->begin()->begin()->getOpcode() == Instruction::Alloca) {
      llvm::BasicBlock::iterator it = mainFn->begin()->begin();
      llvm::BasicBlock::iterator ie = mainFn->begin()->end();
      for (; it != ie && it->getOpcode() == Instruction::Alloca; it++);
      llvm::IRBuilder<> Builder(&*it);
      Builder.CreateCall(getStubFunctionForCtorList(m, ctors, "klee.ctor_stub"));
    } else {
      llvm::IRBuilder<> Builder(&*mainFn->begin()->begin());
      Builder.CreateCall(getStubFunctionForCtorList(m, ctors, "klee.ctor_stub"));
    }
  }

  if (dtors) {
    Function *dtorStub = getStubFunctionForCtorList(m, dtors, "klee.dtor_stub");
    for (Function::iterator it = mainFn->begin(), ie = mainFn->end(); it != ie;
         ++it) {
      if (isa<ReturnInst>(it->getTerminator())) {
        llvm::IRBuilder<> Builder(it->getTerminator());
        Builder.CreateCall(dtorStub);
      }
    }
  }
}

void KModule::addInternalFunction(const char* functionName){
  Function* internalFunction = module->getFunction(functionName);
  if (!internalFunction) {
    KLEE_DEBUG(klee_warning(
        "Failed to add internal function %s. Not found.", functionName));
    return ;
  }
  KLEE_DEBUG(klee_message("Added function %s.",functionName));
  internalFunctions.insert(internalFunction);
}

void KModule::calculateBackwardDistance(KFunction *kf) {
  std::map<KFunction*, unsigned int> &bdist = backwardDistance[kf];
  std::vector<std::pair<KFunction*, unsigned int>> &bsort = sortedBackwardDistance[kf];
  std::deque<KFunction*> nodes;
  nodes.push_back(kf);
  bdist[kf] = 0;
  bsort.push_back({kf, 0});
  while(!nodes.empty()) {
    KFunction *currKF = nodes.front();
    for (auto &cf : callMap[currKF->function]) {
      if (cf->isDeclaration()) continue;
      KFunction *callKF = functionMap[cf];
      if (bdist.find(callKF) == bdist.end()) {
        bdist[callKF] = bdist[currKF] + 1;
        bsort.push_back({callKF, bdist[currKF] + 1});
        nodes.push_back(callKF);
      }
    }
    nodes.pop_front();
  }
}

void KModule::calculateDistance(KFunction *kf) {
  std::map<KFunction*, unsigned int> &dist = distance[kf];
  std::vector<std::pair<KFunction*, unsigned int>> &sort = sortedDistance[kf];
  std::deque<KFunction*> nodes;
  nodes.push_back(kf);
  dist[kf] = 0;
  sort.push_back({kf, 0});
  while(!nodes.empty()) {
    KFunction *currKF = nodes.front();
    for (auto &callBlock : currKF->kCallBlocks) {
      if (!callBlock->calledFunction || callBlock->calledFunction->isDeclaration()) continue;
      KFunction *callKF = functionMap[callBlock->calledFunction];
      if (dist.find(callKF) == dist.end()) {
        dist[callKF] = dist[currKF] + 1;
        sort.push_back({callKF, dist[currKF] + 1});
        nodes.push_back(callKF);
      }
    }
    nodes.pop_front();
  }
}

size_t KModule::functionHash(KFunction* kf) {
  if (!functionHashMap.count(kf)) {
    std::hash<KFunction> hasher;
    functionHashMap[kf] = hasher(*kf);
  }
  return functionHashMap[kf];
}

bool KModule::link(std::vector<std::unique_ptr<llvm::Module>> &modules,
                   const std::string &entryPoint) {
  auto numRemainingModules = modules.size();
  // Add the currently active module to the list of linkables
  modules.push_back(std::move(module));
  std::string error;
  module = std::unique_ptr<llvm::Module>(
      klee::linkModules(modules, entryPoint, error));
  if (!module)
    klee_error("Could not link KLEE files %s", error.c_str());

  targetData = std::unique_ptr<llvm::DataLayout>(new DataLayout(module.get()));

  // Check if we linked anything
  return modules.size() != numRemainingModules;
}

void KModule::instrument(const Interpreter::ModuleOptions &opts) {
  // Inject checks prior to optimization... we also perform the
  // invariant transformations that we will end up doing later so that
  // optimize is seeing what is as close as possible to the final
  // module.
  legacy::PassManager pm;
  pm.add(new RaiseAsmPass());

  // This pass will scalarize as much code as possible so that the Executor
  // does not need to handle operands of vector type for most instructions
  // other than InsertElementInst and ExtractElementInst.
  //
  // NOTE: Must come before division/overshift checks because those passes
  // don't know how to handle vector instructions.
  pm.add(createScalarizerPass());

  // This pass will replace atomic instructions with non-atomic operations
  pm.add(createLowerAtomicPass());
  if (opts.CheckDivZero) pm.add(new DivCheckPass());
  if (opts.CheckOvershift) pm.add(new OvershiftCheckPass());

  pm.add(new IntrinsicCleanerPass(*targetData));
  pm.run(*module);
}

void KModule::optimiseAndPrepare(
    const Interpreter::ModuleOptions &opts,
    llvm::ArrayRef<const char *> preservedFunctions) {
  // Preserve all functions containing klee-related function calls from being
  // optimised around
  if (!OptimiseKLEECall) {
    legacy::PassManager pm;
    pm.add(new OptNonePass());
    pm.run(*module);
  }

  if (opts.Optimize)
    Optimize(module.get(), preservedFunctions);

  // Add internal functions which are not used to check if instructions
  // have been already visited
  if (opts.CheckDivZero)
    addInternalFunction("klee_div_zero_check");
  if (opts.CheckOvershift)
    addInternalFunction("klee_overshift_check");

  // Needs to happen after linking (since ctors/dtors can be modified)
  // and optimization (since global optimization can rewrite lists).
  injectStaticConstructorsAndDestructors(module.get(), opts.EntryPoint);

  // Finally, run the passes that maintain invariants we expect during
  // interpretation. We run the intrinsic cleaner just in case we
  // linked in something with intrinsics but any external calls are
  // going to be unresolved. We really need to handle the intrinsics
  // directly I think?
  legacy::PassManager pm3;
  pm3.add(createCFGSimplificationPass());
  switch(SwitchType) {
  case eSwitchTypeInternal: break;
  case eSwitchTypeSimple: pm3.add(new LowerSwitchPass()); break;
  case eSwitchTypeLLVM:  pm3.add(createLowerSwitchPass()); break;
  default: klee_error("invalid --switch-type");
  }
  pm3.add(new IntrinsicCleanerPass(*targetData));
  pm3.add(createScalarizerPass());
  pm3.add(new PhiCleanerPass());
  pm3.add(new FunctionAliasPass());
  if (SplitCalls) {
    pm3.add(new CallSplitter());
  }
  if (SplitReturns) {
    pm3.add(new ReturnSplitter());
  }
  pm3.run(*module);
}

void KModule::manifest(InterpreterHandler *ih, bool forceSourceOutput) {
  if (OutputSource || forceSourceOutput) {
    std::unique_ptr<llvm::raw_fd_ostream> os(ih->openOutputFile("assembly.ll"));
    assert(os && !os->has_error() && "unable to open source output");
    *os << *module;
  }

  if (OutputModule) {
    std::unique_ptr<llvm::raw_fd_ostream> f(ih->openOutputFile("final.bc"));
#if LLVM_VERSION_CODE >= LLVM_VERSION(7, 0)
    WriteBitcodeToFile(*module, *f);
#else
    WriteBitcodeToFile(module.get(), *f);
#endif
  }

  /* Build shadow structures */

  infos = std::unique_ptr<InstructionInfoTable>(
      new InstructionInfoTable(*module.get()));

  std::vector<Function *> declarations;

  for (auto &Function : *module) {
    if (Function.isDeclaration()) {
      declarations.push_back(&Function);
      continue;
    }

    auto kf = std::unique_ptr<KFunction>(new KFunction(&Function, this));

    llvm::Function *function = &Function;
    for (auto &BasicBlock : *function) {
        unsigned numInstructions = kf->blockMap[&BasicBlock]->numInstructions;
        KBlock *kb = kf->blockMap[&BasicBlock];
        for (unsigned i=0; i<numInstructions; ++i) {
          KInstruction *ki = kb->instructions[i];
          ki->info = &infos->getInfo(*ki->inst);
        }
    }

    functionMap.insert(std::make_pair(&Function, kf.get()));
    functionNameMap.insert(std::make_pair(kf.get()->function->getName(), kf.get()));
    functions.push_back(std::move(kf));
  }

  /* Compute various interesting properties */

  for (auto &kf : functions) {
    if (functionEscapes(kf->function))
      escapingFunctions.insert(kf->function);
  }

  for (auto &declaration : declarations) {
    if (functionEscapes(declaration))
      escapingFunctions.insert(declaration);
  }

  for (auto &kfp : functions) {
    for (auto &kcb : kfp.get()->kCallBlocks) {
      callMap[kcb->calledFunction].insert(kfp.get()->function);
    }
  }

  if (DebugPrintEscapingFunctions && !escapingFunctions.empty()) {
    llvm::errs() << "KLEE: escaping functions: [";
    std::string delimiter = "";
    for (auto &Function : escapingFunctions) {
      llvm::errs() << delimiter << Function->getName();
      delimiter = ", ";
    }
    llvm::errs() << "]\n";
  }
}

void KModule::checkModule() {
  InstructionOperandTypeCheckPass *operandTypeCheckPass =
      new InstructionOperandTypeCheckPass();

  legacy::PassManager pm;
  if (!DontVerify)
    pm.add(createVerifierPass());
  pm.add(operandTypeCheckPass);
  pm.run(*module);

  // Enforce the operand type invariants that the Executor expects.  This
  // implicitly depends on the "Scalarizer" pass to be run in order to succeed
  // in the presence of vector instructions.
  if (!operandTypeCheckPass->checkPassed()) {
    klee_error("Unexpected instruction operand types detected");
  }
}

KBlock* KModule::getKBlock(llvm::BasicBlock *bb) {
  return functionMap[bb->getParent()]->blockMap[bb];
}

std::map<KFunction*, unsigned int> &KModule::getDistance(KFunction *kf) {
  if (distance.find(kf) == distance.end())
    calculateDistance(kf);
  return distance[kf];
}

std::vector<std::pair<KFunction*, unsigned int>> &KModule::getSortedDistance(KFunction *kf) {
  if (distance.find(kf) == distance.end())
    calculateDistance(kf);
  return sortedDistance[kf];
}

std::map<KFunction*, unsigned int> &KModule::getBackwardDistance(KFunction *kf) {
  if (backwardDistance.find(kf) == backwardDistance.end())
    calculateBackwardDistance(kf);
  return backwardDistance[kf];
}

std::vector<std::pair<KFunction*, unsigned int>> &KModule::getSortedBackwardDistance(KFunction *kf) {
  if (backwardDistance.find(kf) == backwardDistance.end())
    calculateBackwardDistance(kf);
  return sortedBackwardDistance[kf];
}

Function* llvm::getTargetFunction(Value *calledVal) {
  SmallPtrSet<const GlobalValue*, 3> Visited;

  Constant *c = dyn_cast<Constant>(calledVal);
  if (!c)
    return 0;

  while (true) {
    if (GlobalValue *gv = dyn_cast<GlobalValue>(c)) {
      if (!Visited.insert(gv).second)
        return 0;

      if (Function *f = dyn_cast<Function>(gv))
        return f;
      else if (GlobalAlias *ga = dyn_cast<GlobalAlias>(gv))
        c = ga->getAliasee();
      else
        return 0;
    } else if (llvm::ConstantExpr *ce = dyn_cast<llvm::ConstantExpr>(c)) {
      if (ce->getOpcode()==Instruction::BitCast)
        c = ce->getOperand(0);
      else
        return 0;
    } else
      return 0;
  }
}

KConstant* KModule::getKConstant(const Constant *c) {
  auto it = constantMap.find(c);
  if (it != constantMap.end())
    return it->second.get();
  return NULL;
}

unsigned KModule::getConstantID(Constant *c, KInstruction* ki) {
  if (KConstant *kc = getKConstant(c))
    return kc->id;  

  unsigned id = constants.size();
  auto kc = std::unique_ptr<KConstant>(new KConstant(c, id, ki));
  constantMap.insert(std::make_pair(c, std::move(kc)));
  constants.push_back(c);
  return id;
}

/***/

KConstant::KConstant(llvm::Constant* _ct, unsigned _id, KInstruction* _ki) {
  ct = _ct;
  id = _id;
  ki = _ki;
}

/***/

std::string KInstruction::toRegisterString() const {
  std::string repr = parent->parent->function->getName().str();
  std::string label;
  llvm::raw_string_ostream label_stream(label);
  label_stream << *inst;
  size_t index = label_stream.str().find('=');
  repr += label_stream.str().substr(2, index - 3);
  return repr;
}

std::string KInstruction::toString() const {
  std::string repr;
  llvm::raw_string_ostream repr_stream(repr);
  repr_stream << *inst;
  size_t k = 0;
  while(repr[k] == ' ') {
    ++k;
  }
  return repr.substr(k);
}

static int getOperandNum(Value *v,
                         std::map<Instruction*, unsigned> &registerMap,
                         KModule *km,
                         KInstruction *ki) {
  if (Instruction *inst = dyn_cast<Instruction>(v)) {
    return registerMap[inst];
  } else if (Argument *a = dyn_cast<Argument>(v)) {
    return a->getArgNo();
  } else if (isa<BasicBlock>(v) || isa<InlineAsm>(v) ||
             isa<MetadataAsValue>(v)) {
    return -1;
  } else {
    assert(isa<Constant>(v));
    Constant *c = cast<Constant>(v);
    return -(km->getConstantID(c, ki) + 2);
  }
}

void KBlock::handleKInstruction(
        std::map<Instruction*, unsigned> &registerMap,
        llvm::Instruction *inst,
        KModule *km, KInstruction *ki) {
  ki->parent = this;
  ki->inst = inst;
  ki->dest = registerMap[inst];
  if (isa<CallInst>(inst) || isa<InvokeInst>(inst)) {
#if LLVM_VERSION_CODE >= LLVM_VERSION(8, 0)
    const CallBase &cs = cast<CallBase>(*inst);
    Value *val = cs.getCalledOperand();
#else
    const CallSite cs(inst);
    Value *val = cs.getCalledValue();
#endif
    unsigned numArgs = cs.arg_size();
    ki->operands = new int[numArgs+1];
    ki->operands[0] = getOperandNum(val, registerMap, km, ki);
    for (unsigned j=0; j<numArgs; j++) {
      Value *v = cs.getArgOperand(j);
      ki->operands[j+1] = getOperandNum(v, registerMap, km, ki);
    }
  } else {
    unsigned numOperands = inst->getNumOperands();
    ki->operands = new int[numOperands];
    for (unsigned j=0; j<numOperands; j++) {
      Value *v = inst->getOperand(j);
      ki->operands[j] = getOperandNum(v, registerMap, km, ki);
    }
  }
}

KFunction::KFunction(llvm::Function *_function,
                     KModule *_km)
  : parent(_km),
    function(_function),
    numArgs(function->arg_size()),
    numInstructions(0),
    trackCoverage(true) {
  for (auto &BasicBlock : *function) {
    numInstructions += BasicBlock.size();
    numBlocks++;
  }
  instructions = new KInstruction*[numInstructions];
  std::map<Instruction*, unsigned> registerMap;
  // Assign unique instruction IDs to each basic block
  unsigned n = 0;
  // The first arg_size() registers are reserved for formals.
  unsigned rnum = numArgs;

  for (llvm::Function::iterator bbit = function->begin(),
         bbie = function->end(); bbit != bbie; ++bbit) {
    for (llvm::BasicBlock::iterator it = bbit->begin(), ie = bbit->end();
         it != ie; ++it)
      registerMap[&*it] = rnum++;
  }
  numRegisters = rnum;

  for (llvm::Function::iterator bbit = function->begin(),
         bbie = function->end(); bbit != bbie; ++bbit) {
    KBlock *kb;
    Instruction *firstit = &*(*bbit).begin();
    Instruction *lastit = &*(--(*bbit).end());
    if (isa<CallInst>(firstit) || isa<InvokeInst>(firstit)) {
#if LLVM_VERSION_CODE >= LLVM_VERSION(8, 0)
      const CallBase &cs = cast<CallBase>(*firstit);
      Value *fp = cs.getCalledOperand();
#else
      CallSite cs(firstit);
      Value *fp = cs.getCalledValue();
#endif
      Function *f = getTargetFunction(fp);
      KCallBlock *ckb = new KCallBlock(this, &*bbit, parent, registerMap, reg2inst, f, &instructions[n]);
      kCallBlocks.push_back(ckb);
      kb = ckb;
    } else if (isa<ReturnInst>(lastit)) {
      kb = new KReturnBlock(this, &*bbit, parent, registerMap, reg2inst, &instructions[n]);
    } else
      kb = new KBlock(this, &*bbit, parent, registerMap, reg2inst, &instructions[n]);
    for (unsigned i = 0; i < kb->numInstructions; i++, n++) {
      instructionMap[instructions[n]->inst] = instructions[n];
    }
    blockMap[&*bbit] = kb;
    blocks.push_back(std::unique_ptr<KBlock>(kb));
    labelMap[kb->getLabel()] = kb;
    if (isa<KReturnBlock>(kb) ||
        isa<UnreachableInst>(kb->instructions[kb->numInstructions - 1]->inst)) {
      finalKBlocks.push_back(kb);
      if (isa<KReturnBlock>(kb)) {
        returnKBlocks.push_back(kb);
      }
    }
  }

  entryKBlock = blockMap[&*function->begin()];
}

KFunction::~KFunction() {
for (unsigned i=0; i<numInstructions; ++i)
    delete instructions[i];
  delete[] instructions;
}

void KFunction::calculateDistance(KBlock *bb) {
  std::map<KBlock *, unsigned int> &dist = distance[bb];
  std::vector<std::pair<KBlock *, unsigned int>> &sort = sortedDistance[bb];
  std::deque<KBlock*> nodes;
  nodes.push_back(bb);
  dist[bb] = 0;
  sort.push_back({bb, 0});
  while(!nodes.empty()) {
    KBlock *currBB = nodes.front();
    for (auto const &succ : successors(currBB->basicBlock)) {
      if (dist.find(blockMap[succ]) == dist.end()) {
        dist[blockMap[succ]] = dist[currBB] + 1;
        sort.push_back({blockMap[succ], dist[currBB] + 1});
        nodes.push_back(blockMap[succ]);
      }
    }
    nodes.pop_front();
  }
}

void KFunction::calculateBackwardDistance(KBlock *bb) {
  std::map<KBlock *, unsigned int> &bdist = backwardDistance[bb];
  std::vector<std::pair<KBlock *, unsigned int>> &bsort = sortedBackwardDistance[bb];
  std::deque<KBlock*> nodes;
  nodes.push_back(bb);
  bdist[bb] = 0;
  bsort.push_back({bb, 0});
  while(!nodes.empty()) {
    KBlock *currBB = nodes.front();
    for (auto const &pred : predecessors(currBB->basicBlock)) {
      if (bdist.find(blockMap[pred]) == bdist.end()) {
        bdist[blockMap[pred]] = bdist[currBB] + 1;
        bsort.push_back({blockMap[pred], bdist[currBB] + 1});
        nodes.push_back(blockMap[pred]);
      }
    }
    nodes.pop_front();
  }
}

std::map<KBlock *, unsigned int>& KFunction::getDistance(KBlock *kb) {
  if (distance.find(kb) == distance.end())
    calculateDistance(kb);
  return distance[kb];
}

std::vector<std::pair<KBlock *, unsigned int>>& KFunction::getSortedDistance(KBlock *kb) {
  if (distance.find(kb) == distance.end())
    calculateDistance(kb);
  return sortedDistance[kb];
}

std::map<KBlock *, unsigned int>& KFunction::getBackwardDistance(KBlock *kb) {
  if (backwardDistance.find(kb) == backwardDistance.end())
    calculateBackwardDistance(kb);
  return backwardDistance[kb];
}

std::vector<std::pair<KBlock *, unsigned int>>& KFunction::getSortedBackwardDistance(KBlock *kb) {
  if (backwardDistance.find(kb) == backwardDistance.end())
    calculateBackwardDistance(kb);
  return sortedBackwardDistance[kb];
}

KBlock *KFunction::getNearestJoinBlock(KBlock *kb) {
  KFunction *kf = kb->parent;
  for (auto &kbd : kf->getSortedBackwardDistance(kb)) {
#if LLVM_VERSION_CODE >= LLVM_VERSION(9, 0)
    if (kbd.first->basicBlock->hasNPredecessorsOrMore(2) ||
        kbd.first->basicBlock->hasNPredecessors(0))
      return kbd.first;
#else
    if (kbd.first->basicBlock->hasNUsesOrMore(2) ||
        kbd.first->basicBlock->hasNUses(0))
      return kbd.first;
#endif
  }
  return nullptr;
}


KBlock *KFunction::getNearestJoinOrCallBlock(KBlock *kb) {
  KFunction *kf = kb->parent;
  for (auto &kbd : kf->getSortedBackwardDistance(kb)) {
#if LLVM_VERSION_CODE >= LLVM_VERSION(9, 0)
    if (kbd.first->basicBlock->hasNPredecessorsOrMore(2) ||
        kbd.first->basicBlock->hasNPredecessors(0) ||
        (kbd.first->getKBlockType() == KBlockType::Call &&
         dyn_cast<KCallBlock>(kbd.first)->internal() &&
         !dyn_cast<KCallBlock>(kbd.first)->intrinsic()))
      return kbd.first;
#else
    if (kbd.first->basicBlock->hasNUsesOrMore(2) ||
        kbd.first->basicBlock->hasNUses(0) ||
        (kbd.first->getKBlockType() == KBlockType::Call &&
         dyn_cast<KCallBlock>(kbd.first)->internal() &&
         !dyn_cast<KCallBlock>(kbd.first)->intrinsic()))
      return kbd.first;
#endif
  }
  return nullptr;
}

std::string KFunction::argToString(llvm::Argument *arg) {
  std::string repr = function->getName().str();
  std::string label;
  llvm::raw_string_ostream label_stream(label);
  label_stream << *arg;
  size_t index = label_stream.str().find('%');
  repr += label_stream.str().substr(index);
  return repr;
}

KBlock::KBlock(KFunction *_kfunction, llvm::BasicBlock *block, KModule *km,
               std::map<Instruction*, unsigned> &registerMap,
               std::map<unsigned, KInstruction*> &reg2inst,
               KInstruction **instructionsKF)
  : parent(_kfunction),
    basicBlock(block),
    numInstructions(0),
    trackCoverage(true) {
  numInstructions += block->size();
  instructions = instructionsKF;

  unsigned i = 0;
  for (llvm::BasicBlock::iterator it = block->begin(), ie = block->end();
     it != ie; ++it) {
    KInstruction *ki;

    switch(it->getOpcode()) {
    case Instruction::GetElementPtr:
    case Instruction::InsertValue:
    case Instruction::ExtractValue:
      ki = new KGEPInstruction(); break;
    default:
      ki = new KInstruction(); break;
    }

    Instruction *inst = &*it;
    handleKInstruction(registerMap, inst, km, ki);
    instructions[i++] = ki;
    reg2inst[registerMap[&*it]] = ki;
  }
}

std::string KBlock::getIRLocation() const {
  std::string repr = "KBlock ";
  std::string label;
  llvm::raw_string_ostream label_stream(label);
  basicBlock->printAsOperand(label_stream);
  repr += label_stream.str().substr(6);
  repr += " in function ";
  repr += parent->function->getName();
  return repr;
}

std::string KBlock::getLabel() const {
  std::string label;
  llvm::raw_string_ostream label_stream(label);
  basicBlock->printAsOperand(label_stream, false);
  return label_stream.str();
}

KCallBlock::KCallBlock(KFunction *_kfunction, llvm::BasicBlock *block, KModule *km,
                    std::map<Instruction*, unsigned> &registerMap, std::map<unsigned, KInstruction*> &reg2inst,
                    llvm::Function *_calledFunction, KInstruction **instructionsKF)
  : KBlock::KBlock(_kfunction, block, km, registerMap, reg2inst, instructionsKF),
    kcallInstruction(this->instructions[0]),
    calledFunction(_calledFunction) {}

KReturnBlock::KReturnBlock(KFunction *_kfunction, llvm::BasicBlock *block, KModule *km,
               std::map<Instruction*, unsigned> &registerMap,
               std::map<unsigned, KInstruction*> &reg2inst,
               KInstruction **instructionsKF)
  : KBlock::KBlock(_kfunction, block, km, registerMap, reg2inst, instructionsKF) {}
