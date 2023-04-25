//===-- Executor.h ----------------------------------------------*- C++ -*-===//
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

#include "ExecutionState.h"
#include "ExternalDispatcher.h"
#include "MemoryManager.h"
#include "SearcherUtil.h"
#include "SeedInfo.h"
#include "SpecialFunctionHandler.h"
#include "StatsTracker.h"
#include "Summary.h"
#include "UserSearcher.h"

#include "klee/ADT/DiscretePDF.h"
#include "klee/ADT/RNG.h"
#include "klee/Core/Interpreter.h"
#include "klee/Expr/ArrayCache.h"
#include "klee/Expr/ArrayExprOptimizer.h"
#include "klee/Expr/ArrayManager.h"
#include "klee/Module/Cell.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Solver/Solver.h"
#include "klee/System/Time.h"

#include "llvm/IR/Argument.h"

#include "llvm/ADT/Twine.h"
#include "llvm/Support/raw_ostream.h"

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

struct KTest;

namespace llvm {
class BasicBlock;
class BranchInst;
class CallInst;
class LandingPadInst;
class Constant;
class ConstantExpr;
class Function;
class GlobalValue;
class Instruction;
class LLVMContext;
class DataLayout;
class Twine;
class Value;

llvm::Function *getTargetFunction(llvm::Value *calledVal,
                                  klee::ExecutionState &state);
} // namespace llvm

namespace klee {
class Array;
struct Cell;
class ExecutionState;
class ExternalDispatcher;
class Expr;
class InstructionInfoTable;
struct KFunction;
struct KInstruction;
class KInstIterator;
class KModule;
class MemoryManager;
class MemoryObject;
class ObjectState;
class PForest;
class IBidirectionalSearcher;
struct ExecutionStateBinaryRank;
class SeedInfo;
class SpecialFunctionHandler;
struct StackFrame;
class StatsTracker;
class TimingSolver;
class TreeStreamWriter;
class MergeHandler;
class MergingSearcher;
template <class T> class ref;

/// \todo Add a context object to keep track of data only live
/// during an instruction step. Should contain addedStates,
/// removedStates, and haltExecution, among others.

class Executor : public Interpreter {
  friend class OwningSearcher;
  friend class WeightedRandomSearcher;
  friend class SpecialFunctionHandler;
  friend class StatsTracker;
  friend class MergeHandler;
  friend class Composer;
  friend class ComposeVisitor;
  friend std::unique_ptr<ForwardSearcher>
  klee::constructUserSearcher(Executor &Executor);

public:
  enum MemoryOperation { Read, Write };

  enum TerminateReason {
    Abort,
    Assert,
    BadVectorAccess,
    Exec,
    External,
    Free,
    Model,
    Overflow,
    Ptr,
    ReadOnly,
    ReportError,
    User,
    UncaughtException,
    UnexpectedException,
    Unhandled,
  };

  typedef std::pair<llvm::BasicBlock *, llvm::BasicBlock *> BasicBlockPair;
  typedef std::map<llvm::BasicBlock *,
                   std::set<ExecutionState *, ExecutionStateIDCompare>>
      ExecutedInterval;
  typedef std::map<llvm::BasicBlock *, std::unordered_set<llvm::BasicBlock *>>
      VisitedBlock;
  typedef std::map<llvm::BasicBlock *,
                   std::unordered_set<Transition, TransitionHash>>
      VisitedTransition;

  struct ExecutionBlockResult {
    VisitedBlock history;
    VisitedTransition transitionHistory;
  };

  typedef std::map<llvm::BasicBlock *, ExecutionBlockResult> ExecutionResult;

  typedef std::pair<ExecutionState *, ExecutionState *> StatePair;

private:
  /// The random number generator.
  RNG theRNG;

  ExecutionResult results;

  // The program state just before any instructions are excecuted.
  ExecutionState *emptyState;

  static const char *TerminateReasonNames[];

  std::unique_ptr<KModule> kmodule;

  InterpreterHandler *interpreterHandler;

  std::unique_ptr<IBidirectionalSearcher> searcher;

  std::unique_ptr<ExternalDispatcher> externalDispatcher;

  std::unique_ptr<TimingSolver> solver;

  std::unique_ptr<MemoryManager> memory;

  std::set<ExecutionState *, ExecutionStateIDCompare> states;
  std::set<ExecutionState *, ExecutionStateIDCompare> isolatedStates;
  std::set<ProofObligation *, ProofObligationIDCompare> pobs;

  std::unique_ptr<StatsTracker> statsTracker;

  TreeStreamWriter *pathWriter, *symPathWriter;

  std::unique_ptr<SpecialFunctionHandler> specialFunctionHandler;

  TimerGroup timers;
  std::unique_ptr<PForest> processForest;
  ExprHashMap<std::pair<ref<Expr>, unsigned>> gepExprBases;
  ExprHashMap<ref<Expr>> gepExprOffsets;

  /// Used to track states that have been added during the current
  /// instructions step.
  /// \invariant \ref addedStates is a subset of \ref states.
  /// \invariant \ref addedStates and \ref removedStates are disjoint.
  std::vector<ExecutionState *> addedStates;
  /// Used to track states that have been removed during the current
  /// instructions step.
  /// \invariant \ref removedStates is a subset of \ref states.
  /// \invariant \ref addedStates and \ref removedStates are disjoint.
  std::vector<ExecutionState *> removedStates;

  /// Used for validity-core initialization in the same manner
  /// as addedStates and removedStates are used.
  ref<TargetedConflict> targetedConflict;

  /// When non-empty the Executor is running in "seed" mode. The
  /// states in this map will be executed in an arbitrary order
  /// (outside the normal search interface) until they terminate. When
  /// the states reach a symbolic branch then either direction that
  /// satisfies one or more seeds will be added to this map. What
  /// happens with other states (that don't satisfy the seeds) depends
  /// on as-yet-to-be-determined flags.
  std::map<ExecutionState *, std::vector<SeedInfo>> seedMap;

  /// Map of globals to their representative memory object.
  std::map<const llvm::GlobalValue *, MemoryObject *> globalObjects;

  /// Map of globals to their bound address. This also includes
  /// globals that have no representative object (i.e. functions).
  std::map<const llvm::GlobalValue *, ref<ConstantExpr>> globalAddresses;

  /// The set of legal function addresses, used to validate function
  /// pointers. We use the actual Function* address as the function address.
  std::set<uint64_t> legalFunctions;

  /// When non-null the bindings that will be used for calls to
  /// klee_make_symbolic in order replay.
  const struct KTest *replayKTest;

  /// When non-null a list of branch decisions to be used for replay.
  const std::vector<bool> *replayPath;

  /// The index into the current \ref replayKTest or \ref replayPath
  /// object.
  unsigned replayPosition;

  /// When non-null a list of "seed" inputs which will be used to
  /// drive execution.
  const std::vector<struct KTest *> *usingSeeds;

  /// Disables forking, instead a random path is chosen. Enabled as
  /// needed to control memory usage. \see fork()
  bool atMemoryLimit;

  /// Disables forking, set by client. \see setInhibitForking()
  bool inhibitForking;

  /// Signals the Executor to halt execution at the next instruction
  /// step.
  bool haltExecution;

  /// Whether implied-value concretization is enabled. Currently
  /// false, it is buggy (it needs to validate its writes).
  bool ivcEnabled;

  /// used to make choice between transferToBasicBlock and composition
  bool prefereComposition = false;

  /// The maximum time to allow for a single core solver query.
  /// (e.g. for a single STP query)
  time::Span coreSolverTimeout;

  /// Maximum time to allow for a single instruction.
  time::Span maxInstructionTime;

  /// Assumes ownership of the created array objects
  ArrayManager arrayManager;

  /// File to print executed instructions to
  std::unique_ptr<llvm::raw_ostream> debugInstFile;

  // @brief Buffer used by logBuffer
  std::string debugBufferString;

  // @brief buffer to store logs before flushing to file
  llvm::raw_string_ostream debugLogBuffer;

  /// Optimizes expressions
  ExprOptimizer optimizer;

  /// Points to the merging searcher of the searcher chain,
  /// `nullptr` if merging is disabled
  MergingSearcher *mergingSearcher = nullptr;

  /// Typeids used during exception handling
  std::vector<ref<Expr>> eh_typeids;

  /// File to print summary
  std::unique_ptr<llvm::raw_fd_ostream> summaryFile;

  std::unique_ptr<Summary> summary;

  std::vector<Symbolic> *symbolics;

  std::unordered_set<Transition, TransitionHash> successTransitions;
  std::unordered_multiset<Transition, TransitionHash> failedTransitions;

public:
  ExecutionState *initialState;

  static void makeConflictCore(const ExecutionState &state,
                               const std::vector<ref<Expr>> &unsatCore,
                               ref<Expr> condition, KInstruction *location,
                               Conflict::core_ty &core);

private:
  /// Return the typeid corresponding to a certain `type_info`
  ref<ConstantExpr> getEhTypeidFor(ref<Expr> type_info);

  void executeInstruction(ExecutionState &state, KInstruction *ki);

  void seed(ExecutionState &initialState);
  void run(ExecutionState &initialState);

  // Given a concrete object in our [klee's] address space, add it to
  // objects checked code can reference.
  MemoryObject *addExternalObject(ExecutionState &state, void *addr,
                                  unsigned size, bool isReadOnly);

  void initializeGlobalAlias(const llvm::Constant *c);
  void initializeGlobalObject(ExecutionState &state, ObjectState *os,
                              const llvm::Constant *c, unsigned offset);
  void initializeGlobals(ExecutionState &state);
  void allocateGlobalObjects(ExecutionState &state);
  void initializeGlobalAliases();
  void initializeGlobalObjects(ExecutionState &state);

  void stepInstruction(ExecutionState &state);

  void updateResult(ref<ActionResult>);
  void removeState(ExecutionState *state);
  void removeIsolatedState(ExecutionState *state);

  void transferToBasicBlock(llvm::BasicBlock *dst, llvm::BasicBlock *src,
                            ExecutionState &state);

  void callExternalFunction(ExecutionState &state, KInstruction *target,
                            llvm::Function *function,
                            std::vector<ref<Expr>> &arguments);

  ObjectState *bindObjectInState(ExecutionState &state, const MemoryObject *mo,
                                 bool IsAlloca, const Array *array = 0);

  ObjectState *bindSymbolicInState(ExecutionState &state,
                                   const MemoryObject *mo, bool IsAlloca,
                                   const Array *array);

  /// Resolve a pointer to the memory objects it could point to the
  /// start of, forking execution when necessary and generating errors
  /// for pointers to invalid locations (either out of bounds or
  /// address inside the middle of objects).
  ///
  /// \param results[out] A list of ((MemoryObject,ObjectState),
  /// state) pairs for each object the given address can point to the
  /// beginning of.
  typedef std::vector<std::pair<
      std::pair<const MemoryObject *, const ObjectState *>, ExecutionState *>>
      ExactResolutionList;
  void resolveExact(ExecutionState &state, ref<Expr> p,
                    ExactResolutionList &results, const std::string &name,
                    KInstruction *target = nullptr);

  /// Allocate and bind a new object in a particular state. NOTE: This
  /// function may fork.
  ///
  /// \param isLocal Flag to indicate if the object should be
  /// automatically deallocated on function return (this also makes it
  /// illegal to free directly).
  ///
  /// \param target Value at which to bind the base address of the new
  /// object.
  ///
  /// \param reallocFrom If non-zero and the allocation succeeds,
  /// initialize the new object from the given one and unbind it when
  /// done (realloc semantics). The initialized bytes will be the
  /// minimum of the size of the old and new objects, with remaining
  /// bytes initialized as specified by zeroMemory.
  ///
  /// \param allocationAlignment If non-zero, the given alignment is
  /// used. Otherwise, the alignment is deduced via
  /// Executor::getAllocationAlignment
  void executeAlloc(ExecutionState &state, ref<Expr> size, bool isLocal,
                    KInstruction *target, bool zeroMemory = false,
                    const ObjectState *reallocFrom = 0,
                    size_t allocationAlignment = 0);

  /// Free the given address with checking for errors. If target is
  /// given it will be bound to 0 in the resulting states (this is a
  /// convenience for realloc). Note that this function can cause the
  /// state to fork and that \ref state cannot be safely accessed
  /// afterwards.
  void executeFree(ExecutionState &state, ref<Expr> address,
                   KInstruction *target = 0);

  /// Serialize a landingpad instruction so it can be handled by the
  /// libcxxabi-runtime
  MemoryObject *serializeLandingpad(ExecutionState &state,
                                    const llvm::LandingPadInst &lpi,
                                    bool &stateTerminated);

  /// Unwind the given state until it hits a landingpad. This is used
  /// for exception handling.
  void unwindToNextLandingpad(ExecutionState &state);

  void executeCall(ExecutionState &state, KInstruction *ki, llvm::Function *f,
                   std::vector<ref<Expr>> &arguments);

  // do address resolution / object binding / out of bounds checking
  // and perform the operation
  void executeMemoryOperation(ExecutionState &state, MemoryOperation operation,
                              ref<Expr> address,
                              ref<Expr> value /* def if write*/,
                              KInstruction *target /* def if read*/,
                              std::vector<ExecutionState *> *results = nullptr);

  ObjectPair lazyInitializeVariable(ExecutionState &state, ref<Expr> address,
                                    bool isLocal, const llvm::Value *allocSite,
                                    uint64_t size);

  ObjectPair transparentLazyInitializeVariable(ExecutionState &state,
                                               ref<Expr> address,
                                               const llvm::Value *allocSite,
                                               uint64_t size);

  ObjectPair lazyInitialize(ExecutionState &state, const MemoryObject *mo,
                            const Array *array);

  ObjectPair executeMakeSymbolic(ExecutionState &state, const MemoryObject *mo,
                                 const std::string &name, bool isAlloca,
                                 const Array *array = nullptr);

  /// Create a new state where each input condition has been added as
  /// a constraint and return the results. The input state is included
  /// as one of the results. Note that the output vector may included
  /// NULL pointers for states which were unable to be created.
  void branch(ExecutionState &state, const std::vector<ref<Expr>> &conditions,
              std::vector<ExecutionState *> &result);

  // Fork current and return states in which condition holds / does
  // not hold, respectively. One of the states is necessarily the
  // current state, and one of the states may be null.
  StatePair fork(ExecutionState &current, ref<Expr> condition, bool isInternal,
                 bool produceUnsatCore, std::vector<ref<Expr>> &conflict);
  StatePair fork(ExecutionState &current, ref<Expr> condition, bool isInternal);

  /// Add the given (boolean) condition as a constraint on state. This
  /// function is a wrapper around the state's addConstraint function
  /// which also manages propagation of implied values,
  /// validity checks, and seed patching.
  void addConstraint(ExecutionState &state, ref<Expr> condition);

  // Called on [for now] concrete reads, replaces constant with a symbolic
  // Used for testing.
  ref<Expr> replaceReadWithSymbolic(ExecutionState &state, ref<Expr> e);

  const Cell &eval(const KInstruction *ki, unsigned index,
                   ExecutionState &state, StackFrame &sf,
                   bool isSymbolic = true);

  ref<Expr> readArgument(ExecutionState &state, StackFrame &frame,
                         const KFunction *kf, unsigned index) {
    ref<Expr> arg = frame.locals[kf->getArgRegister(index)].value;
    if (!arg) {
      prepareSymbolicArg(state, frame, index);
    }
    return frame.locals[kf->getArgRegister(index)].value;
  }

  ref<Expr> readDest(ExecutionState &state, StackFrame &frame,
                     const KInstruction *target) {
    unsigned index = target->dest;
    ref<Expr> reg = frame.locals[index].value;
    if (!reg) {
      prepareSymbolicRegister(state, frame, index);
    }
    return frame.locals[index].value;
  }

  Cell &getArgumentCell(const StackFrame &frame, const KFunction *kf,
                        unsigned index) {
    return frame.locals[kf->getArgRegister(index)];
  }

  Cell &getDestCell(const StackFrame &frame, const KInstruction *target) {
    return frame.locals[target->dest];
  }

  const Cell &eval(const KInstruction *ki, unsigned index,
                   ExecutionState &state, bool isSymbolic = true);

  Cell &getArgumentCell(const ExecutionState &state, const KFunction *kf,
                        unsigned index) {
    return getArgumentCell(state.stack.back(), kf, index);
  }

  Cell &getDestCell(const ExecutionState &state, const KInstruction *target) {
    return getDestCell(state.stack.back(), target);
  }

  void bindLocal(const KInstruction *target, StackFrame &frame,
                 ref<Expr> value);

  void bindArgument(KFunction *kf, unsigned index, StackFrame &frame,
                    ref<Expr> value);

  void bindLocal(const KInstruction *target, ExecutionState &state,
                 ref<Expr> value);

  void bindArgument(KFunction *kf, unsigned index, ExecutionState &state,
                    ref<Expr> value);

  /// Evaluates an LLVM constant expression.  The optional argument ki
  /// is the instruction where this constant was encountered, or NULL
  /// if not applicable/unavailable.
  ref<klee::ConstantExpr> evalConstantExpr(const llvm::ConstantExpr *c,
                                           const KInstruction *ki = NULL);

  /// Evaluates an LLVM constant.  The optional argument ki is the
  /// instruction where this constant was encountered, or NULL if
  /// not applicable/unavailable.
  ref<klee::ConstantExpr> evalConstant(const llvm::Constant *c,
                                       const KInstruction *ki = NULL);

  /// Return a unique constant value for the given expression in the
  /// given state, if it has one (i.e. it provably only has a single
  /// value). Otherwise return the original expression.
  ref<Expr> toUnique(const ExecutionState &state, ref<Expr> &e);

  /// Return a constant value for the given expression, forcing it to
  /// be constant in the given state by adding a constraint if
  /// necessary. Note that this function breaks completeness and
  /// should generally be avoided.
  ///
  /// \param purpose An identify string to printed in case of concretization.
  ref<klee::ConstantExpr> toConstant(ExecutionState &state, ref<Expr> e,
                                     const char *purpose);

  /// Bind a constant value for e to the given target. NOTE: This
  /// function may fork state if the state has multiple seeds.
  void executeGetValue(ExecutionState &state, ref<Expr> e,
                       KInstruction *target);

  /// Get textual information regarding a memory address.
  std::string getAddressInfo(ExecutionState &state, ref<Expr> address) const;

  // Determines the \param lastInstruction of the \param state which is not KLEE
  // internal and returns its InstructionInfo
  const InstructionInfo &
  getLastNonKleeInternalInstruction(const ExecutionState &state,
                                    llvm::Instruction **lastInstruction);

  bool shouldExitOn(enum TerminateReason termReason);

  // remove state from queue and delete
  void terminateState(ExecutionState &state);
  // call exit handler and terminate state
  void terminateStateEarly(ExecutionState &state, const llvm::Twine &message);
  // call exit handler and terminate state
  void terminateStateOnExit(ExecutionState &state);
  // call error handler and terminate state
  void terminateStateOnTerminator(ExecutionState &state);
  // terminate state
  void terminateStateOnError(ExecutionState &state, const llvm::Twine &message,
                             enum TerminateReason termReason,
                             const char *suffix = NULL,
                             const llvm::Twine &longMessage = "");

  // call error handler and terminate state, for execution errors
  // (things that should not be possible, like illegal instruction or
  // unlowered instrinsic, or are unsupported, like inline assembly)
  void terminateStateOnExecError(ExecutionState &state,
                                 const llvm::Twine &message,
                                 const llvm::Twine &info = "") {
    terminateStateOnError(state, message, Exec, NULL, info);
  }

  /// bindModuleConstants - Initialize the module constant table.
  void bindModuleConstants();

  template <typename SqType, typename TypeIt>
  void computeOffsetsSeqTy(KGEPInstruction *kgepi,
                           ref<ConstantExpr> &constantOffset, uint64_t index,
                           const TypeIt it);

  template <typename TypeIt>
  void computeOffsets(KGEPInstruction *kgepi, TypeIt ib, TypeIt ie);

  /// bindInstructionConstants - Initialize any necessary per instruction
  /// constant values.
  void bindInstructionConstants(KInstruction *KI);

  void doImpliedValueConcretization(ExecutionState &state, ref<Expr> e,
                                    ref<ConstantExpr> value);

  /// check memory usage and terminate states when over threshold of -max-memory
  /// + 100MB \return true if below threshold, false otherwise (states were
  /// terminated)
  bool checkMemoryUsage();

  /// check if branching/forking is allowed
  bool branchingPermitted(const ExecutionState &state) const;

  void printDebugInstructions(ExecutionState &state);
  void doDumpStates();

  /// Only for debug purposes; enable via debugger or klee-control
  void dumpStates();
  void dumpPForest();

  void replayStateFromPob(ProofObligation *pob);
  int getBase(ref<Expr> expr, std::pair<Symbolic, ref<Expr>> &resolved);
  int resolveLazyInstantiation(
      ExecutionState &state,
      std::map<ref<Expr>, std::pair<Symbolic, ref<Expr>>> &resolved);
  void extractSourcedSymbolics(ExecutionState &state, std::vector<Symbolic> &);

public:
  Executor(llvm::LLVMContext &ctx, const InterpreterOptions &opts,
           InterpreterHandler *ie);
  virtual ~Executor() = default;

  const InterpreterHandler &getHandler() { return *interpreterHandler; }

  void setPathWriter(TreeStreamWriter *tsw) override { pathWriter = tsw; }

  void setSymbolicPathWriter(TreeStreamWriter *tsw) override {
    symPathWriter = tsw;
  }

  void setReplayKTest(const struct KTest *out) override {
    assert(!replayPath && "cannot replay both buffer and path");
    replayKTest = out;
    replayPosition = 0;
  }

  void setReplayPath(const std::vector<bool> *path) override {
    assert(!replayKTest && "cannot replay both buffer and path");
    replayPath = path;
    replayPosition = 0;
  }

  llvm::Module *
  setModule(std::vector<std::unique_ptr<llvm::Module>> &modules,
            const ModuleOptions &opts,
            const std::vector<llvm::Function *> &mainFunctions) override;

  void useSeeds(const std::vector<struct KTest *> *seeds) override {
    usingSeeds = seeds;
  }

  ExecutionState *formState(llvm::Function *f, int argc, char **argv,
                            char **envp);

  void clearGlobal();

  void addAllocaDisequality(ExecutionState &state, const llvm::Value *allocSite,
                            ref<Expr> address);

  void prepareSymbolicValue(ExecutionState &state, StackFrame &frame,
                            const KInstruction *targetW);
  void prepareSymbolicRegister(ExecutionState &state, StackFrame &frame,
                               unsigned index);
  void prepareSymbolicArg(ExecutionState &state, StackFrame &frame,
                          unsigned index);
  void prepareSymbolicArgs(ExecutionState &state, StackFrame &frame);

  ref<Expr> makeSymbolicValue(llvm::Value *value, ExecutionState &state,
                              uint64_t size, Expr::Width width,
                              const std::string &name);

  void runFunctionAsMain(llvm::Function *f, int argc, char **argv,
                         char **envp) override;

  /*** Runtime options ***/

  void setHaltExecution(bool value) override { haltExecution = value; }

  void setInhibitForking(bool value) override { inhibitForking = value; }

  void prepareForEarlyExit() override;

  /*** State accessor methods ***/

  unsigned getPathStreamID(const ExecutionState &state) override;

  unsigned getSymbolicPathStreamID(const ExecutionState &state) override;

  void
  getConstraintLog(const ExecutionState &state, std::string &res,
                   Interpreter::LogType logFormat = Interpreter::STP) override;

  int resolveLazyInstantiation(ExecutionState &state) override;

  void setInstantiationGraph(ExecutionState &state, TestCase &tc) override;

  void logState(ExecutionState &state, int id,
                std::unique_ptr<llvm::raw_fd_ostream> &f) override;

  bool getSymbolicSolution(const ExecutionState &state, TestCase &res) override;

  void getCoveredLines(
      const ExecutionState &state,
      std::map<const std::string *, std::set<unsigned>> &res) override;

  Expr::Width getWidthForLLVMType(llvm::Type *type) const;
  size_t getAllocationAlignment(const llvm::Value *allocSite) const;

  /// Returns the errno location in memory of the state
  int *getErrnoLocation(const ExecutionState &state) const;

  TimingSolver *getSolver();

  time::Span getSolverTimeout();

  KInstruction *getKInst(llvm::Instruction *ints);

  KBlock *getKBlock(llvm::BasicBlock &bb);

  const KFunction *getKFunction(const llvm::Function *f) const;

  ArrayManager *getArrayManager();

  MemoryManager *getMemoryManager();

  ExecutionManager *getExecutionManager();

  MergingSearcher *getMergingSearcher() const { return mergingSearcher; };
  void setMergingSearcher(MergingSearcher *ms) { mergingSearcher = ms; };
  const Array *makeArray(ExecutionState &state, const uint64_t size,
                         const std::string &name, bool isExternal,
                         ref<Expr> liSource);
  const Array *makeArray(ExecutionState &state, const uint64_t size,
                         const std::string &name, bool isExternal);
  const Array *makeArray(ExecutionState &state, const uint64_t size,
                         const std::string &name,
                         ref<Expr> liSource = ref<Expr>());
  void executeStep(ExecutionState &state);
  void silentRemove(ExecutionState &state);
  bool isGEPExpr(ref<Expr> expr);

public:
  void addCompletedResult(ExecutionState &state);
  void addErroneousResult(ExecutionState &state);
  void addHistoryResult(ExecutionState &state);

  void pauseState(ExecutionState &state);
  void pauseRedundantState(ExecutionState &state);
  void unpauseState(ExecutionState &state);

  ref<InitializeResult> initBranch(ref<InitializeAction> action);
  ref<ForwardResult> goForward(ref<ForwardAction> action);
  ref<BackwardResult> goBackward(ref<BackwardAction> action);

  ref<ActionResult> executeAction(ref<BidirectionalAction> action);

  KBlock *getStartLocation(const ExecutionState &state);
  KBlock *getLastExecutedLocation(const ExecutionState &state);
  KBlock *getCurrentLocation(const ExecutionState &state);

  void actionBeforeStateTerminating(ExecutionState &state,
                                    TerminateReason reason);

  void runMainWithTarget(llvm::Function *mainFn, llvm::BasicBlock *target,
                         int argc, char **argv, char **envp);

  KBlock *calculateTargetByTransitionHistory(ExecutionState &state);
  KBlock *calculateTargetByBlockHistory(ExecutionState &state);
  void initializeRoots(ExecutionState *initialState);

  void addInvariantToSummary(KBlock *location, ref<Expr> invariant) const;
};

} // namespace klee
