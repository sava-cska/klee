#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys
import shutil
import tempfile
import time


class TestRunner(object):

    def __init__(self, source_dir, timeout, keep_output, output, builddir, llvm):
        self.source_directory = source_dir
        self.base_directory = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        self.llvm = llvm
        if builddir:
            self.build_directory = builddir
        else:
            self.build_directory = "build"

        self.klee_path = os.path.join(self.base_directory, self.build_directory, "bin", "klee")
        self.include_path = os.path.join(self.base_directory, "include")
        self.lib_path = os.path.join(self.base_directory, self.build_directory, 'lib')

        self.output_dir = os.path.join(self.base_directory, output)

        if os.path.exists(self.output_dir):
            print(f"Coverage output directory ({self.output_dir}) already exists. To continue, move or remove the directory.")
            sys.exit(1)

        os.mkdir(self.output_dir)

        self.keep_output = keep_output
        self.tmp = tempfile.mkdtemp()

        if self.keep_output:
            self.klee_output_dir = os.path.join(self.base_directory, self.keep_output)
            if os.path.exists(self.klee_output_dir):
                print(f"Klee output directory ({self.klee_output_dir}) already exists. To continue, move, or remove the directory.")
                sys.exit(1)
        else:
            self.klee_output_dir = os.path.join(self.tmp, "klee_output")

        os.mkdir(self.klee_output_dir)

        self.timeout = timeout


    def compile(self, source):
        print(f"Compiling {source}")
        path_to_clang = os.path.join(self.llvm,"bin","clang") if self.llvm else "clang"
        path_to_llvm_link = os.path.join(self.llvm,"bin","llvm-link") if self.llvm else "llvm-link"
        instance_dir = os.path.join(self.output_dir, os.path.splitext(source)[0])
        os.mkdir(instance_dir)
        shutil.copy(os.path.join(self.source_directory, source), instance_dir)
        source = os.path.join(instance_dir, source)

        compiler_options = ["-O0", "-Xclang", "-disable-O0-optnone"]
        compiled_file = os.path.join(instance_dir, os.path.basename(source) + '.bc')
        cmd = [path_to_clang, "-I", self.include_path, "-c", "-Wno-everything",  "-g", "-emit-llvm", "-o", compiled_file, source] + compiler_options
        subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        compiled_library =  os.path.join(instance_dir, "library.bc")
        include_path=os.path.join(self.include_path, "klee-test-comp.c")
        cmd = [path_to_clang, "-c", "-g", "-emit-llvm", "-o", compiled_library, include_path] + compiler_options
        subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        cmd = [path_to_llvm_link, "-o", compiled_file]
        cmd += [compiled_library, compiled_file]
        subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def run(self, source):
        print(f"Running KLEE on {source}")
        instance_dir = os.path.join(self.tmp,os.path.splitext(source)[0])
        compiled_file = os.path.join(instance_dir, os.path.basename(source) + '.bc')
        # Add different options for KLEE
        cmd = [self.klee_path]

        # Add common arguments
        cmd += ["--output-dir=" + os.path.join(self.klee_output_dir, os.path.splitext(source)[0])]
        cmd += ["--max-time=" + str(self.timeout) + "s"]
        cmd += ["--max-solver-time=" + str(self.timeout) + "s"]
        cmd += ["--libc=uclibc"]
        cmd += ["--posix-runtime"]
        cmd += [compiled_file]
        subprocess.check_call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def compile_coverage(self, source):
        print(f"Compiling {source} for coverage")
        os.environ['LD_LIBRARY_PATH'] = f"{self.lib_path}"
        instance_dir = os.path.join(self.tmp,os.path.splitext(source)[0])
        compiled_file = os.path.join(instance_dir, os.path.basename(source) + "o")
        source = os.path.join(instance_dir, source)
        cmd = ["clang", "-I", self.include_path, "-L", self.lib_path]
        cmd += ["-DEXTERNAL"]
        cmd += ['-lkleeRuntest', '-fprofile-arcs', '-ftest-coverage']
        cmd += ["-o", compiled_file, source]
        include_path=os.path.join(self.base_directory, "include/klee-test-comp.c")
        cmd += [include_path]
        subprocess.check_call(cmd, env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def run_coverage(self, source):
        print(f"Running {source} on generated tests")
        t_end = time.time() + 60
        instance_dir = os.path.join(self.tmp,os.path.splitext(source)[0])
        compiled_file = os.path.join(instance_dir, os.path.basename(source) + "o")
        for filename in os.listdir(os.path.join(self.klee_output_dir, os.path.splitext(source)[0])):
            if time.time() > t_end:
                print(f"Stopping running generated tests after 60 seconds, some tests have not run.")
                break
            if filename.endswith(".ktestjson"):
                os.environ["KTEST_FILE"] = os.path.join(self.klee_output_dir, os.path.splitext(source)[0] ,filename)
                try:
                    subprocess.run([compiled_file], env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
                except subprocess.TimeoutExpired:
                    pass

    def save_gcov(self):
        subprocess.check_call(["gcovr", "-r", self.tmp, "--html-details" ,"GCOV_coverage"], cwd=self.tmp, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        cur_dir = os.path.join(self.output_dir, "GCOV_results")
        os.mkdir(cur_dir)
        for filename in os.listdir(self.tmp):
            if filename.startswith("GCOV_coverage"):
                shutil.copy(os.path.join(self.tmp,filename), cur_dir)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", help='source directory')
    parser.add_argument("--t", default=30, help='timeout in seconds, default: 30')
    parser.add_argument("--klee-output", default=None, help='where to keep klee output, default: not to keep')
    parser.add_argument("--output", required=True, help="where to keep coverage output, required")
    parser.add_argument("--builddir", default=None, help="build directory, default: build")
    parser.add_argument("--llvm", default=None)
    parser.add_argument("--just-compile", choices=('True','False'), help="Just compile files")
    args = parser.parse_args()

    wrapper = TestRunner(args.source, args.t, args.klee_output, args.output, args.builddir, args.llvm)
    count = 1;
    num_sources = len([name for name in os.listdir(args.source)])
    print(f"{num_sources} total.")
    for filename in os.listdir(args.source):
        if args.just_compile == 'True':
            wrapper.compile(filename)
        else:
            print(f"Source {count}/{num_sources}")
            wrapper.compile(filename)
            wrapper.run(filename)
            wrapper.compile_coverage(filename)
            wrapper.run_coverage(filename)
            count += 1
            wrapper.save_gcov()
            shutil.rmtree(wrapper.tmp)

if __name__ == '__main__':
    main()
