#include <stdlib.h>
#include "klee/klee.h"

int dec(int n) {
  return --n;
}

int sum(int a, int b) {
  return a + b;
}

int fib(int n) {
  if (n == 0 || n == 1) return 1;
  return sum(fib(dec(n)), fib(dec(dec(n))));
}

int main() {
  int n = 0;
  klee_make_symbolic(&n, sizeof(n), "n");
  klee_assume(n > 0);
  fib(n);
  return 0;
}
