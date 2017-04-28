//
// Created by robin on 27.04.17.
//
#include "drake/examples/hello_folder/helloworld.h"

namespace hello_namespace {

Hello::Hello() {};

int Hello::getter() {
  return internal_variable_;
}

void Hello::setter(int v) {
  internal_variable_ = v;
  new_var_ = v + 1;
}

} // namespace hello
