

#include <gflags/gflags.h>
#include <iostream>
#include "drake/examples/hello_folder/helloworld.h"



namespace hello_namespace {

namespace {

int DoMain() {

  auto my_class_instance = Hello();
  std::cout << my_class_instance.getter() <<  std::endl;
  return 0;
}

} // namespace
} // hello_namespace


int main(int argc, char *argv[]) {
  hello_namespace::DoMain();
  return 0;
}
