#include <iostream>

#include "library/library_robin.h"

int main() {
  std::cout << "Hello" << std::endl;
  TestClass myT(1);
  int v = myT.get_value();
  std::cout << "The value is: " << v << std::endl;
  return 0;
}
