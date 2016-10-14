#include "gtest/gtest.h"

#include "drake/examples/RobinsExample/library/library_robin.h"

GTEST_TEST(testHelloworld, doesReturnSomething) {
int initial_value = 1;
TestClass myT(1);
int v = myT.get_value();

ASSERT_EQ(initial_value, v);
}

