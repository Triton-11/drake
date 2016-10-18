#include "gtest/gtest.h"

#include "drake/examples/RobinsExample/library/library_robin.h"
#include "drake/examples/RobinsExample/library/type_template_methods.h"

GTEST_TEST(testHelloworld, compareReturnValue) {
int initial_value = 12;
TestClass myInstance(initial_value);
int return_value = myInstance.get_value();

ASSERT_EQ(initial_value, return_value);
}

