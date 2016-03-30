#include "drake/examples/Pendulum/Pendulum.h"  // to get some types
#include "drake/util/testUtil.h"

#include "drake/util/test/eigen_matrix_compare_mock.h"
#include "gtest/gtest.h"

using Drake::CombinedVector;
using Drake::size;
using Drake::CombinedVectorUtil;
using Drake::NullVector;
using Drake::InputOutputRelation;

// Liang: DEBUG! Trying this to see if the following msvc-32 error goes away:
// "error C2718: 'const T1': actual parameter with requested alignment of 16 won't be aligned"
// #define EIGEN_DONT_ALIGN

namespace drake {
namespace test {

// Tests the ability to set a PendulumState equal to a vector and vice versa.
TEST(VectorTest, ValueAssignment) {
  Eigen::Vector2d x;
  x << 0.2, 0.4;

  PendulumState<double> state;
  state.theta = 0.2;
  state.thetadot = .3;

  EXPECT_EQ(size(state), static_cast<size_t>(2));

  state = x;
  EXPECT_EQ(state.thetadot, 0.4);

  state.theta = 0.5;
  x = toEigen(state);
  EXPECT_EQ(x(0), 0.5);

  Eigen::VectorXd y = toEigen(state);
  const double tolerance = 1e-8;

  EXPECT_THAT(x, drake::test::EigenMatrixIsApproximatelyEqual(
                     y, tolerance, MatrixCompareType::absolute));
}

// // Tests the ability to set a CombinedVector's value
// TEST(VectorTest, CombinedVector) {
//   Eigen::Vector3d abc;
//   abc << 1, 2, 3;

//   CombinedVector<double, PendulumState, PendulumInput> test(abc);
//   test = 2 * abc;

//   EXPECT_EQ(test.first().theta, 2.0);
//   EXPECT_EQ(test.first().thetadot, 4.0);
//   EXPECT_EQ(test.second().tau, 6.0);
// }

// // Tests the ability to use a CombinedVectorUtil
// TEST(VectorTest, CombinedVectorUtil) {
//   Eigen::Vector3d abc;
//   abc << 1, 2, 3;

//   CombinedVectorUtil<PendulumState, PendulumInput>::type<double> test(abc);
//   test = 2 * abc;
//   EXPECT_EQ(test.first().theta, 2.0);
//   EXPECT_EQ(test.first().thetadot, 4.0);
//   EXPECT_EQ(test.second().tau, 6.0);
// }

// // Verify that combining a vector with an unused or empty vector returns the
// // original type
// TEST(VectorTest, CombineVectorCornerCases) {
//   CombinedVectorUtil<PendulumState, NullVector>::type<double> test1;
//   EXPECT_TRUE((is_same<PendulumState<double>, decltype(test1)>::value))
//       << "combined vector builder returned " +
//              static_cast<string>(typeid(test1).name());

//   CombinedVectorUtil<NullVector, PendulumState>::type<double> test2;
//   EXPECT_TRUE((is_same<PendulumState<double>, decltype(test2)>::value))
//       << "combined vector builder returned " +
//              static_cast<string>(typeid(test2).name());
// }

// // Tests the RowsAtCompileTime
// TEST(VectorTest, RowsAtCompileTime) {
//   EXPECT_TRUE((Eigen::Matrix<double, 2, 1>::RowsAtCompileTime == 2))
//       << "failed to evaluate RowsAtCompileTime";
// }

// // Tests the InputOutputRelation. Verify that linear is a polynomial.
// TEST(VectorTest, InputOutputRelationLinearIsPolynomial) {
//   EXPECT_TRUE((InputOutputRelation::isA(InputOutputRelation::Form::LINEAR,
//                                         InputOutputRelation::Form::POLYNOMIAL)))
//       << "linear is polynomial";
// }

// // Tests the InputOutputRelation. Verify that zero is arbitrary.
// TEST(VectorTest, InputOutputRelationZeroIsArbitrary) {
//   EXPECT_TRUE((InputOutputRelation::isA(InputOutputRelation::Form::ZERO,
//                                         InputOutputRelation::Form::ARBITRARY)))
//       << "zero is arbitrary";
// }

// // Verify that the least common ancestor of the I/O relations
// // AFFINE, LINEAR, AND POLYNOMIAL is polynomial.
// TEST(VectorTest, InputOutputRelationLeastCommonAncestor) {
//   EXPECT_TRUE((
//       InputOutputRelation::leastCommonAncestor(
//           {InputOutputRelation::Form::AFFINE, InputOutputRelation::Form::LINEAR,
//            InputOutputRelation::Form::POLYNOMIAL}) ==
//       InputOutputRelation::Form::POLYNOMIAL))
//       << "least common ancestor should be polynomial";
// }

// // Verify that compositions of I/O relations are as expected
// TEST(VectorTest, InputOutputRelationCompositionTests) {
//   InputOutputRelation g(InputOutputRelation::Form::LINEAR);
//   InputOutputRelation f(InputOutputRelation::Form::POLYNOMIAL);

//   EXPECT_TRUE(InputOutputRelation::composeWith(g, f).form ==
//               InputOutputRelation::Form::POLYNOMIAL)
//       << "should be poly";

//   EXPECT_TRUE(InputOutputRelation::composeWith(f, g).form ==
//               InputOutputRelation::Form::POLYNOMIAL)
//       << "should be poly";
// }

// // Verify that combinations of I/O relations are as expected
// TEST(VectorTest, InputOutputRelationCombinationTests) {
//   InputOutputRelation g(InputOutputRelation::Form::LINEAR);
//   InputOutputRelation f(InputOutputRelation::Form::POLYNOMIAL);

//   EXPECT_TRUE(InputOutputRelation::combine(g, f).form ==
//               InputOutputRelation::Form::POLYNOMIAL)
//       << "should be poly";

//   EXPECT_TRUE(InputOutputRelation::combine(f, g).form ==
//               InputOutputRelation::Form::POLYNOMIAL)
//       << "should be poly";
// }

}  // namespace test
}  // namespace drake
