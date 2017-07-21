#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/find_resource.h"
#include "drake/examples/Quadrotor/quadrotor_plant.h"
#include "drake/examples/Quadrotor/state_conversion.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_autodiff.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/linear_system.h"

// The following sequence of tests compares the behaviour of three kinds of
// plants : (i) A GenericQuadrotor created from the QuadrotorPlant, and
// (ii) RigidBodyPlantAutodiff created from parsing a corresponding model URDF
// with additional thrust inputs, an (iii) same as (ii) but with quaternion
// floatingbase instead of euler angles.
namespace drake {
namespace examples {
namespace quadrotor {
namespace {

const double equilibrium_check_tolerance = 1e3;
const double g = 9.81,  // Gravitational acceleration (m/s^2).
    m = 0.5;            // Mass of the robot (kg).
// The nominal hover position is at (0, 0, 1.0) in world coordinates.
const Eigen::Vector3d kNominalPosition{((Eigen::Vector3d() << 0.0, 0.0, 1.0).
    finished())};

// A Generic Quadrotor Plant with the plant created from QuadrotorPlant.
template <typename T>
class GenericQuadrotor: public systems::Diagram<T> {
 public:
  GenericQuadrotor(VectorX<T> input) {
    this->set_name("QuadrotorTest");
    systems::DiagramBuilder<double> builder;

    plant_ = builder.template AddSystem<QuadrotorPlant<double>>();
    plant_->set_name("plant");
    context_ = plant_->CreateDefaultContext();
    context_->FixInputPort(0, input);

    systems::ConstantVectorSource<T>* source =
        builder.template AddSystem<systems::ConstantVectorSource<T>>(input);
    source->set_name("input");

    builder.Connect(source->get_output_port(), plant_->get_input_port(0));

    builder.BuildInto(this);
  }

  void SetState(const VectorX<T>& x0, const VectorX<T>& u0) {
    plant_->set_state(context_.get(), x0);
  }

  std::vector<Eigen::MatrixXd> getABMatrices(const VectorX<T>& x0,
                                             const VectorX<T>& u0) {
    SetState(x0, u0);
    auto linear_system = systems::Linearize(*plant_, *context_,
                                            equilibrium_check_tolerance);

    std::vector<Eigen::MatrixXd> ABMatrices;
    ABMatrices.push_back(linear_system->A());
    ABMatrices.push_back(linear_system->B());

    return ABMatrices;
  }

 private:
  QuadrotorPlant<T> *plant_{};
  std::unique_ptr<systems::Context<double>> context_;
};

// A Quadrotor as a RigidBodyPlantAutodiff that is created from a model
// specified in a URDF file using Euler angles.
template <typename T>
class RigidBodyAutoDiffRPYQuadrotor: public systems::Diagram<T> {
 public:
  RigidBodyAutoDiffRPYQuadrotor() {
    this->set_name("QuadrotorAD");

    systems::DiagramBuilder<AutoDiffXd> builder;

    auto tree = std::make_unique<RigidBodyTree<double>>();

    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow("drake/examples/Quadrotor/quadrotor.urdf"),
        multibody::joints::kRollPitchYaw, tree.get());

    systems::RigidBodyPlantAutodiff<double>::SetupInputMatrixB(tree->B);

    auto treead = tree->ToAutoDiffXd();
    plant_ =
        builder.template AddSystem<systems::RigidBodyPlantAutodiff<AutoDiffXd>>(
            std::move(treead));
    plant_->set_name("plantAD");

    builder.BuildInto(this);
  }

  std::vector<Eigen::MatrixXd> getABMatrices(const VectorX<double>& x0,
                                             const VectorX<double>& u0) {
    std::vector<Eigen::MatrixXd> ABMatrices;
    ABMatrices = plant_->LinearizeAB(x0, u0);

    return ABMatrices;
  }

 private:
  systems::RigidBodyPlantAutodiff<AutoDiffXd> *plant_{};
};

GTEST_TEST(TestQuadrotorPlantLinearize, hover) {
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  x0.topRows(3) << kNominalPosition;

  // Nominal input corresponds to a hover.
  const Eigen::VectorXd u0 = Eigen::VectorXd::Constant(4, m * g / 4);

  auto ge_model_ = std::make_unique<GenericQuadrotor<double>>(u0);
  auto ad_model_ =
      std::make_unique<RigidBodyAutoDiffRPYQuadrotor<AutoDiffXd>>();

  std::vector<Eigen::MatrixXd> ge_AB = ge_model_->getABMatrices(x0, u0);
  std::vector<Eigen::MatrixXd> ad_AB = ad_model_->getABMatrices(x0, u0);

  double tol = 1e-10;
  EXPECT_TRUE(
      CompareMatrices(ge_AB[0], ad_AB[0], tol, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(ge_AB[1], ad_AB[1], tol, MatrixCompareType::absolute));
}
GTEST_TEST(TestQuadrotorPlantLinearize, zero) {
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  x0.topRows(3) << kNominalPosition;

  const Eigen::VectorXd u0 = Eigen::VectorXd::Constant(4, 0);

  auto ge_model_ = std::make_unique<GenericQuadrotor<double>>(u0);
  auto ad_model_ =
      std::make_unique<RigidBodyAutoDiffRPYQuadrotor<AutoDiffXd>>();

  std::vector<Eigen::MatrixXd> ge_AB = ge_model_->getABMatrices(x0, u0);
  std::vector<Eigen::MatrixXd> ad_AB = ad_model_->getABMatrices(x0, u0);

  double tol = 1e-10;
  EXPECT_TRUE(
      CompareMatrices(ge_AB[0], ad_AB[0], tol, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(ge_AB[1], ad_AB[1], tol, MatrixCompareType::absolute));
}

GTEST_TEST(TestQuadrotorPlantLinearize, random) {
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  x0 << 1, 2, 3, 0.4, 0.5, 0.6, 1, 2, 3, 4, 5, 6;

  Eigen::VectorXd u0 = Eigen::VectorXd::Constant(4, 0);
  u0 << 0.6, 0.9, 1.2, 1.0;

  auto ge_model_ = std::make_unique<GenericQuadrotor<double>>(u0);
  auto ad_model_ =
      std::make_unique<RigidBodyAutoDiffRPYQuadrotor<AutoDiffXd>>();

  std::vector<Eigen::MatrixXd> ge_AB = ge_model_->getABMatrices(x0, u0);
  std::vector<Eigen::MatrixXd> ad_AB = ad_model_->getABMatrices(x0, u0);

  double tol = 1e-10;
  EXPECT_TRUE(
      CompareMatrices(ge_AB[0], ad_AB[0], tol, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(ge_AB[1], ad_AB[1], tol, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
