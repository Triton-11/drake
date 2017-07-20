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

const double equilibrium_check_tolerance = 1;

// A Generic Quadrotor Plant Diagram with the plant created from
// QuadrotorPlant.
template<typename T>
class GenericQuadrotor {
 public:
  GenericQuadrotor() {

    systems::DiagramBuilder<double> builder;

    plant_ = builder.AddSystem<QuadrotorPlant<double>>();
    plant_->set_name("plant");
    context_ = plant_->CreateDefaultContext();
  }

  void SetState(const VectorX<T>& x0, const VectorX<T>& u0) {
    context_->FixInputPort(0, u0);
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


//  A Quadrotor as a RigidBodyPlant that is created from a model
// specified in a URDF file using a Euler angles.
template<typename T>
class RigidBodyAutoDiffRPYQuadrotor {
 public:
  RigidBodyAutoDiffRPYQuadrotor() {

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

  }

  std::vector<Eigen::MatrixXd> getABMatrices(const VectorX<T>& x0,
                                             const VectorX<T>& u0) {
    std::vector<Eigen::MatrixXd> ABMatrices;
    ABMatrices = plant_->LinearizeAB(x0, u0);

    return ABMatrices;
  }
 private:
  systems::RigidBodyPlantAutodiff<T> *plant_{};
};

class QuadrotorTest: public ::testing::Test {
 public:
  QuadrotorTest() {
    ge_model_ = std::make_unique<GenericQuadrotor<double>>();
    ad_model_ = std::make_unique<RigidBodyAutoDiffRPYQuadrotor<double>>();

  }

  void BehaviorTest(VectorX<double> x0, VectorX<double> u0) {
    std::vector<Eigen::MatrixXd> ge_AB = ge_model_->getABMatrices(x0, u0);
    std::vector<Eigen::MatrixXd> ad_AB = ad_model_->getABMatrices(x0, u0);

    double tol = 1e-10;
    EXPECT_TRUE(
        CompareMatrices(ge_AB[0], ad_AB[0], tol, MatrixCompareType::absolute));
    EXPECT_TRUE(
        CompareMatrices(ge_AB[1], ad_AB[1], tol, MatrixCompareType::absolute));
  }

  const double g_{9.81},  // Gravitational acceleration (m/s^2).
      m_{0.5};            // Mass of the robot (kg).


 protected:
  VectorX<double> x0_ = VectorX<double>::Zero(12);

  std::unique_ptr<GenericQuadrotor<double>> ge_model_;
  std::unique_ptr<RigidBodyAutoDiffRPYQuadrotor<double>> ad_model_;

  std::unique_ptr<systems::Context<double>> ge_context_;


};

TEST_F(QuadrotorTest, hover) {

  // The nominal hover position is at (0, 0, 1.0) in world coordinates.
  const Eigen::Vector3d kNominalPosition{((Eigen::Vector3d() << 0.0, 0.0, 1.0).
      finished())};

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  x0.topRows(3) = kNominalPosition;

// Nominal input corresponds to a hover.
  Eigen::VectorXd u0 = Eigen::VectorXd::Constant(
      4, m_ * g_ / 4);

  BehaviorTest(x0, u0);
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
