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

// The following sequence of tests compares the behaviour of three kinds of
// plants : (i) A GenericQuadrotor created from the QuadrotorPlant, and
// (ii) RigidBodyPlantAutodiff created from parsing a corresponding model URDF
// with additional thrust inputs, an (iii) same as (ii) but with quaternion
// floatingbase instead of euler angles.
namespace drake {
namespace examples {
namespace quadrotor {
namespace {

GTEST_TEST(QuadrotorPlantTest, DirectFeedthrough) {
  QuadrotorPlant<double> quadrotor;
  EXPECT_FALSE(quadrotor.HasAnyDirectFeedthrough());
}

// The models are integrated and compared for the duration specified by the
// following constant.
const double kSimulationDuration = 0.1;

// A Generic Quadrotor Plant Diagram with the plant created from
// QuadrotorPlant.
template<typename T>
class GenericQuadrotor: public systems::Diagram<T> {
 public:
  GenericQuadrotor(VectorX<T> input) {
    this->set_name("QuadrotorTest");

    systems::DiagramBuilder<T> builder;

    plant_ = builder.template AddSystem<QuadrotorPlant<T>>();
    plant_->set_name("plant");

    DRAKE_DEMAND(plant_->get_input_size() == input.size());
    VectorX<T> test_input = input;

    systems::ConstantVectorSource<T>* source =
        builder.template AddSystem<systems::ConstantVectorSource<T>>(
            test_input);
    source->set_name("test_input");

    builder.Connect(source->get_output_port(), plant_->get_input_port(0));

    builder.BuildInto(this);
  }

  void SetState(systems::Context<T> *context, VectorX<T> x) const {
    systems::Context<T>& plant_context =
        this->GetMutableSubsystemContext(*plant_, context);
    plant_->set_state(&plant_context, x);
  }

 private:
  QuadrotorPlant<T> *plant_{};
};

// A Quadrotor as a RigidBodyPlantAutodiff that is created from a model
// specified in a URDF file using a Euler angles.
template<typename T>
class RigidBodyAutoDiffRPYQuadrotor: public systems::Diagram<T> {
 public:
  RigidBodyAutoDiffRPYQuadrotor(VectorX<T> input) {
    this->set_name("QuadrotorAD");

    auto tree = std::make_unique<RigidBodyTree<T>>();

    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        FindResourceOrThrow("drake/examples/Quadrotor/quadrotor.urdf"),
        multibody::joints::kRollPitchYaw, nullptr, tree.get());

    systems::RigidBodyPlantAutodiff<T>::SetupInputMatrixB(tree->B);

    systems::DiagramBuilder<T> builder;

    plant_ = builder.template AddSystem<systems::RigidBodyPlantAutodiff<T>>(
        std::move(tree));
    plant_->set_name("plantAD");

    const int num_inputs = (plant_->get_num_input_ports() > 0)
                           ? plant_->get_input_port(0).size()
                           : 0;

    DRAKE_DEMAND(num_inputs == input.size());
    VectorX<T> test_input = input;

    systems::ConstantVectorSource<T>* source =
        builder.template AddSystem<systems::ConstantVectorSource<T>>(
            test_input);
    source->set_name("test_input");

    builder.Connect(source->get_output_port(), plant_->get_input_port(0));

    builder.BuildInto(this);
  }

  void SetState(systems::Context<T> *context, VectorX<T> x) const {
    systems::Context<T>& plant_context =
        this->GetMutableSubsystemContext(*plant_, context);
    plant_->set_state_vector(&plant_context, x);
  }

 private:
  systems::RigidBodyPlant<T> *plant_{};
};
// A Quadrotor as a RigidBodyPlantAutodiff that is created from a model
// specified in a URDF file using quaternions
template<typename T>
class RigidBodyAutoDiffQuaternionQuadrotor: public systems::Diagram<T> {
 public:
  RigidBodyAutoDiffQuaternionQuadrotor(VectorX<T> input) {
    this->set_name("QuadrotorADQuaternion");

    auto tree = std::make_unique<RigidBodyTree<T>>();

    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        FindResourceOrThrow("drake/examples/Quadrotor/quadrotor.urdf"),
        multibody::joints::kQuaternion, nullptr, tree.get());

    systems::RigidBodyPlantAutodiff<T>::SetupInputMatrixB(tree->B);

    systems::DiagramBuilder<T> builder;

    plant_ = builder.template AddSystem<systems::RigidBodyPlantAutodiff<T>>(
        std::move(tree));
    plant_->set_name("plantADQuaternion");

    const int num_inputs = (plant_->get_num_input_ports() > 0)
                           ? plant_->get_input_port(0).size()
                           : 0;

    DRAKE_DEMAND(num_inputs == input.size());
    VectorX<T> test_input = input;

    systems::ConstantVectorSource<T>* source =
        builder.template AddSystem<systems::ConstantVectorSource<T>>(
            test_input);
    source->set_name("test_input");

    builder.Connect(source->get_output_port(), plant_->get_input_port(0));

    builder.BuildInto(this);
  }

  void SetState(systems::Context<T> *context, VectorX<T> x) const {
    systems::Context<T>& plant_context =
        this->GetMutableSubsystemContext(*plant_, context);
    VectorX<T> x_quaternion = ConvertRPYStateToQuaternion(x);
    plant_->set_state_vector(&plant_context, x_quaternion);
  }

 private:
  systems::RigidBodyPlant<T> *plant_{};
};

//  Combines test setup for both kinds of plants:
//  ge_model_:  GenericQuadrotor
class QuadrotorTest: public ::testing::Test {
 public:
  QuadrotorTest() {

    VectorX<double> test_input(4);
    test_input.setZero();
    test_input << 1.1, 0.8, 0.9, 1.2;

    ge_model_ = std::make_unique<GenericQuadrotor<double>>(test_input);
    ad_model_ = std::make_unique<RigidBodyAutoDiffRPYQuadrotor<double>>(
        test_input);
    qa_model_ = std::make_unique<RigidBodyAutoDiffQuaternionQuadrotor<double>>(
        test_input);

    ge_simulator_ = std::make_unique<systems::Simulator<double>>(*ge_model_);
    ad_simulator_ = std::make_unique<systems::Simulator<double>>(*ad_model_);
    qa_simulator_ = std::make_unique<systems::Simulator<double>>(*qa_model_);

    ge_derivatives_ = ge_model_->AllocateTimeDerivatives();
    ad_derivatives_ = ad_model_->AllocateTimeDerivatives();
    qa_derivatives_ = qa_model_->AllocateTimeDerivatives();
  }

  void SetUp() override {
    ge_context_ = ge_model_->CreateDefaultContext();
    ad_context_ = ad_model_->CreateDefaultContext();
    qa_context_ = qa_model_->CreateDefaultContext();

    ge_model_->SetState(ge_context_.get(), x0_);
    ad_model_->SetState(ad_context_.get(), x0_);
    qa_model_->SetState(qa_context_.get(), x0_);

    ge_simulator_->Initialize();
    ad_simulator_->Initialize();
    qa_simulator_->Initialize();
  }

  void SetState(const VectorX<double> x0) {
    ge_context_ = ge_model_->CreateDefaultContext();
    ad_context_ = ad_model_->CreateDefaultContext();
    qa_context_ = qa_model_->CreateDefaultContext();

    ge_model_->SetState(ge_context_.get(), x0);
    ad_model_->SetState(ad_context_.get(), x0);
    qa_model_->SetState(qa_context_.get(), x0);
  }

  void Simulate(const double t) {
    ge_simulator_->Initialize();
    ad_simulator_->Initialize();
    qa_simulator_->Initialize();

    ge_simulator_->StepTo(t);
    ad_simulator_->StepTo(t);
    qa_simulator_->StepTo(t);
  }

  VectorX<double> GetState(systems::Simulator<double> *simulator) {
    return simulator->get_context()
        .get_continuous_state_vector()
        .CopyToVector();
  }

  void PassiveBehaviorTest(VectorX<double> x0) {
    SetState(x0);
    Simulate(kSimulationDuration);
    VectorX<double> my_state = ge_simulator_->get_context()
        .get_continuous_state_vector()
        .CopyToVector();
    VectorX<double> ad_state = ad_simulator_->get_context()
        .get_continuous_state_vector()
        .CopyToVector();
    VectorX<double> qa_state_quaternion = qa_simulator_->get_context()
        .get_continuous_state_vector()
        .CopyToVector();
    VectorX<double> qa_state = ConvertQuaternionStateToRPY(qa_state_quaternion);
    double tol = 1e-10;
    EXPECT_TRUE(
        CompareMatrices(my_state, ad_state, tol, MatrixCompareType::absolute));
    EXPECT_TRUE(
        CompareMatrices(my_state, qa_state, tol, MatrixCompareType::absolute));
  }

 protected:
  VectorX<double> x0_ = VectorX<double>::Zero(12);

  std::unique_ptr<GenericQuadrotor<double>> ge_model_;
  std::unique_ptr<RigidBodyAutoDiffRPYQuadrotor<double>> ad_model_;
  std::unique_ptr<RigidBodyAutoDiffQuaternionQuadrotor<double>> qa_model_;

  std::unique_ptr<systems::Simulator<double>> ge_simulator_, ad_simulator_,
      qa_simulator_;
  std::unique_ptr<systems::Context<double>> ge_context_, ad_context_,
      qa_context_;
  std::unique_ptr<systems::ContinuousState<double>> ge_derivatives_,
      ad_derivatives_, qa_derivatives_;
};

//  Test comparing the computation of derivatives for a fixed state.
TEST_F(QuadrotorTest, derivatives) {
  VectorX<double> x0 = VectorX<double>::Ones(12);  // Set state to ones.
  SetState(x0);

  ge_model_->CalcTimeDerivatives(*ge_context_, ge_derivatives_.get());
  ad_model_->CalcTimeDerivatives(*ad_context_, ad_derivatives_.get());
  qa_model_->CalcTimeDerivatives(*qa_context_, qa_derivatives_.get());

  VectorX<double> my_derivative_vector = ge_derivatives_->CopyToVector();
  VectorX<double> ad_derivative_vector = ad_derivatives_->CopyToVector();

  // qa_derivatives conversion to rpy is missing in order to compare correctly.

  EXPECT_TRUE(CompareMatrices(my_derivative_vector, ad_derivative_vector,
                              1e-10 /* tolerance */,
                              MatrixCompareType::absolute));
}

// Test comparing the state for of each kind of plant under passive behaviour
// after a 1.0 second motion. Each plant is dropped from rest at the origin.
TEST_F(QuadrotorTest, drop_from_rest) {
  VectorX<double> x0 = VectorX<double>::Zero(12);
  PassiveBehaviorTest(x0);
}

// Test comparing the state for of each kind of plant under passive behaviour
// after a 1.0 second motion. Each plant is dropped from the origin with an
// initial translational velocity.
TEST_F(QuadrotorTest, drop_from_initial_velocity) {
  VectorX<double> x0 = VectorX<double>::Zero(12);
  x0.segment(6, 3) << 1, 1, 1;  // Some translational velocity.
  PassiveBehaviorTest(x0);
}

// Test comparing the state for of each kind of plant under passive behaviour
// after a 1.0 second motion. Each plant is dropped from the origin with an
// initial rotational velocity.
TEST_F(QuadrotorTest, drop_from_initial_rotation) {
  VectorX<double> x0 = VectorX<double>::Zero(12);
  x0.segment(9, 3) << 1, 1, 1;  // Some rotary velocity.
  PassiveBehaviorTest(x0);
}

// Test comparing the state for of each kind of plant under passive behaviour
// after a 1.0 second motion. Each plant is dropped from and arbitrary initial
// state.
TEST_F(QuadrotorTest, drop_from_arbitrary_state) {
  VectorX<double> x0 = VectorX<double>::Zero(12);
  x0 << 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6;  // Some initial state.
  PassiveBehaviorTest(x0);
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
