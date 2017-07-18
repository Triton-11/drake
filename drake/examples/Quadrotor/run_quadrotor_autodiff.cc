/// @file
///
/// This demo sets up a controlled Quadrotor that uses a Linear Quadratic
/// Regulator to (locally) stabilize a nominal hover.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/Quadrotor/quadrotor_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_autodiff.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_int32(simulation_trials, 10, "Number of trials to simulate.");
DEFINE_double(simulation_real_time_rate, 1.0, "Real time rate");
DEFINE_double(trial_duration, 7.0, "Duration of execution of each trial");

namespace drake {
using systems::DiagramBuilder;
using systems::Simulator;
using systems::Context;
using systems::ContinuousState;
using systems::VectorBase;

namespace examples {
namespace quadrotor {
namespace {

template <typename T>
int do_main() {

  // A file to just test some stuff...

  lcm::DrakeLcm lcm;
  DiagramBuilder<T> builder;

  /*
  auto quad_tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
      multibody::joints::kRollPitchYaw, quad_tree.get());
  auto quad_tree_autodiff = quad_tree->ToAutoDiffXd();
  */



  /****** RigidBodyTree Clone/ToAutoDiffXd Testing ******/

  auto tree = std::make_unique<RigidBodyTree<T>>();
  auto treed = std::make_unique<RigidBodyTree<double>>();
  auto treed_clone = treed->Clone();

  //auto treead = std::make_unique<RigidBodyTree<AutoDiffXd>>();

  //int a = 2;
  std::cout << tree->get_num_actuators() << std::endl;
  //tree->RigidBodyTreeAutoDiff::PrintValue();

  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/Quadrotor/quadrotor.urdf"),
      multibody::joints::kRollPitchYaw, treed.get());

  auto treead = treed->ToAutoDiffXd();
  std::unique_ptr<RigidBodyTree<AutoDiffXd>> treead2 = treed->ToAutoDiffXd();

  //std::cout << "treead n of positions: " << treead->get_num_positions() << std::endl;
  //std::cout << "treead2 n of positions: " << treead2->get_num_positions() << std::endl;
  //std::unique_ptr<RigidBodyTree<double>> clone_1 = tree->Clone();

  std::cout << "finish" << std::endl;


  //return 0;

  /****** Manually assemble a RigidBodyTree ******/
  /*
  auto tree = std::make_unique<RigidBodyTree<T>>();
  auto base = std::make_unique<RigidBody<T>>();
  base->set_name("base");
  base->set_mass(0.5);

  Matrix6<double> I;
  I << 1, 0, 0, 0, 1, 0,
       0, 2, 0, 0, 0, 0,
       0, 0, 3, 0, 0, 0,
       0, 0, 0, 4, 0, 0,
       1, 0, 0, 0, 5, 0,
       0, 0, 0, 0, 0, 10;
  base->set_spatial_inertia(I);




  RigidBody<T>* base_ptr = tree->add_rigid_body(std::move(base));

  base_ptr->add_joint(&tree->world(), std::make_unique<QuaternionFloatingJoint>(
      "base", Eigen::Isometry3d::Identity()));

  tree->compile();
  std::cout << "n o bodies: " << tree->get_num_bodies() << std::endl;
  std::cout << "n o positions: " << tree->get_num_positions() << std::endl;
  // There are two bodies: the "world" and "free_body".
  std::cout << "n o velocities: " << tree->get_num_velocities() << std::endl;
  */


  /*
  auto plant =
      builder.template AddSystem<systems::RigidBodyPlantAutodiff<T>>(std::move(tree));

  std::cout << &plant << std::endl;
  */
  /*
  //auto plant =
      builder.template AddSystem<systems::RigidBodyPlant<T>>(std::move(tree));

  //(void)plant;

  VectorX<T> hover_input(plant->get_input_size());
  hover_input.setZero();

  // Publisher is not AutoDiff capable...
  //systems::DrakeVisualizer* publisher =
  //    builder.template AddSystem<systems::DrakeVisualizer>(
  //        plant->get_rigid_body_tree(), &lcm);

  //builder.Connect(plant->get_output_port, publisher->get_input_port(0));

  auto diagram = builder.Build();


  systems::Simulator<T> simulator(*diagram);

  //simulator.Initialize();
  //simulator.StepTo(10);
  */


  const double timestep = 0.1;

  systems::RigidBodyPlantAutodiff<T>::SetupInputMatrixB(tree->B);

  std::cout << "treead->B" << std::endl << treead->B << std::endl;

  auto continuous_plant =
      builder.template AddSystem<systems::RigidBodyPlantAutodiff<T>>(std::move(treead));


  //systems::RigidBodyPlantAutodiff<T> continuous_plant(std::move(tree));

  auto continuous_context = continuous_plant->AllocateContext();
  continuous_plant->SetDefaults(continuous_context.get());


  // Check that the time-stepping model has the same states as the continuous,
  // but as discrete state.
  if (continuous_context->has_only_continuous_state() == false) {
    std::cout << "error, no continuous state" << std::endl;
  };

  // Check that the dynamics of the time-stepping model match the
  // (backwards-)Euler approximation of the continuous time dynamics.
  //auto derivatives = continuous_plant->AllocateTimeDerivatives();
  //continuous_plant->CalcTimeDerivatives(*continuous_context, derivatives.get());


  const int num_inputs = (continuous_plant->get_num_input_ports() > 0)
                         ? continuous_plant->get_input_port(0).size()
                         : 0;

  const Eigen::VectorXd x0 =
      Eigen::VectorXd::Zero(continuous_plant->get_num_states());
  const Eigen::VectorXd u0 =
      Eigen::VectorXd::Ones(num_inputs);

  continuous_plant->LinearizeAB(x0, u0);

  const VectorX<T> x = continuous_context->get_continuous_state()->CopyToVector();

  const VectorX<T> q = continuous_context->get_continuous_state()
      ->get_generalized_position()
      .CopyToVector();
  const VectorX<T> v = continuous_context->get_continuous_state()
      ->get_generalized_velocity()
      .CopyToVector();

  const VectorX<T> vn =
      v;// + timestep * derivatives->get_generalized_velocity().CopyToVector();

  auto kinsol = continuous_plant->get_rigid_body_tree().doKinematics(q, v);
  const VectorX<T> qn =
      q +
          timestep *
              continuous_plant->get_rigid_body_tree().transformVelocityToQDot(kinsol,
                                                                             vn);
  VectorX<T> xn(qn.rows() + vn.rows());
  xn << qn, vn;


  return 0;



  /*
  auto tree = std::make_unique<RigidBodyTree<double>>();

  auto base = std::make_unique<RigidBody<double>>();
  base->set_name("base");
  base->set_mass(0.5);

  RigidBody<double>* base_ptr = tree->add_rigid_body(std::move(base));

  base_ptr->add_joint(&tree->world(), std::make_unique<QuaternionFloatingJoint>(
      "base", Eigen::Isometry3d::Identity()));

  tree->compile();

  auto plant =
      builder.template AddSystem<systems::RigidBodyPlantAutodiff<double>>(std::move(tree));
  */


  //std::cout << &plant << std::endl;


  // TODO: WIP: Wrapper stuff now, Linearize method
  /*
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
      multibody::joints::kRollPitchYaw, tree.get());

  // The nominal hover position is at (0, 0, 1.0) in world coordinates.
  const Eigen::Vector3d kNominalPosition{((Eigen::Vector3d() << 0.0, 0.0, 1.0).
      finished())};

  auto quadrotor = builder.AddSystem<QuadrotorPlant<double>>();
  quadrotor->set_name("quadrotor");
  auto controller = builder.AddSystem(StabilizingLQRController(
      quadrotor, kNominalPosition));
  controller->set_name("controller");
  auto visualizer =
      builder.AddSystem<drake::systems::DrakeVisualizer>(*tree, &lcm);
  visualizer->set_name("visualizer");

  builder.Connect(quadrotor->get_output_port(0), controller->get_input_port());
  builder.Connect(controller->get_output_port(), quadrotor->get_input_port(0));
  builder.Connect(quadrotor->get_output_port(0), visualizer->get_input_port(0));

  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
  VectorX<double> x0 = VectorX<double>::Zero(12);

  const VectorX<double> kNominalState{((Eigen::VectorXd(12) << kNominalPosition,
      Eigen::VectorXd::Zero(9)).finished())};

  srand(42);

  for (int i = 0; i < FLAGS_simulation_trials; i++) {
    auto diagram_context = diagram->CreateDefaultContext();
    x0 = VectorX<double>::Random(12);

    simulator.get_mutable_context()
        ->get_mutable_continuous_state_vector()
        ->SetFromVector(x0);

    simulator.Initialize();
    simulator.set_target_realtime_rate(FLAGS_simulation_real_time_rate);
    simulator.StepTo(FLAGS_trial_duration);

    // Goal state verification.
    const Context<double>& context = simulator.get_context();
    const ContinuousState<double>* state = context.get_continuous_state();
    const VectorX<double>& position_vector = state->CopyToVector();

    if (!is_approx_equal_abstol(
        position_vector, kNominalState, 1e-4)) {
      throw std::runtime_error("Target state is not achieved.");
    }

    simulator.reset_context(std::move(diagram_context));
  }
  */


  /****** Cholesky Testing ******/
  /*
  // Two methods to get the same result with a different initialization.
  Eigen::Matrix2f A, b;
  Eigen::LLT<Eigen::Matrix2f> llt;
  A << 2, -1, -1, 3;
  b << 1, 2, 3, 1;
  std::cout << "Here is the matrix A:\n" << A << std::endl;
  std::cout << "Here is the right hand side b:\n" << b << std::endl;
  std::cout << "Computing LLT decomposition..." << std::endl;
  llt.compute(A);
  std::cout << "The solution is:\n" << llt.solve(b) << std::endl;

  Eigen::LLT<Eigen::Matrix2f> lltOfA(A); // compute the Cholesky decomposition of A
  std::cout << "The solution is:\n" << lltOfA.solve(b) << std::endl;
  Eigen::Matrix2f L = lltOfA.matrixL(); // retrieve factor L  in the decomposition
// The previous two lines can also be written as "L = A.llt().matrixL()"
  std::cout << "The Cholesky factor L is" << std::endl << L << std::endl;
  */



  /*
  // Cholesky Testing
  MatrixX<T> A(3,3);
  VectorX<T> b(3);
  b << 1, 2, 3;
  A << 4,-1,2, -1,6,0, 2,0,5;
  std::cout << "The matrix A is" << std::endl << A << std::endl;
  Eigen::LLT<MatrixX<T>> lltOfA(A); // compute the Cholesky decomposition of A
  auto xx = lltOfA.solve(b);
  std::cout << xx.cols() << std::endl;
  MatrixX<T> L = lltOfA.matrixL(); // retrieve factor L  in the decomposition
// The previous two lines can also be written as "L = A.llt().matrixL()"
  std::cout << "The Cholesky factor L is" << std::endl << L << std::endl;
  std::cout << "To check this, let us compute L * L.transpose()" << std::endl;
  std::cout << L * L.transpose() << std::endl;
  std::cout << "This should equal the matrix A" << std::endl;
  std::cout << "Try the inverse as L.inverse()" << std::endl;
  //std::cout << L.inverse() << std::endl;//

  // Cholesky Testing
  MatrixX<double> Ad(3,3);
  VectorX<double> bd(3);
  bd << 1, 2, 3;
  Ad << 4,-1,2, -1,6,0, 2,0,5;
  std::cout << "The matrix A is" << std::endl << Ad << std::endl;
  Eigen::LLT<MatrixX<double>> lltOfAd(Ad); // compute the Cholesky decomposition of A
  auto xxd = lltOfA.solve(bd);
  std::cout << xxd.cols() << std::endl;
  std::cout << "xxd:" << std::endl;
  //std::cout << xxd << std::endl;
  MatrixX<double> Ld = lltOfAd.matrixL(); // retrieve factor L  in the decomposition
// The previous two lines can also be written as "L = A.llt().matrixL()"
  std::cout << "The Cholesky factor L is" << std::endl << Ld << std::endl;
  std::cout << "To check this, let us compute Ld * Ld.transpose()" << std::endl;
  std::cout << Ld * Ld.transpose() << std::endl;
  std::cout << "This should equal the matrix A" << std::endl;
  std::cout << "Try the inverse as Ld.inverse()" << std::endl;
  std::cout << Ld.inverse() << std::endl;//
  */


  return 0;
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  //return drake::examples::quadrotor::do_main<double>();
  return drake::examples::quadrotor::do_main<drake::AutoDiffXd>();
}
