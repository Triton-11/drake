/// @file
///
/// This demo sets up a controlled Quadrotor that uses a Linear Quadratic
/// Regulator to (locally) stabilize a nominal hover.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/Quadrotor/quadrotor_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_int32(simulation_trials, 10, "Number of trials to simulate.");
DEFINE_double(simulation_real_time_rate, 1.0, "Real time rate");
DEFINE_double(trial_duration, 7.0, "Duration of execution of each trial");

namespace drake {
using systems::DiagramBuilder;
using systems::Simulator;
using systems::Context;
using systems::ContinuousState;
using systems::RigidBodyPlant;
using systems::VectorBase;

namespace examples {
namespace quadrotor {
namespace {

int do_main(int argc, char* argv[]) {
  lcm::DrakeLcm lcm;

  DiagramBuilder<double> builder;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
      multibody::joints::kRollPitchYaw, tree.get());

  /*
  auto quad_base = tree->get_mutable_body(1);

  auto link_rot = std::make_unique<RigidBody<double>>();
  link_rot->set_name("link_rot");
  link_rot->set_mass(0);
  auto link_rope = std::make_unique<RigidBody<double>>();
  link_rope->set_name("link_rope");
  link_rope->set_mass(0);

  const Eigen::Isometry3d I = Eigen::Isometry3d::Identity();

  DrakeShapes::VisualElement cylinder1(
      DrakeShapes::Cylinder(0.002, 0.8), I, Eigen::Vector4d(0.3, 0.7, 0.7, 1));
  DrakeShapes::VisualElement cylinder2(
      DrakeShapes::Cylinder(0.02, 0.008), I, Eigen::Vector4d(0.3, 0.3, 0.7, 1));

  link_rope->AddVisualElement(cylinder1);

  auto load = std::make_unique<RigidBody<double>>();
  load->set_name("load");
  load->set_mass(4.084);
  load->AddVisualElement(cylinder2);

  SquareTwistMatrix<double> spatial_inertia =
      SquareTwistMatrix<double>::Identity();

  // Spatial inertia of a sphere = 2/5 * m * r^2.
  spatial_inertia.block(0, 0, 3, 3) *= 0.4 * pow(0.025, 2) * load->get_mass();
  spatial_inertia.block(3, 3, 3, 3) *= load->get_mass();

  load->set_spatial_inertia(spatial_inertia);

  RigidBody<double>* link_rot_p = tree->add_rigid_body(std::move(link_rot));
  RigidBody<double>* link_rope_p = tree->add_rigid_body(std::move(link_rope));
  RigidBody<double>* load_p = tree->add_rigid_body(std::move(load));

  Eigen::Isometry3d link_position = Eigen::Isometry3d::Identity();
  link_position(2, 3) = 0;
  Eigen::Isometry3d load_position = Eigen::Isometry3d::Identity();
  load_position(2, 3) = 0.8;


  link_rot_p->add_joint(quad_base, std::make_unique<RevoluteJoint>(
      "rot_x", Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitX()));
  link_rope_p->add_joint(link_rot_p, std::make_unique<RevoluteJoint>(
      "rot_y", Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitY()));
//  load_p->add_joint(link_rope_p, std::make_unique<FixedJoint>(
//      "load_fix", load_position));


  load_p->add_joint(link_rope_p, std::make_unique<RevoluteJoint>(
      "load_rot", Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitY()));

  tree->compile();




  auto quadrotor =
      builder.template AddSystem<RigidBodyPlant<double>>(std::move(tree));
  quadrotor->set_name("quadrotor");


  auto publisher =
      builder.template AddSystem<systems::DrakeVisualizer>(
          quadrotor->get_rigid_body_tree(), &lcm);

  VectorX<double> hover_input(quadrotor->get_input_size());
  hover_input.setOnes();




  auto source = builder.AddSystem<systems::ConstantVectorSource>(hover_input);


  builder.Connect(source->get_output_port(), quadrotor->get_input_port(0));
  builder.Connect(quadrotor->get_output_port(0), publisher->get_input_port(0));



*/

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
  //VectorX<double> x0 = VectorX<double>::Zero(18);

  const VectorX<double> kNominalState{((Eigen::VectorXd(12) << kNominalPosition,
  Eigen::VectorXd::Zero(9)).finished())};

  srand(42);

  for (int i = 0; i < FLAGS_simulation_trials; i++) {
    auto diagram_context = diagram->CreateDefaultContext();
    x0 = VectorX<double>::Random(12);
    //x0 = VectorX<double>::Random(18);

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
  return 0;
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::quadrotor::do_main(argc, argv);
}
