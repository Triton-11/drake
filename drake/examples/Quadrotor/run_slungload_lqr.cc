/// @file
///
/// This demo sets up a controlled Quadrotor that uses a Linear Quadratic
/// Regulator to (locally) stabilize a nominal hover.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/Quadrotor/quadrotor_plant.h"
#include "drake/examples/Quadrotor/state_conversion.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_autodiff.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_int32(simulation_trials, 10, "Number of trials to simulate.");
DEFINE_double(simulation_real_time_rate, 0.5, "Real time rate");
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
class QuadRbpDiagram : public systems::Diagram<T> {
 public:
  QuadRbpDiagram() {
    this->set_name("QuadrotorRBP");

    auto tree = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow("drake/examples/Quadrotor/slungload.urdf"),
        multibody::joints::kRollPitchYaw, tree.get());

    systems::RigidBodyPlantAutodiff<T>::SetupInputMatrixB(tree->B);

    systems::DiagramBuilder<T> builder;

    auto plant = builder.template AddSystem<systems::RigidBodyPlantAutodiff<T>>(
        std::move(tree));
    plant->set_name("plant");

    builder.ExportInput(plant->get_input_port(0));
    builder.ExportOutput(plant->get_output_port(0));

    builder.BuildInto(this);
  }
};

std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
    const systems::RigidBodyPlantAutodiff<double>* plant,
    Eigen::Vector3d nominal_position) {
  auto quad_context_goal = plant->CreateDefaultContext();

  auto num_states = plant->get_num_states();
  auto num_positions = plant->get_num_positions();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(num_states);
  x0.topRows(3) = nominal_position;
  if (num_positions == 9 || num_positions == 7) {
    x0.segment(3, 1) << 1;
  }
  auto m = plant->get_rigid_body_tree().getMass();
  auto g = 9.81;
  // Nominal input corresponds to a hover.
  Eigen::VectorXd u0 = Eigen::VectorXd::Constant(
      4, m * g / 4);

  quad_context_goal->FixInputPort(0, u0);
  quad_context_goal->get_mutable_continuous_state_vector()->SetFromVector(x0);

  //plant->set_state(quad_context_goal.get(), x0);

  // Setup LQR cost matrices (penalize position error 10x more than velocity
  // error).
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(num_states, num_states);
  Q.topLeftCorner(num_positions, num_positions) =
      10 * Eigen::MatrixXd::Identity(num_positions, num_positions);

  Eigen::Matrix4d R = Eigen::Matrix4d::Identity();

  QuadRbpDiagram<double> diagram;

  return systems::LinearQuadraticRegulator(diagram,
                                           *quad_context_goal, Q, R);
}

int do_main() {
  lcm::DrakeLcm lcm;

  DiagramBuilder<double> builder;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/Quadrotor/slungload.urdf"),
      multibody::joints::kRollPitchYaw, tree.get());

  systems::RigidBodyPlantAutodiff<double>::SetupInputMatrixB(tree->B);

  // The nominal hover position is at (0, 0, 1.0) in world coordinates.
  const Eigen::Vector3d kNominalPosition{((Eigen::Vector3d() << 0.0, 0.0, 1.0).
      finished())};

  auto quadrotor = builder.AddSystem<systems::RigidBodyPlantAutodiff<double>>(
      std::move(tree));
  quadrotor->set_name("quadrotor");

  auto controller = builder.AddSystem(StabilizingLQRController(
      quadrotor, kNominalPosition));
  controller->set_name("controller");

  auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(
      quadrotor->get_rigid_body_tree(), &lcm);
  visualizer->set_name("visualizer");

  builder.Connect(quadrotor->get_output_port(0), controller->get_input_port());
  builder.Connect(controller->get_output_port(), quadrotor->get_input_port(0));
  builder.Connect(quadrotor->get_output_port(0), visualizer->get_input_port(0));

  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
  int num_states = quadrotor->get_num_states();
  VectorX<double> x0 = VectorX<double>::Zero(num_states);

  const VectorX<double> kNominalState{((Eigen::VectorXd(num_states)
      << kNominalPosition, Eigen::VectorXd::Zero(num_states - 3 )).finished())};
  if (quadrotor->get_num_positions() == 9 ||
      quadrotor->get_num_positions() == 7) {
    x0.segment(3, 1) << 1;
  }

  srand(42);

  for (int i = 0; i < FLAGS_simulation_trials; i++) {
    auto diagram_context = diagram->CreateDefaultContext();
    x0 = VectorX<double>::Random(num_states);
    // Some debugging and testing and proper quaternions
    if (quadrotor->get_num_positions() == 9) {
      x0 << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.08, 0.09, 1.0, 1.1, 1.2, 1.3,
          1.4, 1.5, 0.6, 0.7;
      VectorX<double> x0_quat =
          ConvertRpyToQuaternion(x0.segment(0, 6), x0.segment(8, 6));
      std::cout << "y0qu" << std::endl << x0_quat << std::endl;
      x0.segment(0, 7) << x0_quat.segment(0, 7);
      x0.segment(9, 6) << x0_quat.segment(7, 6);
      x0 << 0.1, 0.2, 0.3,
            1, 0, 0, 0,
            0.1, 0.1,
            0, 0, 0,
            0, 0, 0,
            0, 0;
      } else if (quadrotor->get_num_positions() == 7) {
      x0 << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.08, 0.09, 1.0, 1.1, 1.2, 1.3;
      VectorX<double> x0_quat =
          ConvertRpyToQuaternion(x0.segment(0, 6), x0.segment(6, 6));
      std::cout << "x0_quat" << std::endl << x0_quat << std::endl;
      x0.segment(0, 7) << x0_quat.segment(0, 7);
      x0.segment(7, 6) << x0_quat.segment(7, 6);
      x0 << 0.0, 0.0, 0.9,
          1, 0, 0, 0,
          0, 0, 0,
          0, 0, 0;
      } else if (quadrotor->get_num_positions() == 8) {
      x0 << 0.0, 0.5, 0.9,
          0, 0, 0,
          0.5, 0.5,
          0, 0, 0,
          0, 0, 0,
          0, 0;
    }
    std::cout << "x0" << std::endl << x0 << std::endl;

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
        position_vector, kNominalState, 1)) {
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
  return drake::examples::quadrotor::do_main();
}
