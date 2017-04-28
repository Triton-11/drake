/// @file
///
/// This demo sets up an uncontrolled KUKA iiwa robot within a simulation
/// mounted upon a table and with some simple objects (2 cylinders and
/// 1 cuboid) placed in its vicinity. The uncontrolled iiwa arm collapses
/// under the influence of gravity and results in the objects being scattered
/// due to collisions with the robot arm and among themselves.

#include <gflags/gflags.h>

#include "drake/examples/quadrotor_with_link/world_sim_diagram_factory_quad.h"
#include "drake/examples/quadrotor_with_link/world_sim_tree_builder_quad.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace quadrotor_with_link {

namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  auto quad_world = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.

  quad_world->StoreModel(
      "quad",
      "/examples/Quadrotor/quadrotor.urdf");

  quad_world->AddGround();

  const Eigen::Vector3d kRobotBase(0, 0, 1);

  quad_world->AddFixedModelInstance("quad", kRobotBase);

  lcm::DrakeLcm lcm;

  auto visualized_plant = std::make_unique<PlantAndVisualizerDiagram<double>>(
      quad_world->Build(), &lcm);

  auto demo_plant = std::make_unique<PassiveVisualizedPlant<double>>(
      std::move(visualized_plant));

  auto simulator = std::make_unique<systems::Simulator<double>>(*demo_plant);

  systems::Context<double>* context = simulator->get_mutable_context();
  demo_plant->SetDefaultState(*context, context->get_mutable_state());

  simulator->Initialize();
  simulator->set_target_realtime_rate(1.0);

  simulator->StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace quadrotor_with_link
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::quadrotor_with_link::DoMain();
}
