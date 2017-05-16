/// @file
///
/// This demo sets up an uncontrolled KUKA iiwa robot within a simulation
/// mounted upon a table and with some simple objects (2 cylinders and
/// 1 cuboid) placed in its vicinity. The uncontrolled iiwa arm collapses
/// under the influence of gravity and results in the objects being scattered
/// due to collisions with the robot arm and among themselves.

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/quadrotor_with_link/world_sim_diagram_factory_quad.h"
#include "drake/examples/quadrotor_with_link/world_sim_tree_builder_quad.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/shapes/geometry.h"


#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
"Number of seconds to simulate.");

namespace drake {

using multibody::joints::kRollPitchYaw;
using parsers::ModelInstanceIdTable;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;


namespace examples {
namespace quadrotor_with_link {

namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  auto quad_world = std::make_unique<WorldSimTreeBuilder<double>>();

  auto tree = std::make_unique<RigidBodyTree<double>>();
  ModelInstanceIdTable model_id_table = AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() + "/examples/quadrotor_with_link/quadrotor.urdf",
      kRollPitchYaw, tree.get());

  auto world_RB = tree->get_mutable_body(0);
  auto quad_base = tree->get_mutable_body(1);
  std::cout << world_RB->get_model_instance_id() << std::endl;
  std::cout << quad_base->get_model_instance_id() << std::endl;

  ModelInstanceIdTable model_id_table2 = AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() + "/examples/quadrotor_with_link/rigid_link_load.urdf",
      kRollPitchYaw, tree.get());

  //WIP Cant find base_link

  auto zerolink = tree->get_mutable_body(2);

  std::cout << tree->get_mutable_body(1)->get_model_name() << std::endl;
  std::cout << tree->get_mutable_body(1)->get_name() << std::endl;
  std::cout << tree->get_mutable_body(2)->get_model_name() << std::endl;
  std::cout << tree->get_mutable_body(2)->get_name() << std::endl;

  zerolink->add_joint(quad_base, std::make_unique<RevoluteJoint>(
      "rot_x", Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitX()));

/*
  auto link_rot = std::make_unique<RigidBody<double>>();
  link_rot->set_name("link_rot");
  link_rot->set_mass(0);
  auto link_rope = std::make_unique<RigidBody<double>>();
  link_rope->set_name("link_rope");
  link_rope->set_mass(0);

  const Eigen::Isometry3d I = Eigen::Isometry3d::Identity();

//  Eigen::Vector4d color_ = Eigen::Vector4d::Ones();

  DrakeShapes::VisualElement cylinder1(
      DrakeShapes::Cylinder(0.2, 0.8), I, Eigen::Vector4d(0.7, 0.7, 0.7, 1));

  const auto cylinder = DrakeShapes::Cylinder(0.4, 0.8);
  link_rope->AddVisualElement(cylinder1);

  auto load = std::make_unique<RigidBody<double>>();
  load->set_name("load");
  load->set_mass(0.084);

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
  link_position(2, 3) = -0.4;
  Eigen::Isometry3d load_position = Eigen::Isometry3d::Identity();
  load_position(2, 3) = -0.025;


  link_rot_p->add_joint(quad_base, std::make_unique<RevoluteJoint>(
      "rot_x", Eigen::Isometry3d::Identity(), Eigen::Vector3d::UnitX()));
  link_rope_p->add_joint(link_rot_p, std::make_unique<RevoluteJoint>(
      "rot_y", link_position, Eigen::Vector3d::UnitY()));
  load_p->add_joint(link_rope_p, std::make_unique<FixedJoint>(
      "load_fix", load_position));

*/
  //WIP fix some stuff here, add correct origin xyz for ropelink, load
  //    diagram still fails

//  ModelInstanceIdTable model_id_table2 = AddModelInstanceFromUrdfFileToWorld(
//      drake::GetDrakePath() + "/examples/quadrotor_with_link/slungload.urdf",
//      kRollPitchYaw, tree.get());

  lcm::DrakeLcm lcm;

  auto visualized_plant = std::make_unique<PlantAndVisualizerDiagram<double>>(
      std::move(tree), &lcm);

  auto demo_plant = std::make_unique<PassiveVisualizedPlant<double>>(
      std::move(visualized_plant));

  auto simulator = std::make_unique<systems::Simulator<double>>(*demo_plant);

  systems::Context<double>* context = simulator->get_mutable_context();
  demo_plant->SetDefaultState(*context, context->get_mutable_state());

  simulator->Initialize();
  simulator->set_target_realtime_rate(0.1);

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
