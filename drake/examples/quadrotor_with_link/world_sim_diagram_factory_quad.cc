#include "drake/examples/quadrotor_with_link/world_sim_diagram_factory_quad.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/primitives/constant_vector_source.h"

using std::allocate_shared;
using Eigen::MatrixXd;
using Eigen::aligned_allocator;

namespace drake {

using lcm::DrakeLcmInterface;
using systems::ConstantVectorSource;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::InputPortDescriptor;
using systems::RigidBodyPlant;
using systems::System;

namespace examples {
namespace quadrotor_with_link {

template <typename T>
PlantAndVisualizerDiagram<T>::PlantAndVisualizerDiagram(
    std::unique_ptr<RigidBodyTree<T>> rigid_body_tree,
    lcm::DrakeLcmInterface* lcm) {
  DiagramBuilder<T> builder;
  this->set_name("PlantAndVisualizerDiagram");

  rigid_body_plant_ =
      builder.template AddSystem<RigidBodyPlant<T>>(std::move(rigid_body_tree));
  rigid_body_plant_->set_name("RigidBodyPlant");

  DRAKE_DEMAND(rigid_body_plant_ != nullptr);
  //DRAKE_DEMAND(rigid_body_plant_->get_num_actuators() > 0);

  // Creates and adds a DrakeVisualizer publisher.
  auto viz_publisher_ = builder.template AddSystem<DrakeVisualizer>(
      rigid_body_plant_->get_rigid_body_tree(), lcm);
  viz_publisher_->set_name("DrakeVisualizer");

  // Connects the plant to the publisher for visualization.
  builder.Connect(rigid_body_plant_->state_output_port(),
                  viz_publisher_->get_input_port(0));

  // Exports all of the RigidBodyPlant's input and output ports.
  for (int i = 0; i < rigid_body_plant_->get_num_input_ports(); ++i) {
    std::cout << "exporting input port" << i << std::endl;
    builder.ExportInput(rigid_body_plant_->get_input_port(i));
  }

  for (int i = 0; i < rigid_body_plant_->get_num_output_ports(); ++i) {
    std::cout << "exporting output port" << i << std::endl;
    builder.ExportOutput(rigid_body_plant_->get_output_port(i));
  }

  drake::log()->debug("Plant and visualizer Diagram built...");

  builder.BuildInto(this);
}
template class PlantAndVisualizerDiagram<double>;

template <typename T>
PassiveVisualizedPlant<T>::PassiveVisualizedPlant(
    std::unique_ptr<PlantAndVisualizerDiagram<T>> visualized_plant) {
  this->set_name("PassiveVisualizedPlant");

  // Sets up a builder for the demo.
  DiagramBuilder<T> builder;
  visualized_plant_ = builder.template AddSystem<PlantAndVisualizerDiagram<T>>(
      std::move(visualized_plant));
  visualized_plant_->set_name("PlantAndVisualizer");

  // Fixes constant sources to all inputs.
  const systems::RigidBodyPlant<T>& plant = visualized_plant_->plant();

  for (int instance_id = RigidBodyTreeConstants::kFirstNonWorldModelInstanceId;
      instance_id < plant.get_num_model_instances(); ++instance_id) {
    if (plant.model_instance_has_actuators(instance_id)) {
      const int input_port_index =
          plant.model_instance_actuator_command_input_port(instance_id)
              .get_index();
      const InputPortDescriptor<T>& input_port =
          visualized_plant_->get_input_port(input_port_index);
      const int num_inputs = input_port.size();
      // Instantiates a constant source that outputs a vector of zeros.
      VectorX<double> constant_value(num_inputs);
      constant_value.setZero();

      // Cascades the constant source to the model instance within the plant and
      // visualizer diagram. This effectively results in the model instance
      // being uncontrolled, i.e., passive.
      systems::ConstantVectorSource<T>* constant_vector_source =
          builder.template AddSystem<systems::ConstantVectorSource<T>>(
              constant_value);
      constant_vector_source->set_name("ConstantVectorZeroSource");
      builder.Connect(constant_vector_source->get_output_port(), input_port);
    }
  }

  builder.BuildInto(this);
}
template class PassiveVisualizedPlant<double>;
}  // namespace quadrotor_with_link
}  // namespace examples
}  // namespace drake
