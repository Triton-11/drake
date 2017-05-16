#pragma once

#include <memory>

#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace quadrotor_with_link {

/// A custom `systems::Diagram` composed of a `systems::RigidBodyPlant`
/// and a `systems::DrakeVisualizer`. The diagram's output port zero is
/// connected to the `systems::DrakeVisualizer`'s input port zero. The
/// resulting diagram has the same input and output ports as the plant.
template <typename T>
class PlantAndVisualizerDiagram : public systems::Diagram<T> {
 public:
  /// Builds the PlantAndVisualizerDiagram.
  /// `rigid_body_tree` the tree to be used within the `RigidBodyPlant`
  /// `penetration_stiffness`, `penetration_dissipation`,
  /// `static_friction_coefficient`, `dynamic_friction_coefficient`, and
  /// `v_stiction_tolerance` define the contact model of the plant.
  /// `lcm` is a pointer to an externally created lcm object.
  PlantAndVisualizerDiagram(std::unique_ptr<RigidBodyTree<T>> rigid_body_tree,
                            lcm::DrakeLcmInterface* lcm);

  const systems::RigidBodyPlant<T>& plant() const {
    return *rigid_body_plant_;
  }
 private:
  systems::RigidBodyPlant<T>* rigid_body_plant_{nullptr};
  systems::DrakeVisualizer* drake_visualizer{nullptr};
};

/// A custom `Diagram` consisting of a `ConstantVectorSource` of 0 magnitude
/// connected to a `VisualizedPlant`'s input port zero. The resulting diagram
/// has no input ports and the same output ports as the `VisualizedPlant`.
template <typename T>
class PassiveVisualizedPlant : public systems::Diagram<T> {
 public:
  /// Builds the PassiveVisualizedPlant.
  /// \param visualized_plant a unique pointer to the `VisualizedPlant`.
  explicit PassiveVisualizedPlant(
      std::unique_ptr<PlantAndVisualizerDiagram<T>> visualized_plant);

 private:
  PlantAndVisualizerDiagram<T>* visualized_plant_{nullptr};
};

}  // namespace quadrotor_with_link
}  // namespace examples
}  // namespace drake
