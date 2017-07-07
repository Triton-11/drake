#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/continuous_state.h"

namespace drake {
namespace systems {

/// This is a child class of RigidBodyPlant that publishes `xdot`, time
/// derivative of RBPlant's state vector `x`, on an LCM channel encoded as an
/// `lcmt_drake_signal` message.
///
/// @tparam T The scalar type. Currently, the only supported type is `double`.
/// @ingroup rigid_body_systems
template <typename T>
class RigidBodyPlantAutodiff : public RigidBodyPlant<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyPlantAutodiff)

  /// Instantiates a %RigidBodyPlantThatPublishesXdot.
  ///
  /// @param[in] tree The RigidBodyTree. This defines the multi-body dynamics of
  /// the world. This parameter must not be `nullptr`.
  ///
  /// @param[in] channel The name of the channel on which to publish `xdot`.
  ///
  /// @param[in] lcm  A non-null pointer to the LCM subsystem to publish on.
  /// The pointer must remain valid for the lifetime of this object.

  // Inheriting RBP constructor.
  //using RigidBodyPlant<T>::RigidBodyPlant;
  explicit RigidBodyPlantAutodiff(std::unique_ptr<const RigidBodyTree<T>> tree,
                                  double timestep = 0.0);

  ~RigidBodyPlantAutodiff() override;

  void LinearizeAB(const Eigen::VectorXd& x0, const Eigen::VectorXd& u0);

 protected:
  // RigidBodyPlant<T> overrides.
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

 private:
  //std::unique_ptr<const RigidBodyTree<T>> tree_;
  const RigidBodyTree<T>& tree_;

  // Quadrotor inputs
  int kInputDimension{4};

  // timestep == 0.0 implies continuous-time dynamics,
  // timestep > 0.0 implies a discrete-time dynamics approximation.
  const double timestep_{0.0};

  VectorX<T> EvaluateActuatorInputs(const Context<T>& context) const;

  // Publishes `xdot`, the derivative of `x`, which is this system's generalized
  // state vector. This vector contains the derivatives of the RigidBodyTree's
  // joint's positions and velocities. Thus, the units are velocities and
  // accelerations.
  void PrintValue(const int& value);
  //void DoPublish(const Context<T>& context) const override;

};

}  // namespace systems
}  // namespace drake
