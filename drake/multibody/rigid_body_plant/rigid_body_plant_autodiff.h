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

  RigidBodyPlantAutodiff<AutoDiffXd>* DoToAutoDiffXd() const override;

  std::vector<Eigen::MatrixXd> LinearizeAB(
      const Eigen::VectorXd& x0, const Eigen::VectorXd& u0);

  static void SetupInputMatrixB(Eigen::MatrixXd& B);

 protected:
  // RigidBodyPlant<T> overrides.
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

 private:
  //std::unique_ptr<const RigidBodyTree<T>> tree_;
  const RigidBodyTree<T>& tree_;

  // timestep == 0.0 implies continuous-time dynamics,
  // timestep > 0.0 implies a discrete-time dynamics approximation.
  const double timestep_{0.0};

  VectorX<T> EvaluateActuatorInputs(const Context<T>& context) const;

  VectorX<T> ThrustsToSpatialForce(const VectorX<T>& u_thrusts,
                                   const KinematicsCache<T>& kinsol) const;
};

}  // namespace systems
}  // namespace drake
