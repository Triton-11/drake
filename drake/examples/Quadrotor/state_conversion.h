#pragma once

#include <iostream>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace examples {
namespace quadrotor {
/**
 * Converts a Quadrotor state from Euler angles to quaternion state
 * representation.
 * @param StateRPY A 12 x 1 column vector [translation; rpy_angles;
 * linear_velocity; angular_velocity].
 * @return A 13 x 1 column vector [translation; quaternion; angular_velocity;
 * linear_velocity].
 */
//template <typename T>
VectorX<double> ConvertRPYStateToQuaternion(const VectorX<double> StateRPY) {
  DRAKE_DEMAND(StateRPY.size() == 12);
  VectorX<double> StateQ = VectorX<double>::Zero(13);

  Vector4<double> angles_q = math::rpy2quat(
      static_cast<Vector3<double>>(StateRPY.segment(3, 3)));

  StateQ << StateRPY.segment(0, 3),
      angles_q,
      StateRPY.segment(9, 3),
      StateRPY.segment(6, 3);
  return StateQ;
}
/**
 * Converts a Quadrotor state from quaternion representation to a Euler angles
 * state.
 * @param A 13 x 1 column vector [translation; quaternion; angular_velocity;
 * linear_velocity].
 * @return StateRPY A 12 x 1 column vector [translation; rpy_angles;
 * linear_velocity; angular_velocity].

 */
//template <typename T>
VectorX<double> ConvertQuaternionStateToRPY(const VectorX<double> StateQ) {
  DRAKE_DEMAND(StateQ.size() == 13);
  VectorX<double> StateRPY = VectorX<double>::Zero(12);

  Vector3<double> angles_RPY = math::quat2rpy(
      static_cast<Vector4<double>>(StateQ.segment(3, 4)));

  StateRPY << StateQ.segment(0, 3),
      angles_RPY,
      StateQ.segment(10, 3),
      StateQ.segment(7, 3);
  return StateRPY;
}

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
