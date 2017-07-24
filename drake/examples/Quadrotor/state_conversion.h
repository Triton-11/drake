#pragma once

#include <iostream>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace examples {
namespace quadrotor {

void ConvertToPoseTwist(const Eigen::Ref<const VectorX<double>>& q,
                        const Eigen::Ref<const VectorX<double>>& v) {
  // Encodes the floating base.
  // J is the root_joint frame.
  Isometry3<double> X_JB;
  Vector6<double> V_JB;

  const int position_start = 0;
  const int velocity_start = 0;

  if (q.size() == 6) {
    // RPY
    Vector3<double> rpy = q.segment<3>(position_start + 3);
    Vector3<double> rpydot = v.segment<3>(velocity_start + 3);

    X_JB.translation() = q.segment<3>(position_start);
    X_JB.linear() = math::rpy2rotmat(rpy);
    X_JB.makeAffine();

    Matrix3<double> phi = Matrix3<double>::Zero();
    angularvel2rpydotMatrix(rpy, phi, static_cast<Matrix3<double> *>(nullptr),
                            static_cast<Matrix3<double> *>(nullptr));

    auto decomp = Eigen::ColPivHouseholderQR<Matrix3<double>>(phi);
    V_JB.head<3>() = decomp.solve(rpydot);
    V_JB.tail<3>() = v.segment<3>(velocity_start);
  } else if (q.size() == 7) {
    // Quaternion
    X_JB.translation() = q.segment<3>(position_start);
    X_JB.linear() = math::quat2rotmat(q.segment<4>(position_start + 3));
    X_JB.makeAffine();

    // v has velocity in body frame.
    V_JB.head<3>() = X_JB.linear() * v.segment<3>(velocity_start);
    V_JB.tail<3>() = X_JB.linear() * v.segment<3>(velocity_start + 3);
  } else {
    DRAKE_ABORT_MSG("Floating joint is neither a RPY or a Quaternion joint.");
  }
  //Isometry3<double> X_WJ = root_joint.get_transform_to_parent_body();
  Isometry3<double> X_WJ = Isometry3<double>::Identity();
  Isometry3<double> X_WB = X_WJ * X_JB;
  Vector6<double> V_WB;
  V_WB.head<3>() = X_WJ.linear() * V_JB.head<3>();
  V_WB.tail<3>() = X_WJ.linear() * V_JB.tail<3>();
  (void) X_WB;
}

/**
 * Converts a Quadrotor state from Euler angles to quaternion state
 * representation.
 * @param q_rpy A 6 x 1 column vector [translation; rpy_angles].
 * @param v_rpy A 6 x 1 column vector [linear_velocity; angular_velocity].
 * @return A 13 x 1 column vector [translation; quaternion; angular_velocity;
 * linear_velocity].
 */
VectorX<double> ConvertRpyToQuaternion(
    const Eigen::Ref<const VectorX<double>>& q_rpy,
    const Eigen::Ref<const VectorX<double>>& v_rpy) {
  DRAKE_DEMAND(q_rpy.size() == 6);
  DRAKE_DEMAND(v_rpy.size() == 6);

  // J is the root_joint frame.
  Isometry3<double> X_JB;
  Vector6<double> V_JB;

  Vector3<double> rpy = q_rpy.segment<3>(3);
  Vector3<double> rpydot = v_rpy.segment<3>(3);

  X_JB.translation() = q_rpy.segment<3>(0);
  X_JB.linear() = math::rpy2rotmat(rpy);
  X_JB.makeAffine();

  Matrix3<double> phi = Matrix3<double>::Zero();
  angularvel2rpydotMatrix(rpy, phi, static_cast<Matrix3<double> *>(nullptr),
                          static_cast<Matrix3<double> *>(nullptr));

  auto decomp = Eigen::ColPivHouseholderQR<Matrix3<double>>(phi);
  V_JB.head<3>() = decomp.solve(rpydot);
  V_JB.tail<3>() = v_rpy.segment<3>(0);

  VectorX<double> q_quat = VectorX<double>::Zero(7);
  VectorX<double> v_quat = VectorX<double>::Zero(6);

// Quaternion-parameterized floating joint.
// Translation.
  q_quat.segment<3>(0) = X_JB.translation();

// Orientation.
  auto quat = math::rotmat2quat(X_JB.linear());
  q_quat.segment<4>(0 + 3) = quat;

// Transform V_WB to the floating base's body frame (V_WB_B).
// Translational velocity.
  v_quat.segment<3>(0 + 3) =
      X_JB.linear().transpose() * V_JB.tail<3>();
// Rotational velocity.
  v_quat.segment<3>(0) =
      X_JB.linear().transpose() * V_JB.head<3>();

  VectorX<double> qvVector = VectorX<double>::Zero(13);
  qvVector << q_quat, v_quat;

  return qvVector;
}

/**
 * Converts a Quadrotor state from quaternion representation to a Euler angles
 * state.
 * @param q_quat A 7 x 1 column vector [translation; quaternion].
 * @param v_quat A 6 x 1 column vector [angular_velocity; linear_velocity].
 * @return qv_rpy A 12 x 1 column vector [translation; rpy_angles;
 * linear_velocity; angular_velocity].

 */
VectorX<double> ConvertQuaternionToRpy(
    const Eigen::Ref<const VectorX<double>>& q_quat,
    const Eigen::Ref<const VectorX<double>>& v_quat) {
  DRAKE_DEMAND(q_quat.size() == 7);
  DRAKE_DEMAND(v_quat.size() == 6);

  // J is the root_joint frame.
  Isometry3<double> X_JB;
  Vector6<double> V_JB;

  // Quaternion
  X_JB.translation() = q_quat.segment<3>(0);
  X_JB.linear() = math::quat2rotmat(q_quat.segment<4>(3));
  X_JB.makeAffine();

  // v has velocity in body frame.
  V_JB.head<3>() = X_JB.linear() * v_quat.segment<3>(0);
  V_JB.tail<3>() = X_JB.linear() * v_quat.segment<3>(3);

  VectorX<double> q_rpy = VectorX<double>::Zero(6);
  VectorX<double> v_rpy = VectorX<double>::Zero(6);

  // RPY-parameterized floating joint.
  // Translation.
  q_rpy.segment<3>(0) = X_JB.translation();

  // Orientation.
  auto rpy = math::rotmat2rpy(X_JB.linear());
  q_rpy.segment<3>(3) = rpy;

  // Translational velocity.
  auto translationdot = V_JB.tail<3>();
  v_rpy.segment<3>(0) = translationdot;

  // Rotational velocity.
  Eigen::Matrix<double, 3, 3> phi;
  typename math::Gradient<decltype(phi), Eigen::Dynamic>::type* dphi =
      nullptr;
  typename math::Gradient<decltype(phi), Eigen::Dynamic, 2>::type* ddphi =
      nullptr;
  angularvel2rpydotMatrix(rpy, phi, dphi, ddphi);
  auto angular_velocity_world = V_JB.head<3>();
  auto rpydot = (phi * angular_velocity_world).eval();
  v_rpy.segment<3>(3) = rpydot;

  VectorX<double> qv_rpy = VectorX<double>::Zero(12);
  qv_rpy << q_rpy, v_rpy;

  return qv_rpy;
}

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
