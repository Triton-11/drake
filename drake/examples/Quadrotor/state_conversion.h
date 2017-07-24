#pragma once

#include <iostream>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/util/drakeGeometryUtil.h"

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

  std::cout << "X_WB" << std::endl << X_WB.matrix() << std::endl;
  std::cout << "V_WB" << std::endl << V_WB << std::endl;
  //EncodePose(X_WB, msg->pose)
  //EncodeTwist(V_WB, msg->twist);

  //EncodeValue(joint_name_to_q_index_, msg->joint_name, q,
  //&(msg->joint_position));
  //EncodeValue(joint_name_to_v_index_, msg->joint_name, v,
  //&(msg->joint_velocity));
}

VectorX<double> ConvertRpyToQuaternion(
    const Eigen::Ref<const VectorX<double>>& q,
    const Eigen::Ref<const VectorX<double>>& v) {

  DRAKE_DEMAND(q.size() == 6);
  DRAKE_DEMAND(v.size() == 6);


  // from rpy

  Isometry3<double> X_JB;
  Vector6<double> V_JB;

  const int position_start = 0;
  const int velocity_start = 0;


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


  VectorX<double> q_quat = VectorX<double>::Zero(7);
  VectorX<double> v_quat = VectorX<double>::Zero(6);

// to Quaternion

// Quaternion-parameterized floating joint.
// Translation.
  q_quat.segment<3>(position_start) = X_JB.translation();

// Orientation.
  auto quat = math::rotmat2quat(X_JB.linear());
  q_quat.segment<4>(position_start + 3) = quat;

// Transform V_WB to the floating base's body frame (V_WB_B).
// Translational velocity.
  v_quat.segment<3>(velocity_start + 3) =
      X_JB.linear().transpose() * V_JB.tail<3>();
// Rotational velocity.
  v_quat.segment<3>(velocity_start) =
      X_JB.linear().transpose() * V_JB.head<3>();

  VectorX<double> qvVector = VectorX<double>::Zero(13);
  qvVector << q_quat, v_quat;

  return qvVector;
}

VectorX<double> ConvertQuaternionToRpy(
    const Eigen::Ref<const VectorX<double>>& q,
    const Eigen::Ref<const VectorX<double>>& v) {

  DRAKE_DEMAND(q.size() == 7);
  DRAKE_DEMAND(v.size() == 6);


  // from quaternion

  Isometry3<double> X_JB;
  Vector6<double> V_JB;

  const int position_start = 0;
  const int velocity_start = 0;


  // Quaternion
  X_JB.translation() = q.segment<3>(position_start);
  X_JB.linear() = math::quat2rotmat(q.segment<4>(position_start + 3));
  X_JB.makeAffine();

  // v has velocity in body frame.
  V_JB.head<3>() = X_JB.linear() * v.segment<3>(velocity_start);
  V_JB.tail<3>() = X_JB.linear() * v.segment<3>(velocity_start + 3);

  VectorX<double> q_rpy = VectorX<double>::Zero(6);
  VectorX<double> v_rpy = VectorX<double>::Zero(6);

  // to RPY

  // RPY-parameterized floating joint.
  // Translation.
  q_rpy.segment<3>(position_start) = X_JB.translation();

  // Orientation.
  auto rpy = math::rotmat2rpy(X_JB.linear());
  q_rpy.segment<3>(position_start + 3) = rpy;

  // Translational velocity.
  auto translationdot = V_JB.tail<3>();
  v_rpy.segment<3>(velocity_start) = translationdot;

  // Rotational velocity.
  Eigen::Matrix<double, 3, 3> phi;
  typename math::Gradient<decltype(phi), Eigen::Dynamic>::type* dphi =
      nullptr;
  typename math::Gradient<decltype(phi), Eigen::Dynamic, 2>::type* ddphi =
      nullptr;
  angularvel2rpydotMatrix(rpy, phi, dphi, ddphi);
  auto angular_velocity_world = V_JB.head<3>();
  auto rpydot = (phi * angular_velocity_world).eval();
  v_rpy.segment<3>(velocity_start + 3) = rpydot;

  VectorX<double> qvVector = VectorX<double>::Zero(12);
  qvVector << q_rpy, v_rpy;

  return qvVector;
}

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
