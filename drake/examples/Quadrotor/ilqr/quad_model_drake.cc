#include "drake/examples/Quadrotor/ilqr/quad_model_drake.h"

#include "drake/math/quaternion.h"

namespace lqr {

void QuadModelDrake::CalculateA(const Eigen::Ref<const Eigen::VectorXf>& q,
                                const Eigen::Ref<const Eigen::VectorXf>& u)
{
  // Transformation matrix P to change from one state space representation to
  // another.
  // x_hat = P * x
  // x_hat = [W_r_WB, q_WB, W_v_WB, B_omega_WB]  RPG
  // x     = [W_r_WB, q_WB, B_omega_WB, B_v_WB]  kQuaternion
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(13, 13);
  P.block(0, 0, 7, 7).setIdentity();
  P.block(10, 7, 3, 3).setIdentity();
  P.block(7, 10, 3, 3) << drake::math::quat2rotmat(q.segment<4>(3).cast<double>());
  Eigen::MatrixXd P_inv = P.inverse();

  Eigen::VectorXd q_double = P_inv * q.cast<double>();

  // Map the collective thrust and the torques acting on the body to the
  // individual rotor thrusts.
  const double kM = 0.0245;
  const double L = 0.175;

  double f1 = (-2*kM*u(2) + L*u(3) + u(0)*kM*L)/(4*kM*L);
  double f2 = ( 2*kM*u(1) - L*u(3) + u(0)*kM*L)/(4*kM*L);
  double f3 = ( 2*kM*u(2) + L*u(3) + u(0)*kM*L)/(4*kM*L);
  double f4 = (-2*kM*u(1) - L*u(3) + u(0)*kM*L)/(4*kM*L);

  Eigen::VectorXd u_double = Eigen::VectorXd::Zero(n_inputs());
  u_double << f1, f2, f3, f4;

  std::vector<Eigen::MatrixXd> ABMatrices;
  ABMatrices = plant_->LinearizeAB(q_double, u.cast<double>());

  Eigen::MatrixXf P_A_P_inv = (P * ABMatrices[0] * P_inv).cast<float>();
  Eigen::MatrixXf P_B = (P * ABMatrices[1]).cast<float>();
  Eigen::MatrixXf A_min = Eigen::MatrixXf::Zero(10, 10);
  Eigen::MatrixXf B_min = Eigen::MatrixXf::Zero(10, 4);

  A_min << P_A_P_inv.block(0, 0, 10, 10);

  B_min.block<4,3>(3,1) << P_A_P_inv.block<4,3>(3,10);
  B_min.block<3,1>(7,0) << P_B.block<3,1>(7,0);
  B_min.block<3,1>(7,0) *= m_;

  // experimental
  //A_min.block<3,4>(0,3).setZero();
  //experimental: set very small values zero to avoid overflow issues
  for (size_t i = 0, nRows = A_min.rows(), nCols = A_min.cols(); i < nCols; ++i)
    for (size_t j = 0; j < nRows; ++j)
    {
      if (fabs(A_min(i,j)) < 0.0000001)
        A_min(i,j) = 0.0;
    }

  // TODO(robinsch): use cast as float?

  // Testing communication
  /*A_min.setIdentity();
  A_min *= 1111;
  B_min.setIdentity();
  B_min *= 22;
  A_min.block<1,10>(0,0) << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  B_min.block<1,4>(0,0) << 10, 20, 30, 40;
  */
  A_ << A_min.cast<float>();
  B_ << B_min.cast<float>();
}

void QuadModelDrake::CalculateB(const Eigen::Ref<const Eigen::VectorXf>& q,
                                    const Eigen::Ref<const Eigen::VectorXf>& u)
{
  // Do nothing, since B is already generated in CalculateA.
  return;
}

} // namespace lqr
