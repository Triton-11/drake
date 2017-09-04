#include "drake/examples/Quadrotor/ilqr/quad_model.h"

namespace lqr {

QuadModel::QuadModel(const int n_states, const int n_inputs,
                     const float m, const float j_xxyy, const float j_zz):
    m_(m),
    j_xxyy_(j_xxyy), j_zz_(j_zz),
    n_states_(n_states), n_inputs_(n_inputs),
    u0_((Eigen::Matrix<float,4, 1>() << 9.806*m_, 0, 0, 0).finished())
{
  A_ = Eigen::MatrixXf::Zero(n_states_, n_states_);
  A_.block<3,3>(0,7) = Eigen::Matrix3f::Identity();
  B_ = Eigen::MatrixXf::Zero(n_states_, n_inputs_);
}


void QuadModel::Update(const Eigen::Ref<const Eigen::VectorXf>& q,
                       const Eigen::Ref<const Eigen::VectorXf>& u)
{
  CalculateA(q, u);
  CalculateB(q, u);
}

void QuadModel::CalculateA(const Eigen::Ref<const Eigen::VectorXf>& q,
                           const Eigen::Ref<const Eigen::VectorXf>& u)
{
  // Not available in drake, look at the rpg_ilqr.
  return;
}

void QuadModel::CalculateB(const Eigen::Ref<const Eigen::VectorXf>& q,
                           const Eigen::Ref<const Eigen::VectorXf>& u)
{
  // Not available in drake, look at the rpg_ilqr.
  return;
}

Eigen::MatrixXf QuadModel::get_A(const Eigen::Ref<const Eigen::VectorXf>& q,
                                 const Eigen::Ref<const Eigen::VectorXf>& u)
{
  Update(q, u);
  return A_;
}

Eigen::MatrixXf QuadModel::get_B(const Eigen::Ref<const Eigen::VectorXf>& q,
                                 const Eigen::Ref<const Eigen::VectorXf>& u)
{
  Update(q, u);
  return B_;
}

} // namespace lqr
