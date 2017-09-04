#pragma once

#include <math.h>

#include <Eigen/Dense>

namespace lqr {

class QuadModel {
 public:
  QuadModel(const int n_states, const int n_inputs,
            const float m, const float j_xxyy, const float j_zz);

  QuadModel(const float m, const float j_xxyy, const float j_zz) :
      QuadModel(13, 4, m, j_xxyy, j_zz) {}

  QuadModel(const QuadModel& model) :
      A_(model.A_), B_(model.B_),
      m_(model.m_), j_xxyy_(model.j_xxyy_), j_zz_(model.j_zz_),
      n_states_(model.n_states_), n_inputs_(model.n_inputs_) {}

  virtual ~QuadModel() {};
  void Update(const Eigen::Ref<const Eigen::VectorXf>& q,
              const Eigen::Ref<const Eigen::VectorXf>& u);
  virtual void CalculateA(const Eigen::Ref<const Eigen::VectorXf>& q,
                          const Eigen::Ref<const Eigen::VectorXf>& u);
  virtual void CalculateB(const Eigen::Ref<const Eigen::VectorXf>& q,
                          const Eigen::Ref<const Eigen::VectorXf>& u);

  Eigen::MatrixXf get_A() { return A_; };
  Eigen::MatrixXf get_A(const Eigen::Ref<const Eigen::VectorXf>& q,
                        const Eigen::Ref<const Eigen::VectorXf>& u);
  Eigen::MatrixXf get_B() { return B_; };
  Eigen::MatrixXf get_B(const Eigen::Ref<const Eigen::VectorXf>& q,
                        const Eigen::Ref<const Eigen::VectorXf>& u);

  // Accessors
  int n_states() { return n_states_; };
  int n_inputs() { return n_inputs_; };
  Eigen::VectorXf u0() { return u0_; };

  Eigen::MatrixXf A_, B_;

 protected:
  const float m_{0.0};
  const float j_xxyy_{0.0};
  const float j_zz_{0.0};

  const int n_states_{-1};
  const int n_inputs_{-1};

  const Eigen::VectorXf u0_;

  Eigen::Matrix4f unit_quaternion_correction_;
};

} // namespace lqr
