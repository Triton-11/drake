//#include <drake/examples/Quadrotor/ilqr/controller.h>

#include <Eigen/Dense>
#include <iostream>
#include <signal.h>

// ilqr includes

//#include "/opt/ros/indigo/include/ros/ros.h"
//#include "/opt/ros/indigo/include/ros/time.h"
//#include "/opt/ros/indigo/include/ros/node_handle.h"
#include "drake/examples/Quadrotor/ilqr/quad_model_bodyrates.h"
#include "drake/examples/Quadrotor/ilqr/quad_model_drake.h"
//#include "drake/examples/Quadrotor/ilqr/tilqr.h"
#include "drake/examples/Quadrotor/ilqr/ilqr.h"
#include "drake/examples/Quadrotor/ilqr/trajectory.h"

// drake includes
#include "drake/examples/Quadrotor/ilqr/quadrotor_lcm_msg_handler.h"
//#include "drake/lcmt_quadrotor_state.hpp"
//#include "drake/lcmt_quadrotor_vector.hpp"
#include "drake/examples/Quadrotor/state_conversion.h"
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_autodiff.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/math/quaternion.h"
#include "drake/common/eigen_types.h"
#include "tilqr.h"
//#include "ros/drake_ros_common/include/drake/ros/parameter_server.h"
//#include "ros/drake_ros_common/include/drake/ros"

namespace drake {
namespace systems {
namespace {

int DoMain() {

  drake::lcm::DrakeLcm lcm;

  std::shared_ptr<lqr::QuadModel> quad_rpg{nullptr};
  std::shared_ptr<lqr::QuadModelBodyrates> quad_b_r{nullptr};
  const int n_states = 13;
  const int n_inputs = 4;
  const float m_rpg = 0.5;
  const float j_xxyy = 2.3;
  const float j_zz   = 4.0;
  quad_rpg = std::make_shared<lqr::QuadModel>(n_states, n_inputs, m_rpg, j_xxyy,
                                              j_zz);

  quad_b_r = std::make_shared<lqr::QuadModelBodyrates>();

  std::shared_ptr<lqr::QuadModelDrake> quad_drake{nullptr};

  quad_drake = std::make_shared<lqr::QuadModelDrake>(
      multibody::joints::kQuaternion);

  const std::string channel_x = "quadrotor_state";
  const std::string channel_ab = "quadrotor_ab_matrices";
  lcmt_quadrotor_ab_matrices msg_ab_matrices;

  std::vector<uint8_t> buffer_ab(msg_ab_matrices.getEncodedSize());

  examples::lqr::MessageHandler handler;
  lcm.Subscribe(channel_x, &handler);
  lcm.StartReceiveThread();


  Eigen::IOFormat CleanFmt(3, 0, " ", "\n", "[", "]");

  Eigen::VectorXf xx = Eigen::VectorXf::Zero(13);
  Eigen::VectorXf uu = Eigen::VectorXf::Zero(4);

  std::cout << "step in WaitForMessage" << std::endl;

  int message_count = 0;
  while (true) {
    message_count = handler.WaitForMessage(message_count);
    lcmt_quadrotor_state received_msg = handler.GetReceivedMessage();

    //Eigen::Map<Eigen::VectorXf> xx(received_msg.state_vector.data(), n_states + 3);
    //Eigen::Map<Eigen::VectorXf> uu(received_msg.input_vector.data(), n_inputs);

    //quad_drake->Update(xx, uu);
    quad_b_r->Update(xx.segment(0, 10), uu);
    quad_drake->Update(
        Eigen::Map<Eigen::VectorXf>(received_msg.state_vector.data(), n_states + 3),
        Eigen::Map<Eigen::VectorXf>(received_msg.input_vector.data(), n_inputs));

    quad_b_r->Update(xx.segment(0, 10), uu);

    Eigen::Map<Eigen::MatrixXf>(msg_ab_matrices.a_matrix, quad_drake->A_.rows(),
                                quad_drake->A_.cols()) = quad_drake->A_;
    Eigen::Map<Eigen::MatrixXf>(msg_ab_matrices.b_matrix, quad_drake->B_.rows(),
                                quad_drake->B_.cols()) = quad_drake->B_;

    std::cout << "A_b_r" << std::endl << quad_b_r->A_.format(CleanFmt)
              << std::endl;
    std::cout << "A_drake" << std::endl << quad_drake->A_.format(CleanFmt)
              << std::endl;

     //raise(SIGSEGV);
    //msg_ab_matrices.timestamp = 1103;
    msg_ab_matrices.n_states = 10;
    msg_ab_matrices.n_inputs = n_inputs;
    //std::cout << "publish after state received" << std::endl;

    msg_ab_matrices.encode(&buffer_ab[0], 0, msg_ab_matrices.getEncodedSize());
    lcm.Publish(channel_ab, &buffer_ab[0], msg_ab_matrices.getEncodedSize());
   }

/*
  //counthandler.WaitForMessage(0);
  std::cout << "step out WaitForMessage" << std::endl;
  lcmt_quadrotor_state received_msg = handler.GetReceivedMessage();
  std::cout << "received timestampt " << received_msg.timestamp << std::endl;
  std::cout << "received n_inputs " << received_msg.n_inputs << std::endl;

  Eigen::Map<Eigen::VectorXf> xx(received_msg.state_vector.data(), n_states);
  Eigen::Map<Eigen::VectorXf> uu(received_msg.input_vector.data(), n_inputs);

  quad_drake->Update(xx, uu);

  Eigen::Map<Eigen::MatrixXf>(msg_ab_matrices.a_matrix, quad_drake->A_.rows(),
      quad_drake->A_.cols()) = quad_drake->A_;
  Eigen::Map<Eigen::MatrixXf>(msg_ab_matrices.b_matrix, quad_drake->B_.rows(),
      quad_drake->B_.cols()) = quad_drake->B_;

  //msg_ab_matrices.timestamp = 1103;
  msg_ab_matrices.n_states = n_states;
  msg_ab_matrices.n_inputs = n_inputs;
  std::cout << "publish after state received" << std::endl;

  msg_ab_matrices.encode(&buffer_ab[0], 0, msg_ab_matrices.getEncodedSize());
  lcm.Publish(channel_ab, &buffer_ab[0], msg_ab_matrices.getEncodedSize());
*/
  // RPG state and input vectors.
  Eigen::VectorXf x0_rpg = Eigen::VectorXf::Zero(quad_rpg->n_states());
  Eigen::VectorXf u0_rpg = Eigen::VectorXf::Zero(quad_rpg->n_inputs());

  float thrust_factor = 1.0;

  const double kM = 0.0245;
  const double kF = 1;
  const double L = 0.175;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4, 4);
  // This mapping matrix is designed for a quadrotor with 4 rotors.
  // Fill the actuator to body mapping matrix.
  B.block(1, 0, 3, 4) << 0.0, L*kF, 0.0, -L*kF,
      -L*kF, 0.0, L*kF, 0.0,
      kM, -kM, kM, -kM;
  B.block(0, 0, 1, 4) << kF, kF, kF, kF;

  u0_rpg << thrust_factor*9.806*m_rpg, 0, 0, 0;
  std::cout << "u_rpg\n" << u0_rpg << std::endl;

  // Drake kQuaternion input vector.
  Eigen::VectorXd u0_qtr = Eigen::VectorXd::Zero(4);
  u0_qtr << Eigen::VectorXd::Constant(4, thrust_factor*m_rpg * 9.81 / 4);
  u0_qtr << 1.0, 0.1, 1.2, 1.31;
  //u0_qtr << 0, 0, 0, 0;
  std::cout << "u0_qtr\n" << u0_qtr << std::endl;
  std::cout << "B*u0_qtr\n" << B*u0_qtr << std::endl;
  u0_rpg << (B*u0_qtr).cast<float>();

  // Define the nominal state for the linearization.
  // properly define quaternion
  x0_rpg.segment(3, 1) << 1;

  // quaternion some pitch
  //x0_rpg.segment(3, 4) << 0.940, 0, 0.342, 0;
  // pitch 90 deg
  //x0_rpg.segment(3, 4) << 0.707, 0, 0.707, 0;
  // 10 deg roll 10 deg pitch 10 deg yaw

  Eigen::Vector3d roll_pitch_yaw;
  roll_pitch_yaw << 0, 45, 0;
  roll_pitch_yaw *= M_PI / 180;
  Eigen::Vector4d quaternion = math::rpy2quat(roll_pitch_yaw);
  x0_rpg.segment(3, 4) <<  quaternion.cast<float>();

  // add rotational velocity
  x0_rpg.segment(10, 1) << 0.31;
  x0_rpg.segment(11, 1) << 0.1;
  //x0_rpg.segment(12, 1) << 0.220;

  // add linear velocity
  x0_rpg.segment(9, 1) << 0.70;

  // Get the A and B matrix using the fullstate RPG quad_model.
  quad_rpg->Update(x0_rpg, u0_rpg);
  quad_drake->Update(x0_rpg, u0_rpg);


  // This block does not make sense, so set to zero??
  //P_A_qtr_P_inv.block(7, 7, 3, 3) << Eigen::MatrixXd::Zero(3, 3);

  std::cout << "A_rpg" << std::endl << quad_rpg->A_.format(CleanFmt)
            << std::endl;
  std::cout << "A_drake" << std::endl << quad_drake->A_.format(CleanFmt)
            << std::endl;
  std::cout << "A_drake - A_rpg" << std::endl
            << (quad_drake->A_ - quad_rpg->A_).format(CleanFmt) << std::endl;
  std::cout << "B_rpg" << std::endl << quad_rpg->B_.format(CleanFmt)
            << std::endl;
  std::cout << "B_drake" << std::endl << quad_drake->B_.format(CleanFmt)
            << std::endl;
  std::cout << "B_drake - B_rpg" << std::endl
            << (quad_drake->B_ - quad_rpg->B_).format(CleanFmt) << std::endl;


  std::cout << "\n###### Test Ax and Ax+Bu with nominal states ######\n\n";

  std::cout << "A_rpg * x0_rpg" << std::endl
            << quad_rpg->A_ * x0_rpg << "\n\n";
  std::cout << "A_drake * x0_drake" << std::endl
            << quad_drake->A_ * x0_rpg << "\n\n";
  std::cout << "A_rpg * x0_rpg + B_ * u0_rpg" << std::endl
            << quad_rpg->A_ * x0_rpg + quad_rpg->B_ * u0_rpg << "\n\n";
  std::cout << "A_drake * x0_drake + B_ * u0_rpg" << std::endl
            << quad_drake->A_ * x0_rpg + quad_drake->B_ * u0_rpg << "\n\n";


  std::cout << "\n###### Test Ax and Ax+Bu with new states ######\n\n";

  float thrust_factor_test = 1.0;

  Eigen::VectorXf x1_rpg_test = Eigen::VectorXf::Constant(n_states, 0.2);
  x1_rpg_test.segment(3, 4) <<  0.996, 0.047, 0.052, 0.047;
  Eigen::VectorXf u1_rpg_test = Eigen::VectorXf::Constant(n_inputs, 0.1);
  u1_rpg_test << 0.0 * 9.806*m_rpg, 0, 0, 0;
  u1_rpg_test << u0_rpg;
  u1_rpg_test *= thrust_factor_test;

  std::cout << "x1_rpg_test\n" << x1_rpg_test << std::endl;
  std::cout << "u1_rpg_test\n" << u1_rpg_test << std::endl;

  //set zero (0, 3, 3, 4) because doesnt make sense...
  //P_A_qtr_P_inv.block(0, 3, 3, 4).setZero();
  std::cout << "A_rpg * x1_rpg_test" << std::endl
            << quad_rpg->A_ * x1_rpg_test << "\n\n";
  std::cout << "A_drake * x1_rpg_test" << std::endl
            << quad_drake->A_ * x1_rpg_test << "\n\n";
  std::cout << "A_rpg * x1_rpg_test + B_ * u1_rpg_test" << std::endl
            << quad_rpg->A_ * x1_rpg_test + quad_rpg->B_ * u1_rpg_test << "\n\n";
  std::cout << "A_drake * x1_rpg_test + B_ * u1_rpg_test" << std::endl
            << quad_drake->A_ * x1_rpg_test + quad_drake->B_ * u1_rpg_test << "\n\n";


  const Eigen::MatrixXf Q_{(Eigen::Matrix<float,13,1>() <<
                           10, 10, 50,
                           1*Eigen::MatrixXf::Ones(4,1),
                           1*Eigen::MatrixXf::Ones(3,1),
                           1*Eigen::MatrixXf::Ones(3,1)
                           ).finished().asDiagonal()};
  const Eigen::MatrixXf R_{(Eigen::Matrix<float,4,1>() <<
                           1, 1, 1, 2).finished().asDiagonal()};
  Eigen::MatrixXf K_rpg;
  Eigen::MatrixXf K_drake;

  lqr::TimeInvariantLQR tilqr_rpg(std::move(quad_rpg), Q_, R_);
  K_rpg = tilqr_rpg.Solve(x0_rpg, u0_rpg);
  lqr::TimeInvariantLQR tilqr_drake(std::move(quad_drake), Q_, R_);
  K_drake = tilqr_drake.Solve(x0_rpg, u0_rpg);

  std::cout << "K rpg\n" <<  K_rpg.format(CleanFmt) << std::endl;
  std::cout << "K drake\n" <<  K_drake.format(CleanFmt) << std::endl;

  std::cout << "PROVOKING SEGMENT FAULT TO OUTPUT COUT IN CLion..." << std::endl;
  raise(SIGSEGV);

  /* NOT USED CURRENTLY

  // This is not correct since A_hat is defined as P*A*P_inv
  Eigen::MatrixXd P_test = get_P(x0_rpg_test);
  Eigen::MatrixXd P_test_inv = P_test.inverse();


  const Eigen::MatrixXf Q_{(Eigen::Matrix<float, 10, 1>() <<
                                                          10, 10, 50,
      1 * Eigen::MatrixXf::Ones(4, 1),
      1 * Eigen::MatrixXf::Ones(3, 1)//,
                               //1*Eigen::MatrixXf::Ones(3,1)
                           ).finished().asDiagonal()};
  const Eigen::MatrixXf R_{(Eigen::Matrix<float, 4, 1>() <<
                                                         1, 1, 1, 2).finished().asDiagonal()};


  std::shared_ptr<lqr::QuadModelBodyrates> quad_{nullptr};
  quad_ = std::make_shared<lqr::QuadModelBodyrates>();
  Eigen::VectorXf q0_ = Eigen::VectorXf::Zero(quad_->n_states());
  Eigen::VectorXf u0_ = quad_->u0();
  // quad_(std::make_shared<QuadModel>(0.6, 3.5, 7.0)),
  Eigen::MatrixXf K_ =
      Eigen::MatrixXf::Zero(quad_->n_states(), quad_->n_states());

  //TimeInvariantLQR tilqr_(quad_, Q_, R_);
  //lqr::IterativeLQR ilqr_(quad_, Q_, R_, 10 * Q_);
  */
  /*
  Eigen::Vector4d quaternion1;
  quaternion1 << 1, 0, 0, 0;
  std::cout << "R\n" << math::quat2rotmat(quaternion1) << std::endl;
  std::cout << "E\n" << math::quat2rpy(quaternion1) << std::endl << std::endl;
  Eigen::Vector4d quaternion2;
  quaternion2 << 0.957, 0.106, -0.250, 0.106;
  std::cout << "R\n" << math::quat2rotmat(quaternion2) << std::endl;
  std::cout << "E\n" << math::quat2rpy(quaternion2) << std::endl << std::endl;
  Eigen::Vector4d quaternion3;
  quaternion3 << 0.707, 0, 0.707, 0;
  std::cout << "R\n" << math::quat2rotmat(quaternion3) << std::endl;
  std::cout << "E\n" << math::quat2rpy(quaternion3) << std::endl << std::endl;
  */


  /*
   *
  std::vector<uint8_t> buffer_x(msg_state.getEncodedSize());
  std::vector<uint8_t> buffer_u(msg_k_matrix.getEncodedSize());

   *
   *
   *
int t = 0;
while (t < 10) {
  // Publishes msg_state.
  msg_state.n_states = 13;
  msg_state.n_inputs = 4;
  msg_state.n_inputs = 4;
  msg_state.input_vector.resize(4);
  msg_state.state_vector.resize(13);
  std::cout << "publish sth" << std::endl;

  msg_state.encode(&buffer_x[0], 0, msg_state.getEncodedSize());
  //lcm.Publish(channel_x, &buffer_x[0], msg_state.getEncodedSize());
  std::cout << "publish sth" << std::endl;



  // Publishes msg_k_matrix using received msg_state.
  if (handler.get_receive_channel() == channel_x) {
    // Gets the received message.
    const lcmt_quadrotor_state received_msg = handler.GetReceivedMessage();

    // Calculates some output from received state.
    msg_k_matrix.n_inputs = 4;
    msg_k_matrix.n_states = 13;
    msg_k_matrix.k_array_length = 13*4;
    msg_k_matrix.k_array.resize(13*4);

    // Publish msg_k_matrix.
    msg_k_matrix.encode(&buffer_u[0], 0, msg_k_matrix.getEncodedSize());
    //lcm.Publish(channel_ab, &buffer_u[0], msg_k_matrix.getEncodedSize());
  }




  std::cout << "step in WaitForMessage" << std::endl;
  handler.WaitForMessage(0);
  std::cout << "step out WaitForMessage" << std::endl;
  const lcmt_quadrotor_state received_msg = handler.GetReceivedMessage();
  std::cout << "received timestampt " << received_msg.timestamp << std::endl;
  std::cout << "received n_inputs " << received_msg.n_inputs << std::endl;

  msg_ab_matrices.n_states = 13;
  msg_ab_matrices.n_inputs = 4;
  msg_ab_matrices.a_array_length = 13*13;
  msg_ab_matrices.a_array_length = 4*13;
  msg_ab_matrices.a_matrix.resize(13*13);
  msg_ab_matrices.b_matrix.resize(4*13);
  std::cout << "publish after state received" << std::endl;

  msg_ab_matrices.encode(&buffer_ab[0], 0, msg_ab_matrices.getEncodedSize());
  lcm.Publish(channel_ab, &buffer_ab[0], msg_ab_matrices.getEncodedSize());

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  t++;
}
*/


  return 1;
}

} // namespace drake
} // namespace systems
} // namespace lqr


int main(int argc, char **argv)
{
  //ros::init(argc, argv, "rpg_ilqr");
  //lqr::Controller controller(ros::NodeHandle(), ros::NodeHandle("~"));
  //ros::spin();
  drake::systems::DoMain();
  return 0;
}
