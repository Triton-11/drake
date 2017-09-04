//#include <drake/examples/Quadrotor/ilqr/controller.h>

#include <Eigen/Dense>
#include <iostream>
#include <signal.h>

#include "drake/examples/Quadrotor/ilqr/quad_model_drake.h"

// drake includes
#include "drake/examples/Quadrotor/ilqr/quadrotor_lcm_msg_handler.h"
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

namespace drake {
namespace systems {
namespace {

int DoMain() {

  drake::lcm::DrakeLcm lcm;

  std::shared_ptr<lqr::QuadModel> quad_rpg{nullptr};
  const int n_states = 10;
  const int n_inputs = 4;

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
    quad_drake->Update(
        Eigen::Map<Eigen::VectorXf>(received_msg.state_vector.data(), n_states + 3),
        Eigen::Map<Eigen::VectorXf>(received_msg.input_vector.data(), n_inputs));

    Eigen::Map<Eigen::MatrixXf>(msg_ab_matrices.a_matrix, quad_drake->A_.rows(),
                                quad_drake->A_.cols()) = quad_drake->A_;
    Eigen::Map<Eigen::MatrixXf>(msg_ab_matrices.b_matrix, quad_drake->B_.rows(),
                                quad_drake->B_.cols()) = quad_drake->B_;

    std::cout << "A_drake" << std::endl << quad_drake->A_.format(CleanFmt)
              << std::endl;

    //msg_ab_matrices.timestamp = 1103;
    msg_ab_matrices.n_states = 10;
    msg_ab_matrices.n_inputs = n_inputs;
    //std::cout << "publish after state received" << std::endl;

    msg_ab_matrices.encode(&buffer_ab[0], 0, msg_ab_matrices.getEncodedSize());
    lcm.Publish(channel_ab, &buffer_ab[0], msg_ab_matrices.getEncodedSize());
   }

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
