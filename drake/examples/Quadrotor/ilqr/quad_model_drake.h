#pragma once

#include <math.h>

#include <Eigen/Dense>
#include "drake/examples/Quadrotor/ilqr/quad_model.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_autodiff.h"
#include "drake/systems/framework/diagram_builder.h"

namespace lqr {

class QuadModelDrake : public QuadModel {
 public:
  QuadModelDrake(drake::multibody::joints::FloatingBaseType joint_type) :
      QuadModel(10, 4, 0.5, 0.0, 0.0) {

    drake::systems::DiagramBuilder<drake::AutoDiffXd> builder;

    auto tree = std::make_unique<RigidBodyTree<double>>();

    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        drake::FindResourceOrThrow("drake/examples/Quadrotor/quadrotor.urdf"),
        joint_type, tree.get());

    drake::systems::RigidBodyPlantAutodiff<double>::SetupInputMatrixB(tree->B);

    //m_ = tree->getMass();

    auto treead = tree->ToAutoDiffXd();
    plant_ =
        builder.template AddSystem<
            drake::systems::RigidBodyPlantAutodiff<drake::AutoDiffXd>>(
        std::move(treead));
    plant_->set_name("plantAD");
    diagram_ = builder.Build();
  }

  virtual void CalculateA(const Eigen::Ref<const Eigen::VectorXf>& q,
                          const Eigen::Ref<const Eigen::VectorXf>& u);
  virtual void CalculateB(const Eigen::Ref<const Eigen::VectorXf>& q,
                          const Eigen::Ref<const Eigen::VectorXf>& u);

 private:
  drake::systems::RigidBodyPlantAutodiff<drake::AutoDiffXd> *plant_{};
  std::unique_ptr<drake::systems::Diagram<drake::AutoDiffXd>> diagram_{};
  int counter_{0};
};

} // namespace ilqr
