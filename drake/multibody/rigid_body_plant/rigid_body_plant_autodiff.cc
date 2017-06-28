#include "drake/multibody/rigid_body_plant/rigid_body_plant_autodiff.h"

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/math/roll_pitch_yaw.h"

using Eigen::VectorXd;

namespace drake {
namespace systems {


template <typename T>
RigidBodyPlantAutodiff<T>::RigidBodyPlantAutodiff(std::unique_ptr<const RigidBodyTree<T>> tree,
                                  double timestep)
    : RigidBodyPlant<T>::RigidBodyPlant(move(tree)),
      tree_(this->get_rigid_body_tree()), timestep_(timestep) {
  /*state_output_port_index_ =
      this->DeclareOutputPort(kVectorValued, get_num_states()).get_index();
  ExportModelInstanceCentricPorts();
  // Declares an abstract valued output port for kinematics results.
  kinematics_output_port_index_ = this->DeclareAbstractOutputPort(
      Value<KinematicsResults<T>>(tree_.get())).get_index();
  // Declares an abstract valued output port for contact information.
  contact_output_port_index_ = this->DeclareAbstractOutputPort(
      Value<ContactResults<T>>()).get_index();
      */
}

template <typename T>
RigidBodyPlantAutodiff<T>::~RigidBodyPlantAutodiff() {}

template <typename T>
void RigidBodyPlantAutodiff<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  //static_assert(std::is_same<double, T>::value,
  //              "Only support templating on double for now");
  if (timestep_ > 0.0) return;

  // TODO(robinsch): Proper Actuator inputs with tree
  //VectorX<T> u = EvaluateActuatorInputs(context);

  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  const int nq = this->get_num_positions();
  const int nv = this->get_num_velocities();
  //const int num_actuators = get_num_actuators();
  // TODO(robinsch): Proper num_actuators with tree
  const int num_actuators = 4;
  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block
  // which
  // is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  auto kinsol = tree_.doKinematics(q, v);


  auto H = tree_.massMatrix(kinsol);  // Alternatively: MatrixX<T> H

  // There are no external wrenches, but it is a required argument in
  // dynamicsBiasTerm.
  // TODO(amcastro-tri): external_wrenches should be made an optional
  // parameter
  // of dynamicsBiasTerm().
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
  // right_hand_side is the right hand side of the system's equations:
  // H*vdot -J^T*f = right_hand_side.
  VectorX<T> right_hand_side =
      -tree_.dynamicsBiasTerm(kinsol, no_external_wrenches);
  VectorX<T> u_;  // The plant-centric input vector of actuation values.
  u_.resize(4);
  u_.fill(0.);
  u_ << 1.6, 1.5, 1.6, 1.51;

  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();


  // Extract orientation and angular velocities.
  Vector3<T> rpy = state.segment(3, 3);

  // Convert orientation to a rotation matrix.
  Matrix3<T> R = math::rpy2rotmat(rpy);

  Matrix3<T> Z;
  Z.fill(0.0);
  Matrix6<T> RR;
  RR << R, Z, Z, R;
  std::cout << "RR" << std::endl <<RR << std::endl;


  std::cout << "right_hand_side:" << std::endl << right_hand_side << std::endl;
  /*
  double kM = 0.0245;
  double kF = 1;
  double L = 0.175;


  Eigen::MatrixXd B;
  B.resize(nv, num_actuators);
  B = Eigen::MatrixXd::Zero(nv, num_actuators);
  B << 0, 0, 0, 0,
       0, 0, 0, 0,
       kF, kF, kF, kF,
       0, L*kF, 0, -L*kF,
       -L*kF, 0, L*kF, 0,
       kM, -kM, kM, -kM;

  tree_.B << 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              kF, kF, kF, kF,
              0.0, L*kF, 0.0, -L*kF,
              -L*kF, 0.0, L*kF, 0.0,
              kM, -kM, kM, -kM;
  */
  // TODO(robinsch): Correctly compute input forces here
  if (num_actuators > 0) right_hand_side += RR * tree_.B * u_;
  std::cout << "tree_.B" << std::endl << tree_.B << std::endl;
  std::cout << "B * u" << std::endl << tree_.B * u_ << std::endl;

  /*
  Vector3<T> F_xyz (4, 0, 0);
  Vector3<T> F_xyz_rot = R * F_xyz;
  Vector3<T> M_xyz;

  if (-0.01 < static_cast<T>(rpy[1]) &&  static_cast<T>(rpy[1]) < 0.01) {
    M_xyz << 0, 3.3, 0;
  } else {
    M_xyz << 0, 0, 0;
  }

  Vector6<T> F;
  F << F_xyz_rot, M_xyz;
  F(2) += 4.905;

  //VectorX<T> F;  // The plant-centric input vector of actuation values.
  //F.resize(6);
  //F.fill(0.);

  */

  //right_hand_side += F;
  // Applies joint limit forces.
  // TODO(amcastro-tri): Maybe move to
  // RBT::ComputeGeneralizedJointLimitForces(C)?
  // TODO(robinsch): Not sure this works with autodiff
  {
    for (auto const& b : tree_.bodies) {
      if (!b->has_parent_body()) continue;
      auto const& joint = b->getJoint();
      // Joint limit forces are only implemented for single-axis joints.
      if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
        const T limit_force =
            this->JointLimitForce(joint, q(b->get_position_start_index()),
                            v(b->get_velocity_start_index()));
        right_hand_side(b->get_velocity_start_index()) += limit_force;
      }
    }
  }

  // Contact force does not support autodiff (ComputeMaximumDepthCollisionPoints doesn't)
  // https://github.com/RobotLocomotion/drake/issues/4267#issuecomment-264078942
  /*
  // Returns zero for now
  right_hand_side += ComputeContactForce(kinsol);
  */

  // There are no position constraints (loops) in our RBT, so we can skip the
  // tree_.getNumPositionConstraints()

  // Solve the linear equation Hx=rhs for x
  // H has to be a Hermitian, positive-definite matrix
  Eigen::LLT<MatrixX<T>> lltOfH(H); // compute the Cholesky decomposition of A
  // Check if H is a Hermitian, positive-definite matrix
  if (lltOfH.info() != Eigen::Success) {
    // TODO(robinsch): Add proper error message when lltOfH fails.
    std::cout << "### FAIL ERROR of lltOfH" << std::endl;
    std::cout << "The H Matrix is:" << std::endl << H << std::endl;
  }

  auto vdot_cholesky = lltOfH.solve(right_hand_side);

  std::cout << vdot_cholesky.cols() << std::endl;
  std::cout << vdot_cholesky.rows() << std::endl;

  std::cout << "The H Matrix: " << std::endl << H << std::endl;
  /*
  std::cout << "To check H, let us compute L * L.transpose()" << std::endl;
  MatrixX<T> L = lltOfH.matrixL(); // retrieve factor L  in the decomposition
  std::cout << L * L.transpose() << std::endl;
  std::cout << "vdot_cholesky" << std::endl << vdot_cholesky << std::endl;
   */


  VectorX<T> xdot(this->get_num_states());
  //const auto& vdot_value = prog.GetSolution(vdot);
  xdot << tree_.transformVelocityToQDot(kinsol, v), vdot_cholesky;
  derivatives->SetFromVector(xdot);
  std::cout << "derivatives:" << std::endl;
  std::cout << derivatives->CopyToVector() << std::endl;
}

// TODO(liang.fok) Eliminate the re-computation of `xdot` once it is cached.
// Ideally, switch to outputting the derivatives in an output port. This can
// only be done once #2890 is resolved.
template <typename T>
void RigidBodyPlantAutodiff<T>::PrintValue(const int& value) {
  std::cout << "value" << value << std::endl;
}

// Explicitly instantiates on the most common scalar types.
template class RigidBodyPlantAutodiff<double>;
template class RigidBodyPlantAutodiff<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
