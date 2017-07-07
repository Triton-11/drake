#include "drake/multibody/rigid_body_plant/rigid_body_plant_autodiff.h"

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/roll_pitch_yaw.h"

using Eigen::VectorXd;

namespace drake {
namespace systems {


template <typename T>
RigidBodyPlantAutodiff<T>::RigidBodyPlantAutodiff(std::unique_ptr<const RigidBodyTree<T>> tree,
                                  double timestep)
    : RigidBodyPlant<T>::RigidBodyPlant(move(tree)),
      tree_(this->get_rigid_body_tree()), timestep_(timestep) {
  this->DeclareInputPort(systems::kVectorValued, kInputDimension);
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
  VectorX<T> u_thrusts = EvaluateActuatorInputs(context);

  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  const int nq = this->get_num_positions();
  const int nv = this->get_num_velocities();
  const int num_inputs = (this->get_num_input_ports() > 0)
                         ? this->get_input_port(0).size()
                         : 0;
  std::cout << "num_inputs: " << num_inputs << std::endl;
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
  eigen_aligned_std_unordered_map<
      RigidBody<T> const*, WrenchVector<T>> no_external_wrenches;
  // right_hand_side is the right hand side of the system's equations:
  // H*vdot -J^T*f = right_hand_side.
  VectorX<T> right_hand_side =
      -tree_.dynamicsBiasTerm(kinsol, no_external_wrenches);

  std::cout << "right_hand_side:" << std::endl << right_hand_side << std::endl;
  // TODO(robinsch): Correctly compute input forces here
  //right_hand_side += F;
  if (num_inputs > 0) right_hand_side += ThrustsToSpatialForce(context,
                                                               u_thrusts);
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
  std::cout << "Cholesky cols and rows:" << std::endl;
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
template <typename T>
void RigidBodyPlantAutodiff<T>::LinearizeAB(const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0) {
//  return;

  //DRAKE_DEMAND(context.is_stateless() || context.has_only_continuous_state());
  // TODO(russt): handle the discrete time case

  // System and context are input, both templated on <double>.

  // IDEA: inputs are current state vector (just VectorXd from external source)
  //       and current thrust inputs (again VectorXd), time and accuracy. Create
  //       an Autodiff context and setup with the states etc.
  //       DoCalcTimeDerivative with the created context

  DRAKE_DEMAND(this->get_num_input_ports() <= 1);
  //DRAKE_DEMAND(this->get_num_output_ports() <= 1);

  const int num_inputs = (this->get_num_input_ports() > 0)
                         ? this->get_input_port(0).size()
                         : 0,
      num_outputs = (this->get_num_output_ports() > 0)
                    ? this->get_output_port(0).size()
                    : 0;
  const int num_states = this->get_num_states();
  std::cout << "num_states: " << num_states << std::endl;

  // TODO(robinsch): check whether ToAutoDiffXd can be used for RBT/RBTAutodiff
  // this could be super helpful to still use URDF files to create RBT<double>
  // and then clone them to <AutoDiffXd> versions. This should be done outside
  // the LinearizeAB function in order to not do it again in every cycle.
  // Create an autodiff version of the system.
  //std::unique_ptr<System<AutoDiffXd>> autodiff_system =
  //    drake::systems::System<double>::ToAutoDiffXd(system);

  // Initialize autodiff context
  std::unique_ptr<Context<AutoDiffXd>> autodiff_context =
      this->CreateDefaultContext();
  // Try without, in the case no context is passed
  //autodiff_context->SetTimeStateAndParametersFrom(context);

  // This should be the input argument
  //const VectorXd x0 =
  //    context.get_continuous_state_vector().CopyToVector();
  //const int num_states = x0.size();

  // This should be the input argument
  //VectorXd u0 = Eigen::VectorXd::Zero(num_inputs);
  //if (num_inputs > 0) {
  //  u0 = this->EvalEigenVectorInput(context, 0);
  //}

  auto autodiff_args = math::initializeAutoDiffTuple(x0, u0);
  autodiff_context->get_mutable_continuous_state_vector()->SetFromVector(
      std::get<0>(autodiff_args));

  if (num_inputs > 0) {
    auto input_vector = std::make_unique<BasicVector<AutoDiffXd>>(num_inputs);
    input_vector->SetFromVector(std::get<1>(autodiff_args));
    autodiff_context->SetInputPortValue(
        0,
        std::make_unique<FreestandingInputPortValue>(std::move(input_vector)));
  }

  std::unique_ptr<ContinuousState<T>> autodiff_xdot =
      this->AllocateTimeDerivatives();
  this->CalcTimeDerivatives(*autodiff_context, autodiff_xdot.get());
  auto autodiff_xdot_vec = autodiff_xdot->CopyToVector();

  // We DON'T only want equilibrium states...
  /*
  // Ensure that xdot0 = f(x0,u0) == 0.
  if (!math::autoDiffToValueMatrix(autodiff_xdot_vec)
      .isZero(equilibrium_check_tolerance)) {
    throw std::runtime_error(
        "The nominal operating point (x0,u0) is not an equilibrium point of "
            "the system.  Without additional information, a time-invariant "
            "linearization of this system is not well defined.");
  }
  */

  Eigen::MatrixXd AB = math::autoDiffToGradientMatrix(autodiff_xdot_vec);
  Eigen::MatrixXd A = AB.leftCols(num_states);
  Eigen::MatrixXd B = AB.rightCols(num_inputs);
  std::cout << "AB" << std::endl << AB << std::endl;
  std::cout << "A" << std::endl << A << std::endl;
  std::cout << "B" << std::endl << B << std::endl;
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_outputs, num_states);
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_outputs, num_inputs);
  /*
  if (num_outputs > 0) {
    std::unique_ptr<SystemOutput<AutoDiffXd>> autodiff_y0 =
        autodiff_system->AllocateOutput(*autodiff_context);
    autodiff_system->CalcOutput(*autodiff_context, autodiff_y0.get());
    auto autodiff_y0_vec = autodiff_y0->get_vector_data(0)->CopyToVector();

    Eigen::MatrixXd CD = math::autoDiffToGradientMatrix(autodiff_y0_vec);
    C = CD.leftCols(num_states);
    D = CD.rightCols(num_inputs);
  }
   */
  //   return std::make_unique<LinearSystem<double>>(A, B, C, D);

}

template <typename T>
VectorX<T> RigidBodyPlantAutodiff<T>::EvaluateActuatorInputs(
    const Context<T>& context) const {
  VectorX<T> u = this->EvalVectorInput(context, 0)->get_value();
  return u;
}

template <typename T>
VectorX<T> RigidBodyPlantAutodiff<T>::ThrustsToSpatialForce(
    const Context<T>& context,
    const VectorX<T>& u_thrusts) const {

  VectorX<T> state = context.get_continuous_state_vector().CopyToVector();
  std::cout << "state:" << std::endl << state << std::endl;

  // Extract orientation and angular velocities.
  Vector3<T> rpy = state.segment(3, 3);

  // Convert orientation to a rotation matrix.
  Matrix3<T> R = math::rpy2rotmat(rpy);

  // TODO(robinsch): Make matrix size dependent on number of states
  Matrix3<T> Z;
  Z.fill(0.0);
  Matrix6<T> RR;
  RR << R, Z, Z, R;
  std::cout << "RR" << std::endl <<RR << std::endl;
  std::cout << "tree_.B" << std::endl << tree_.B << std::endl;
  std::cout << "B * u" << std::endl << tree_.B * u_thrusts << std::endl;


  VectorX<T> F = RR * tree_.B * u_thrusts;

  return F;
}

// TODO(liang.fok) Eliminate the re-computation of `xdot` once it is cached.
// Ideally, switch to outputting the derivatives in an output port. This can
// only be done once #2890 is resolved.
template <typename T>
void RigidBodyPlantAutodiff<T>::PrintValue(const int& value) {
  std::cout << "value" << value << std::endl;
}

// Explicitly instantiates on the most common scalar types.
//template class RigidBodyPlantAutodiff<double>;
template class RigidBodyPlantAutodiff<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
