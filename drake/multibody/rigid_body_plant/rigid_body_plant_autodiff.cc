#include "drake/multibody/rigid_body_plant/rigid_body_plant_autodiff.h"

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

using Eigen::VectorXd;

namespace drake {
namespace systems {


template <typename T>
RigidBodyPlantAutodiff<T>::~RigidBodyPlantAutodiff() {}

// TODO(liang.fok) Eliminate the re-computation of `xdot` once it is cached.
// Ideally, switch to outputting the derivatives in an output port. This can
// only be done once #2890 is resolved.
template <typename T>
void RigidBodyPlantAutodiff<T>::PrintValue(const int& value) {
  std::cout << "value" << value << std::endl;
}

// Explicitly instantiates on the most common scalar types.
template class RigidBodyPlantAutodiff<double>;

}  // namespace systems
}  // namespace drake
