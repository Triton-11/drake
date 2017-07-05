#include "drake/multibody/test/rigid_body_loop_compare_to_clone.h"

#include <memory>

#include "drake/common/text_logging.h"
#include "drake/multibody/test/rigid_body_frame_compare_to_clone.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_loop {

template <typename T>
bool CompareToClone(const RigidBodyLoop<double>& original,
                  const RigidBodyLoop<T>& other) {
  if (!rigid_body_frame::CompareToClone(*original.frameA_, *other.frameA_)) {
    drake::log()->debug("CompareToClone(RigidBodyLoop): FrameA mismatch.");
    return false;
  }
  if (!rigid_body_frame::CompareToClone(*original.frameB_, *other.frameB_)) {
    drake::log()->debug("CompareToClone(RigidBodyLoop): FrameB mismatch.");
    return false;
  }
  if (original.axis_ != other.axis_) {
    drake::log()->debug(
        "CompareToClone(RigidBodyLoop): Axes mismatch:\n"
        "  - this: {}\n"
        "  - other: {}",
        original.axis_.transpose(), other.axis_.transpose());
    return false;
  }
  return true;
}

// Explicitly instantiates on the most common scalar types.
template bool CompareToClone<double>(const RigidBodyLoop<double>& original,
                                     const RigidBodyLoop<double>& other);
template bool CompareToClone<AutoDiffXd>(const RigidBodyLoop<double>& original,
                                         const RigidBodyLoop<AutoDiffXd>& other);

}  // namespace rigid_body_loop
}  // namespace test
}  // namespace multibody
}  // namespace drake
