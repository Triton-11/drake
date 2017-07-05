#include "drake/multibody/test/rigid_body_frame_compare_to_clone.h"

#include <memory>

#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_frame {

template <typename T>
bool CompareToClone(const RigidBodyFrame<T>& original,
                  const RigidBodyFrame<T>& clone) {
  if (original.get_name() != clone.get_name()) {
    drake::log()->debug(
        "RigidBodyFrame::CompareToClone(): name mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_name(),
        clone.get_name());
    return false;
  }
  if (original.get_rigid_body().get_body_index() !=
      clone.get_rigid_body().get_body_index()) {
    drake::log()->debug(
        "RigidBodyFrame::CompareToClone(): rigid body index mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_rigid_body().get_body_index(),
        clone.get_rigid_body().get_body_index());
    return false;
  }
  if (original.get_transform_to_body().matrix() !=
      clone.get_transform_to_body().matrix()) {
    drake::log()->debug(
        "RigidBodyFrame::CompareToClone(): transform_to_body mismatch:\n"
        "  - original:\n{}\n"
        "  - clone:\n{}",
        original.get_transform_to_body().matrix(),
        clone.get_transform_to_body().matrix());
    return false;
  }
  if (original.get_frame_index() != clone.get_frame_index()) {
    drake::log()->debug(
        "RigidBodyFrame::CompareToClone(): frame index mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_frame_index(),
        clone.get_frame_index());
    return false;
  }
  return true;
}

bool CompareToAutoDiffXd(const RigidBodyFrame<double>& original,
                         const RigidBodyFrame<AutoDiffXd>& clone) {
  if (original.get_name() != clone.get_name()) {
    drake::log()->debug(
        "RigidBodyFrame::CompareToClone(): name mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_name(),
        clone.get_name());
    return false;
  }
  if (original.get_rigid_body().get_body_index() !=
      clone.get_rigid_body().get_body_index()) {
    drake::log()->debug(
        "RigidBodyFrame::CompareToClone(): rigid body index mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_rigid_body().get_body_index(),
        clone.get_rigid_body().get_body_index());
    return false;
  }
  if (original.get_transform_to_body().matrix() !=
      clone.get_transform_to_body().matrix()) {
    drake::log()->debug(
        "RigidBodyFrame::CompareToClone(): transform_to_body mismatch:\n"
        "  - original:\n{}\n"
        "  - clone:\n{}",
        original.get_transform_to_body().matrix(),
        clone.get_transform_to_body().matrix());
    return false;
  }
  if (original.get_frame_index() != clone.get_frame_index()) {
    drake::log()->debug(
        "RigidBodyFrame::CompareToClone(): frame index mismatch:\n"
        "  - original: {}\n"
        "  - clone: {}",
        original.get_frame_index(),
        clone.get_frame_index());
    return false;
  }
  return true;
}

// Explicitly instantiates on the most common scalar types.
template bool CompareToClone<double>(const RigidBodyFrame<double>& original,
                                     const RigidBodyFrame<double>& clone);
template bool CompareToClone<AutoDiffXd>(
    const RigidBodyFrame<AutoDiffXd>& original,
    const RigidBodyFrame<AutoDiffXd>& clone);

}  // namespace rigid_body_frame
}  // namespace test
}  // namespace multibody
}  // namespace drake
