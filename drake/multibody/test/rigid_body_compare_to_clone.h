#pragma once

#include "drake/multibody/rigid_body.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body {

/**
 * Compares the provided `original` object to the provided `clone` and verifies
 * that the clone is correct relative to the original. Since this method is
 * intended to compare an original to its clone, an *exact* match is performed.
 * This method will only return `true` if the provided `original` is exactly the
 * same as the provided clone.
 */
template <typename T>
bool CompareToClone(const RigidBody<double>& original,
                    const RigidBody<T>& clone);

}  // namespace rigid_body
}  // namespace test
}  // namespace multibody
}  // namespace drake
