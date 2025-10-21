#include "trapezoidal_joint_controller/read_write_interfaces/ReferenceInterfaces.hpp"

namespace trapezoidal_joint_controller {

ReferenceInterfaces::ReferenceInterfaces(size_t dof)
         : controller_utils::interfaces::JointReferenceInterface(dof),
           mVelocityProfiles(dof, std::numeric_limits<double>::quiet_NaN()),
           mAccelerationProfiles(dof,
                                 std::numeric_limits<double>::quiet_NaN()) {
   // Initialize profile vectors with NaN values to indicate uninitialized state.
   // The base class handles position reference initialization.
}

void ReferenceInterfaces::setVelocityProfile(
       const std::vector<double>& values) {
   // Validate input size matches the configured number of degrees of freedom.
   // Log error but continue execution to avoid real-time control disruption.
   if (values.size() != mVelocityProfiles.size()) {
      spdlog::error(
             "The size of the velocity profile values does not match the "
             "number of joints. Expected: {}, Received: {}",
             mVelocityProfiles.size(), values.size());
   }
   // Copy velocity profile values for trajectory generation.
   std::copy(values.begin(), values.end(), mVelocityProfiles.begin());
}

void ReferenceInterfaces::setAccelerationProfile(
       const std::vector<double>& values) {
   // Validate input size matches the configured number of degrees of freedom.
   // Log error but continue execution to maintain real-time performance.
   if (values.size() != mAccelerationProfiles.size()) {
      spdlog::error(
             "The size of the acceleration profile values does not match the "
             "number of joints. Expected: {}, Received: {}",
             mAccelerationProfiles.size(), values.size());
   }
   // Copy acceleration profile values for trajectory constraint enforcement.
   std::copy(values.begin(), values.end(), mAccelerationProfiles.begin());
}

const std::vector<double>& ReferenceInterfaces::getVelocityProfile() const {
   // Return current velocity profile for all joints.
   // Used for status monitoring and trajectory planning.
   return mVelocityProfiles;
}

const std::vector<double>& ReferenceInterfaces::getAccelerationProfile() const {
   // Return current acceleration profile for all joints.
   // Used for trajectory constraint validation and monitoring.
   return mAccelerationProfiles;
}

double* ReferenceInterfaces::exportReferenceProfileVelocity(size_t index) {
   // Export direct pointer access to velocity profile data for performance.
   // Used by hardware interface system for efficient data sharing.
   if (index >= mVelocityProfiles.size()) {
      spdlog::error("The index is out of bounds for the velocity profile");
      return nullptr;
   }
   return &mVelocityProfiles[index];
}

double* ReferenceInterfaces::exportReferenceProfileAcceleration(size_t index) {
   // Export direct pointer access to acceleration profile data for performance.
   // Used by hardware interface system for efficient data sharing.
   if (index >= mAccelerationProfiles.size()) {
      spdlog::error("The index is out of bounds for the acceleration profile");
      return nullptr;
   }
   return &mAccelerationProfiles[index];
}

}  // namespace trapezoidal_joint_controller
