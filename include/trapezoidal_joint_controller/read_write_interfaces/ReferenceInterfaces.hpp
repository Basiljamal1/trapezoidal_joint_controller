
#ifndef TRAPEZOIDAL_JOINT_CONTROLLER__REFERENCE_INTERFACES_HPP_
#define TRAPEZOIDAL_JOINT_CONTROLLER__REFERENCE_INTERFACES_HPP_

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <controller_utils/kinematic_interfaces/JointReferenceInterface.hpp>

namespace trapezoidal_joint_controller {

/**
 * @class ReferenceInterfaces
 * @brief Manages reference interfaces for controller chaining
 * 
 * This class extends the base JointReferenceInterface to provide additional
 * reference types specific to trapezoidal motion planning. It handles:
 * - Position references (inherited from base class)
 * - Velocity profile limits for motion planning  
 * - Acceleration profile limits for motion planning
 * 
 * These interfaces allow upstream controllers to command this controller
 * in a chained configuration, providing smooth motion coordination between
 * different control layers.
 */
class ReferenceInterfaces
         : public controller_utils::interfaces::JointReferenceInterface {
   public:
   /**
    * @brief Construct reference interfaces for specified degrees of freedom
    * @param dof Number of joints/degrees of freedom to manage
    * 
    * Initializes storage for position, velocity profile, and acceleration
    * profile references for the specified number of joints.
    */
   explicit ReferenceInterfaces(
          size_t dof = controller_utils::interfaces::DEFAULT_DOF);

   /**
    * @brief Set velocity profile limits for all joints
    * @param values Velocity profile limits for each joint
    * 
    * These limits constrain the maximum velocities used by the trapezoidal
    * motion planner when generating trajectories.
    */
   void setVelocityProfile(const std::vector<double>& values);
   
   /**
    * @brief Set acceleration profile limits for all joints  
    * @param values Acceleration profile limits for each joint
    * 
    * These limits constrain the maximum accelerations used by the trapezoidal
    * motion planner when generating trajectories.
    */
   void setAccelerationProfile(const std::vector<double>& values);

   /**
    * @brief Get current velocity profile limits
    * @return Reference to velocity profile limit values
    */
   const std::vector<double>& getVelocityProfile() const;
   
   /**
    * @brief Get current acceleration profile limits
    * @return Reference to acceleration profile limit values  
    */
   const std::vector<double>& getAccelerationProfile() const;

   /**
    * @brief Export velocity profile reference for a specific joint
    * @param index Joint index
    * @return Pointer to velocity profile value for external access
    * 
    * Allows upstream controllers to write directly to velocity profile
    * references through hardware interface mechanisms.
    */
   double* exportReferenceProfileVelocity(size_t index);
   
   /**
    * @brief Export acceleration profile reference for a specific joint
    * @param index Joint index  
    * @return Pointer to acceleration profile value for external access
    * 
    * Allows upstream controllers to write directly to acceleration profile
    * references through hardware interface mechanisms.
    */
   double* exportReferenceProfileAcceleration(size_t index);

   private:
   /** @brief Velocity profile limits for each joint */
   std::vector<double> mVelocityProfiles;
   
   /** @brief Acceleration profile limits for each joint */
   std::vector<double> mAccelerationProfiles;
};

}  // namespace trapezoidal_joint_controller

#endif  // TRAPEZOIDAL_JOINT_CONTROLLER__REFERENCE_INTERFACES_HPP_
