#ifndef TRAPEZOIDAL_JOINT_CONTROLLER__JOINT_STATE_INTERFACES_HPP_
#define TRAPEZOIDAL_JOINT_CONTROLLER__JOINT_STATE_INTERFACES_HPP_

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <controller_utils/kinematic_interfaces/KinematicStateInterfaces.hpp>

namespace trapezoidal_joint_controller {

/**
 * @brief Type alias for joint state interface management
 * 
 * This provides a convenient alias for the KinematicStateInterfaces class,
 * which handles reading joint state information (position, velocity, acceleration)
 * from hardware interfaces.
 * 
 * The KinematicStateInterfaces class provides methods for:
 * - Registering required state interfaces during configuration
 * - claiming to actual hardware state interfaces during activation  
 * - Reading current joint positions and velocities in real-time
 * - Managing interface lifecycle (unclaim, unregister, etc.)
 * 
 * This abstraction allows the trapezoidal controller to work with different
 * types of joints and hardware interfaces in a consistent manner.
 */
using JointStateInterfaces = controller_utils::interfaces::KinematicStateInterfaces;

}  // namespace trapezoidal_joint_controller

#endif  // TRAPEZOIDAL_JOINT_CONTROLLER__JOINT_STATE_INTERFACES_HPP_
