#ifndef TRAPEZOIDAL_JOINT_CONTROLLER__JOINT_COMMAND_INTERFACES_HPP_
#define TRAPEZOIDAL_JOINT_CONTROLLER__JOINT_COMMAND_INTERFACES_HPP_

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <controller_utils/kinematic_interfaces/JointInterfaces.hpp>

namespace trapezoidal_joint_controller {

/**
 * @brief Type alias for joint command interface management
 * 
 * This provides a convenient alias for managing hardware command interfaces
 * used to send position or velocity commands to joints.
 * 
 * The JointInterfaces template specialized with LoanedCommandInterface provides:
 * - Registration of command interfaces during configuration
 * - claiming to actual hardware command interfaces during activation
 * - Writing position/velocity commands to hardware in real-time
 * - Interface lifecycle management (unclaim, unregister, etc.)
 * 
 * This abstraction allows the controller to work with different command
 * interface types (position, velocity, effort) in a unified manner while
 * maintaining type safety and real-time performance.
 */
using JointCommandInterfaces = controller_utils::interfaces::JointInterfaces<
                  hardware_interface::LoanedCommandInterface>;

}  // namespace trapezoidal_joint_controller

#endif  // TRAPEZOIDAL_JOINT_CONTROLLER__JOINT_COMMAND_INTERFACES_HPP_
