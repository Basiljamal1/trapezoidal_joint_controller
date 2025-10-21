#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace trapezoidal_joint_controller {

/**
 * @struct JointCommand
 * @brief Command message structure for multi-joint control
 * 
 * This structure represents a command that can be sent to the trapezoidal joint controller.
 * It supports different control modes (position, velocity, torque) with optional fields
 * depending on the active control mode.
 * 
 * Usage by Control Mode:
 * - Position Mode: jointPositions (targets), jointVelocities (limits), jointAccelerations (limits)
 * - Velocity Mode: jointVelocities (targets), jointAccelerations (limits) 
 * - Torque Mode: jointTorques (targets)
 * 
 * All vector fields must have the same size as jointNames when present.
 */
struct JointCommand {
   /** @brief Timestamp when this command was created */
   std::chrono::nanoseconds timestamp;

   /** @brief Names of the joints this command applies to */
   std::vector<std::string> jointNames;

   /** @brief Target positions for position mode, or empty for other modes */
   std::optional<std::vector<double>> jointPositions;

   /** 
    * @brief Target velocities for velocity mode, or velocity limits for position mode
    * Units: rad/s for revolute joints, m/s for prismatic joints
    */
   std::optional<std::vector<double>> jointVelocities;

   /** 
    * @brief Acceleration limits for all modes
    * Units: rad/s² for revolute joints, m/s² for prismatic joints  
    */
   std::optional<std::vector<double>> jointAccelerations;

};

}  // namespace trapezoidal_joint_controller
