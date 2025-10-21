#ifndef TRAPEZOIDAL_JOINT_CONTROLLER__COMMAND_PROXY_HPP_
#define TRAPEZOIDAL_JOINT_CONTROLLER__COMMAND_PROXY_HPP_

#include <vector>
#include <string>
#include <memory>

#include "trapezoidal_joint_controller/read_write_interfaces/JointCommandInterfaces.hpp"
#include "trapezoidal_joint_controller/read_write_interfaces/ReferenceInterfaces.hpp"

namespace trapezoidal_joint_controller {

/**
 * @class CommandProxy
 * @brief Unified interface for managing command and reference interfaces
 *
 * This class abstracts the complexity of different control modes (position/velocity)
 * and provides a consistent API for the main controller. It handles:
 * - Hardware interface registration and management
 * - Reference interface setup for controller chaining
 * - Command validation and execution
 * - State tracking for monitoring and debugging
 * 
 * The proxy automatically adapts its behavior based on the control mode specified
 * during construction, ensuring type-safe operations for each mode.
 */
class CommandProxy {
   public:
   /**
    * @brief Construct a CommandProxy for a specific control configuration
    * @param dof Number of degrees of freedom (joints) to manage
    * @param position_mode true for position control, false for velocity control
    * 
    * The control mode determines which interfaces are registered and how
    * commands are processed and validated.
    */
   CommandProxy(const size_t dof, bool position_mode);

   /** @brief Default destructor */
   ~CommandProxy() = default;

   // Interface registration and management methods
   
   /**
    * @brief Register command interfaces for a joint component
    * @param componentPrefix Name prefix for the joint/component  
    * @param interfaceConfig List of interface types to register
    * @return true if registration successful, false otherwise
    * 
    * Registers the required command interfaces (position/velocity) based on
    * the control mode and interface configuration.
    */
   bool registerCommandInterfaces(
          const std::string& componentPrefix,
          const std::vector<std::string>& interfaceConfig);
   
   /**
    * @brief Unregister all command interfaces
    * 
    * Removes command interface registrations. Called during cleanup.
    */
   void unregisterCommandInterfaces();

   /**
    * @brief Configure reference interfaces for controller chaining
    * @param nodeName Name of the controller node
    * @param availableInterfaces List of available reference interface types
    * @return Vector of configured reference interfaces
    * 
    * Creates reference interfaces that upstream controllers can write to
    * when this controller is used in a chained configuration.
    */
   std::vector<hardware_interface::CommandInterface>
   referenceInterfaceConfiguration(
          const std::string& nodeName,
          const std::vector<std::string>& availableInterfaces);

   /**
    * @brief Get the list of required command interface names
    * @return Vector of command interface names needed by this controller
    */
   std::vector<std::string> commandInterfaceConfiguration() const;

   /**
    * @brief claim registered interfaces to actual hardware interfaces
    * @param unorderedInterfaces Available hardware command interfaces
    * @return true if claiming successful, false otherwise
    * 
    * Connects the registered interfaces to actual hardware interfaces
    * provided by the controller manager during activation.
    */
   bool claimInterfaces(std::vector<hardware_interface::LoanedCommandInterface>&
                              unorderedInterfaces);

   /**
    * @brief unclaim all interface registrations
    * 
    * Releases interface registrations without unregistering them.
    * Called during deactivation.
    */
   void unclaimInterfaces();

   // Reference management methods (for controller chaining)
   
   /**
    * @brief Initialize reference values with current joint positions
    * @param currentPosition Current joint positions to use as initial references
    * 
    * Sets up initial reference values when the controller becomes active.
    * Only sets position references in position mode.
    */
   void initializeReferences(const std::vector<double>& currentPosition);

   /**
    * @brief Set position references (position mode only)
    * @param position Target positions for each joint
    * @return true if successful, false if not in position mode
    */
   bool setReferencePosition(const std::vector<double>& position);
   
   /**
    * @brief Set velocity references (limits in position mode, targets in velocity mode)
    * @param velocity Velocity values for each joint
    * @return true if successful, false otherwise
    */
   bool setReferenceVelocity(const std::vector<double>& velocity);
   
   /**
    * @brief Set acceleration references (limits for motion planning)
    * @param acceleration Acceleration limits for each joint  
    * @return true if successful, false otherwise
    */
   bool setReferenceAcceleration(const std::vector<double>& acceleration);

   // Reference access methods (for reading current references)
   
   /**
    * @brief Get current position references
    * @return Current position reference values
    */
   const std::vector<double> getReferencePosition() const;
   
   /**
    * @brief Get current velocity references  
    * @return Current velocity reference values
    */
   const std::vector<double> getReferenceVelocity() const;
   
   /**
    * @brief Get current acceleration references
    * @return Current acceleration reference values  
    */
   const std::vector<double> getReferenceAcceleration() const;

   /**
    * @brief Set controller to hold current state
    * @param currentState Current joint positions to hold
    * 
    * Configures the controller to maintain the current position with
    * zero velocity and acceleration references.
    */
   void holdState(const std::vector<double>& currentState);

   // Hardware command methods (for writing to hardware interfaces)
   
   /**
    * @brief Send position commands to hardware interfaces
    * @param position Position commands for each joint
    * @return true if successful, false otherwise
    * 
    * Only available in position control mode. Writes position commands
    * directly to the hardware command interfaces.
    */
   bool setCommandPosition(const std::vector<double>& position);
   
   /**
    * @brief Send velocity commands to hardware interfaces  
    * @param velocity Velocity commands for each joint
    * @return true if successful, false otherwise
    * 
    * Only available in velocity control mode. Writes velocity commands
    * directly to the hardware command interfaces.
    */
   bool setCommandVelocity(const std::vector<double>& velocity);
   
   // Command access methods (for state publishing and monitoring)
   
   /**
    * @brief Get current position commands being sent to hardware
    * @return Current position command values
    */
   const std::vector<double> getCommandPosition() const;
   
   /**
    * @brief Get current velocity commands being sent to hardware
    * @return Current velocity command values
    */
   const std::vector<double> getCommandVelocity() const;

   private:
   /** @brief Number of degrees of freedom managed by this proxy */
   size_t mDof;
   
   /** @brief true if operating in position mode, false for velocity mode */
   bool mPositionMode;

   /** @brief Handler for joint command interfaces (position/velocity output) */
   JointCommandInterfaces mCommandInterfaces;
   
   /** @brief Handler for reference interfaces (controller chaining input) */
   ReferenceInterfaces mReferenceCommands;
};

}  // namespace trapezoidal_joint_controller

#endif  // TRAPEZOIDAL_JOINT_CONTROLLER__COMMAND_PROXY_HPP_