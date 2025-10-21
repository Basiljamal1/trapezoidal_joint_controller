#include "trapezoidal_joint_controller/CommandProxy.hpp"
#include <controller_utils/InterfaceRegistry.hpp>
#include <algorithm>
#include <limits>

namespace trapezoidal_joint_controller {

CommandProxy::CommandProxy(const size_t dof, bool position_mode)
         : mDof(dof), mPositionMode(position_mode) {
   // Store configuration for later interface setup. The actual interface
   // registration happens during the controller configuration phase.
}

bool CommandProxy::registerCommandInterfaces(
       const std::string& componentPrefix,
       const std::vector<std::string>& interfaceConfig) {
   // Register command interfaces with the joint command handler.
   // The third parameter {true, true, true} enables position, velocity, and effort interfaces.
   return mCommandInterfaces.registerBaseInterfaces(
          componentPrefix, interfaceConfig, {true, true, true});
}

std::vector<hardware_interface::CommandInterface>
CommandProxy::referenceInterfaceConfiguration(
       const std::string& nodeName,
       const std::vector<std::string>& availableInterfaces) {
   std::vector<hardware_interface::CommandInterface> referenceInterfacesConfig;
   mReferenceCommands = ReferenceInterfaces(mDof);
   const auto componentNames =
          mPositionMode ? mCommandInterfaces.getPositionComponentNames()
                        : mCommandInterfaces.getVelocityComponentNames();

   for (size_t i = 0; i < mDof; i++) {
      const auto jointName = componentNames[i];

      // TODO(Basil): Expose the feedforward velocity for position mode.
      if (mPositionMode &&
          std::find(availableInterfaces.begin(), availableInterfaces.end(),
                    hardware_interface::HW_IF_POSITION) !=
                 availableInterfaces.end()) {
         referenceInterfacesConfig.push_back(
                hardware_interface::CommandInterface(
                       nodeName,
                       jointName + "/" + hardware_interface::HW_IF_POSITION,
                       mReferenceCommands.exportReferencePosition(i)));
      }

      if (std::find(availableInterfaces.begin(), availableInterfaces.end(),
                    hardware_interface::HW_IF_VELOCITY) !=
          availableInterfaces.end()) {
         const std::string referenceInterfaceName =
                jointName + "/" + hardware_interface::HW_IF_VELOCITY;
         referenceInterfacesConfig.push_back(
                hardware_interface::CommandInterface(
                       nodeName, referenceInterfaceName,
                       mReferenceCommands.exportReferenceProfileVelocity(i)));
      }

      if (std::find(availableInterfaces.begin(), availableInterfaces.end(),
                    hardware_interface::HW_IF_ACCELERATION) !=
          availableInterfaces.end()) {
         referenceInterfacesConfig.push_back(
                hardware_interface::CommandInterface(
                       nodeName,
                       jointName + "/" + hardware_interface::HW_IF_ACCELERATION,
                       mReferenceCommands.exportReferenceProfileAcceleration(
                              i)));
      }
   }
   return referenceInterfacesConfig;
}

std::vector<std::string> CommandProxy::commandInterfaceConfiguration() const {
   return mCommandInterfaces.baseInterfaceConfiguration();
}

bool CommandProxy::claimInterfaces(
       std::vector<hardware_interface::LoanedCommandInterface>&
              unorderedInterfaces) {
   return mCommandInterfaces.claimInterfaces(unorderedInterfaces);
}

void CommandProxy::unregisterCommandInterfaces() {
   // Unregister all previously registered command interfaces.
   // This is called during controller deactivation/cleanup.
   mCommandInterfaces.unregisterBaseInterfaces();
}

void CommandProxy::unclaimInterfaces() {
   // unclaim the complete interface registry, removing all registered interfaces.
   // Used for complete controller reset scenarios.
   mCommandInterfaces.unclaimBaseInterfaces();
}

void CommandProxy::initializeReferences(
       const std::vector<double>& currentState) {
   // Initialize reference commands based on the current joint state.
   // In position mode, start with current positions as reference.
   if (mPositionMode) mReferenceCommands.setPosition(currentState);
   
   // Always initialize velocity and acceleration profiles to zero for smooth startup.
   mReferenceCommands.setVelocityProfile(std::vector<double>(mDof, 0.0));
   mReferenceCommands.setAccelerationProfile(std::vector<double>(mDof, 0.0));
}

bool CommandProxy::setReferencePosition(const std::vector<double>& position) {
   // Position references can only be set in position control mode.
   // In velocity mode, position is not directly controlled.
   if (!mPositionMode) {
      return false;  // Cannot set position reference in velocity mode
   }
   mReferenceCommands.setPosition(position);
   return true;
}

bool CommandProxy::setReferenceVelocity(const std::vector<double>& velocity) {
   // Velocity references are valid in both position and velocity modes.
   // In position mode, this affects the velocity profile of the trajectory.
   // In velocity mode, this is the direct velocity command.
   mReferenceCommands.setVelocityProfile(velocity);
   return true;
}

bool CommandProxy::setReferenceAcceleration(
       const std::vector<double>& acceleration) {
   // Acceleration references define the acceleration limits or profiles.
   // Used by the trapezoidal motion planner for trajectory generation.
   mReferenceCommands.setAccelerationProfile(acceleration);
   return true;
}

const std::vector<double> CommandProxy::getReferencePosition() const {
   // Position references are only meaningful in position control mode.
   // Return NaN values in velocity mode to indicate invalid data.
   if (!mPositionMode) {
      return std::vector<double>(
             mDof,
             std::numeric_limits<double>::
                    quiet_NaN());  // No position reference in velocity mode
   }
   return mReferenceCommands.getReferencePositions();
}

const std::vector<double> CommandProxy::getReferenceVelocity() const {
   // Velocity references are valid in both control modes.
   return mReferenceCommands.getVelocityProfile();
}

const std::vector<double> CommandProxy::getReferenceAcceleration() const {
   // Acceleration references define trajectory limits and profiles.
   return mReferenceCommands.getAccelerationProfile();
}

const std::vector<double> CommandProxy::getCommandPosition() const {
   // Return the current position command values being sent to hardware.
   return mCommandInterfaces.getPositionValues();
}

const std::vector<double> CommandProxy::getCommandVelocity() const {
   // Return the current velocity command values being sent to hardware.
   return mCommandInterfaces.getVelocityValues();
}

void CommandProxy::holdState(const std::vector<double>& currentState) {
   // Configure the controller to hold the current joint state.
   // In position mode, set both reference and command to current positions.
   if (mPositionMode) mReferenceCommands.setPosition(currentState);
   if (mPositionMode) mCommandInterfaces.setPositionValues(currentState);

   // Always set velocity and acceleration profiles to zero for holding.
   // This ensures the joints remain stationary.
   mReferenceCommands.setVelocityProfile(std::vector<double>(mDof, 0.0));
   mCommandInterfaces.setVelocityValues(
          std::vector<double>(mDof, 0.0));

   mReferenceCommands.setAccelerationProfile(std::vector<double>(mDof, 0.0));;
}

bool CommandProxy::setCommandPosition(const std::vector<double>& position) {
   // Set position command values to be sent to hardware.
   // Used by the trapezoidal motion planner output.
   return mCommandInterfaces.setPositionValues(position);
}

bool CommandProxy::setCommandVelocity(const std::vector<double>& velocity) {
   // Set velocity command values to be sent to hardware.
   // Used by the trapezoidal motion planner output.
   return mCommandInterfaces.setVelocityValues(velocity);
}

}  // namespace trapezoidal_joint_controller