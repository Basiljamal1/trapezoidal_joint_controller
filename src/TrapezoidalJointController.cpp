#include <algorithm>
#include <controller_interface/controller_interface_base.hpp>
#include <controller_utils/InterfaceRegistry.hpp>
#include <limits>
#include <optional>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <controller_utils/Utils.hpp>

#include "controller_interface/controller_interface_base.hpp"
#include "hardware_interface/handle.hpp"
#include "trapezoidal_joint_controller/TrapezoidalJointController.hpp"

#include "trapezoidal_joint_controller_parameters.hpp"

namespace trapezoidal_joint_controller {

TrapezoidalJointController::TrapezoidalJointController()
         : controller_interface::ChainableControllerInterface(),
           mLogger(rclcpp::get_logger("TrapezoidalJointController")) {
   // Constructor only initializes basic members. Actual configuration
   // happens during the lifecycle initialization phases.
}

void TrapezoidalJointController::declare_parameters() {
   // Create parameter listener if not already initialized
   if (mParamListener == nullptr) {
      mParamListener =
             std::make_shared<trapezoidal_joint_controller::ParamListener>(
                    get_node());
   }
   RCLCPP_INFO(mLogger, "Declaring parameters for controller node: %s",
               get_node()->get_name());
   
   // Load initial parameter values
   mParams = mParamListener->get_params();
}

controller_interface::CallbackReturn TrapezoidalJointController::on_init() {
   // Initialize controller parameters and validate basic configuration
   try {
      declare_parameters();
   } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(
             mLogger,
             "Parameter initialization failed with exception: " << e.what());
      return controller_interface::CallbackReturn::ERROR;
   }

   RCLCPP_DEBUG(mLogger, "Controller initialization completed successfully");
   return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrapezoidalJointController::on_configure(
       const rclcpp_lifecycle::State& /*previousState*/) {
   // Check if parameter listener has been configured
   if (!mParamListener) {
      RCLCPP_ERROR(mLogger, "The parameter listener is NOT set.");
      return controller_interface::CallbackReturn::ERROR;
   }

   // update the dynamic map parameters
   mParamListener->refresh_dynamic_parameters();

   // Read parameters
   auto ret = this->read_parameters();
   if (ret != controller_interface::CallbackReturn::SUCCESS) {
      return ret;
   }

   mPublisher = get_node()->create_publisher<ControllerStateMsg>(
          "~/controller_state", rclcpp::SystemDefaultsQoS());
   mStatePublisher = std::make_unique<StatePublisher>(mPublisher);

   RCLCPP_DEBUG(get_node()->get_logger(), "Configuration successful.");
   return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TrapezoidalJointController::read_parameters() {
   mParams = mParamListener->get_params();

   // Most of the validation is done by the parameters library.
   // Get the Dof and the joint information

   if (mParams.velocity_limits.size() != mParams.joints.size()) {
      RCLCPP_ERROR(mLogger,
                   "The size of the velocity limits does not match the number "
                   "of joints");
      return controller_interface::CallbackReturn::FAILURE;
   }

   if (mParams.acceleration_limits.size() != mParams.joints.size()) {
      RCLCPP_ERROR(mLogger,
                   "The size of the acceleration limits does not match the "
                   "number of joints");
      return controller_interface::CallbackReturn::FAILURE;
   }

   // Position limits are optional - if provided, they must match joint count
   if (!mParams.position_limits_min.empty() && 
       mParams.position_limits_min.size() != mParams.joints.size()) {
      RCLCPP_ERROR(mLogger,
                   "The size of position_limits_min does not match the "
                   "number of joints");
      return controller_interface::CallbackReturn::FAILURE;
   }

   if (!mParams.position_limits_max.empty() && 
       mParams.position_limits_max.size() != mParams.joints.size()) {
      RCLCPP_ERROR(mLogger,
                   "The size of position_limits_max does not match the "
                   "number of joints");
      return controller_interface::CallbackReturn::FAILURE;
   }

   if (!controller_utils::interfaces::containsInterfaceType(
              mParams.command_interfaces, mParams.control_mode)) {
      RCLCPP_ERROR_STREAM(mLogger,
                          "The control mode "
                                 << mParams.control_mode
                                 << " is not in the command interfaces");
      return controller_interface::CallbackReturn::FAILURE;
   }

   // Create unified CommandProxy (supports both position and velocity modes)
   if (mParams.control_mode ==
              controller_utils::interfaces::POSITION_INTERFACE_NAME ||
       mParams.control_mode ==
              controller_utils::interfaces::VELOCITY_INTERFACE_NAME) {
      mCommandProxy = std::make_unique<CommandProxy>(
             mParams.joints.size(), mParams.control_mode == "position");
   } else {
      RCLCPP_ERROR_STREAM(mLogger,
                          "The control mode "
                                 << mParams.control_mode
                                 << " is not supported. Only position and "
                                    "velocity modes are supported.");
      return controller_interface::CallbackReturn::FAILURE;
   }

   // Position and velocity are not optional and are required to function
   const controller_utils::interfaces::OptionalStateInterfaces
          stateInterfacesFlags{{false, false, true}, true, true};

   for (const std::string& joint : mParams.joints) {
      if (!mCommandProxy->registerCommandInterfaces(
                 joint, mParams.command_interfaces)) {
         RCLCPP_ERROR(mLogger,
                      "Failed to check command interfaces for joint %s",
                      joint.c_str());
         return controller_interface::CallbackReturn::FAILURE;
      }

      if (!mStateInterfaces.registerStateInterfaces(
                 joint, mParams.state_interfaces, stateInterfacesFlags)) {
         RCLCPP_ERROR(mLogger, "Failed to check state interfaces for joint %s",
                      joint.c_str());
         return controller_interface::CallbackReturn::FAILURE;
      }
   }

   // Initialize planner states for each joint
   const size_t n = mParams.joints.size();
   mJointPlanners.resize(n);

   for (size_t i = 0; i < n; ++i) {
      auto& planner = mJointPlanners[i];
      planner.status.controlInitialized = false;
      planner.status.coordinateMaxVelocity =
             static_cast<float>(mParams.velocity_limits[i]);

      // Default interpolation config
      // TODO(Basil): Expose those as config to enable configuration of the max error thresholds.
      planner.config.recaptureVelocityEps = 0.05f;
      planner.config.maxPositionSlip = trapezoidal_mp::kNoClamp;
      planner.config.maxVelocitySlip = trapezoidal_mp::kNoClamp;

      // Initialize per-joint position limits
      if (mParams.position_limits_min.size() > i && 
          std::isfinite(mParams.position_limits_min[i])) {
         planner.positionLimits.positionMin = static_cast<float>(mParams.position_limits_min[i]);
      } else {
         planner.positionLimits.positionMin = -std::numeric_limits<float>::infinity();
      }

      if (mParams.position_limits_max.size() > i && 
          std::isfinite(mParams.position_limits_max[i])) {
         planner.positionLimits.positionMax = static_cast<float>(mParams.position_limits_max[i]);
      } else {
         planner.positionLimits.positionMax = std::numeric_limits<float>::infinity();
      }

      // Initialize per-joint CommandData (these are the *limits*, not the desired velocity)
      planner.cmdData.position = std::numeric_limits<float>::quiet_NaN();
      planner.cmdData.velocity = std::numeric_limits<float>::quiet_NaN();  // unused in position mode
      planner.cmdData.accelerationProfile = static_cast<float>(mParams.acceleration_limits[i]);
      planner.cmdData.velocityProfile = static_cast<float>(mParams.velocity_limits[i]);

      // For linear joints - no wrapping, use position bounds  
      planner.cmdData.syntheticTheta = false;
      planner.cmdData.ignorePositionBounds = false;

      RCLCPP_DEBUG(mLogger, 
         "Joint[%zu] initialized: pos_min=%.3f, pos_max=%.3f, vel_limit=%.3f, acc_limit=%.3f",
         i, planner.positionLimits.positionMin, planner.positionLimits.positionMax,
         planner.cmdData.velocityProfile, planner.cmdData.accelerationProfile);
   }

   return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TrapezoidalJointController::state_interface_configuration() const {
   RCLCPP_DEBUG(mLogger, "Configuring state interface");
   controller_interface::InterfaceConfiguration conf;
   conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
   conf.names = mStateInterfaces.stateInterfaceConfiguration();

   std::stringstream ss;
   for (const auto& name : conf.names) {
      ss << name << " ";
   }

   RCLCPP_DEBUG_STREAM(mLogger, "Configured state interface. State interfaces: "
                                       << ss.str());
   return conf;
}

controller_interface::InterfaceConfiguration
TrapezoidalJointController::command_interface_configuration() const {
   RCLCPP_DEBUG(mLogger, "Configuring command interface");
   controller_interface::InterfaceConfiguration conf;
   conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

   conf.names = mCommandProxy->commandInterfaceConfiguration();

   std::stringstream ss;
   for (const auto& name : conf.names) {
      ss << name << " ";
   }

   RCLCPP_DEBUG(mLogger, "Configured command interfaces: %s", ss.str().c_str());
   return conf;
}

std::vector<hardware_interface::CommandInterface>
TrapezoidalJointController::on_export_reference_interfaces() {
   RCLCPP_DEBUG(mLogger, "Exporting the reference interfaces");
   std::vector<hardware_interface::CommandInterface> reference_interfaces;
   reference_interfaces_.resize(
          mParams.joints.size() *
          mParams.reference_interfaces
                 .size());  // Required by ros2 control, although not used.
   reference_interfaces.reserve(mParams.joints.size() *
                                mParams.reference_interfaces.size());

   reference_interfaces = mCommandProxy->referenceInterfaceConfiguration(
          get_node()->get_name(), mParams.reference_interfaces);

   RCLCPP_DEBUG(mLogger, "The reference interfaces have been exported");
   return reference_interfaces;
}

bool TrapezoidalJointController::on_set_chained_mode(bool /*chained_mode*/) {
   return true;
}

bool TrapezoidalJointController::claimInterfaces() {
   if (!mStateInterfaces.claimInterfaces(state_interfaces_)) {
      RCLCPP_ERROR(mLogger, "Could not claim the state interfaces");
      return false;
   }
   if (!mCommandProxy->claimInterfaces(command_interfaces_)) {
      RCLCPP_ERROR(mLogger, "Could not claim the command interfaces");
      return false;
   }
   return true;
}

void TrapezoidalJointController::updateAndPublishState(
       const std::optional<rclcpp::Time>& timeStamp) {
   rclcpp::Time publishTime;
   if (!timeStamp.has_value()) {
      publishTime = get_node()->now();
   } else {
      publishTime = timeStamp.value();
   }

   mControllerState.header.stamp = publishTime;
   const auto nanState = std::vector<double>(
          mParams.joints.size(), std::numeric_limits<double>::quiet_NaN());
   controller_utils::utils::assignState(
          publishTime, mControllerState.feedback,
          mStateInterfaces.getPositionValues(),
          mStateInterfaces.getVelocityValues(), nanState, nanState);
   controller_utils::utils::assignState(publishTime, mControllerState.output,
                                        mCommandProxy->getCommandPosition(),
                                        mCommandProxy->getCommandVelocity(),
                                        nanState, nanState);

   controller_utils::utils::assignState(
          publishTime, mControllerState.reference,
          mCommandProxy->getReferencePosition(),
          mCommandProxy->getReferenceVelocity(),
          mCommandProxy->getReferenceAcceleration(), nanState);

   mControllerState.error = controller_utils::utils::calculateError(
          publishTime, mControllerState.feedback, mControllerState.output);

   // Attempt to publish

   mStatePublisher->try_publish(mControllerState);
}

controller_interface::CallbackReturn TrapezoidalJointController::on_activate(
       const rclcpp_lifecycle::State& /*previous_state*/) {
   RCLCPP_DEBUG(mLogger, "Activating the trapezoidal joint controller");
   mParamListener->refresh_dynamic_parameters();
   mParams = mParamListener->get_params();

   // unclaim the command and state interfaces to ensure we do not extend them
   // upon chaining this controller
   unclaimAllInterfaces();

   if (!claimInterfaces()) {
      RCLCPP_ERROR(mLogger, "Could not claim the interfaces");
      return controller_interface::CallbackReturn::ERROR;
   }

   holdCurrentState();
   mCommandProxy->initializeReferences(
          mStateInterfaces.getPositionValues());

   // Initialize planner states from current position
   for (size_t i = 0; i < mParams.joints.size(); ++i) {
      const auto pos = mStateInterfaces.getPositionValues()[i];
      const auto vel = mStateInterfaces.getVelocityValues()[i];

      mJointPlanners[i].integratedPosition = pos;
      mJointPlanners[i].feedback.position = static_cast<float>(pos);
      mJointPlanners[i].feedback.velocity = static_cast<float>(vel);
      mJointPlanners[i].status.controlInitialized = false;
      mJointPlanners[i].status.controlPosition = static_cast<float>(pos);
      mJointPlanners[i].status.controlVelocity = static_cast<float>(vel);
   }

   updateAndPublishState(std::nullopt);
   RCLCPP_DEBUG(mLogger, "The controller has been activated");

   // This is necessary in activation sequence to ensure the robot starts at the
   // correct state. The joints expect a non zero velocity and acceleratio
   // profiles to start this mode.
   const std::vector<double> stopVector =
          std::vector<double>(mParams.joints.size(), 0.0);
   JointCommand bhInterface;
   bhInterface.jointNames = mParams.joints;
   bhInterface.jointPositions = mStateInterfaces.getPositionValues();
   bhInterface.jointVelocities = mParams.velocity_limits;
   bhInterface.jointAccelerations = mParams.acceleration_limits;
   this->setTargetCommand(bhInterface);
   return controller_interface::CallbackReturn::SUCCESS;
}

void TrapezoidalJointController::holdCurrentState() {
   const std::vector<double> currentPosition =
          mStateInterfaces.getPositionValues();

   mCommandProxy->holdState(currentPosition);
   JointCommand bhInterface;
   bhInterface.jointNames = mParams.joints;
   bhInterface.jointPositions = mStateInterfaces.getPositionValues();
   bhInterface.jointVelocities = mParams.velocity_limits;
   bhInterface.jointAccelerations = mParams.acceleration_limits;
   mCommandBuffer.setFromNonRT(bhInterface);
}

controller_interface::return_type
TrapezoidalJointController::update_reference_from_subscribers(
       const rclcpp::Time&, const rclcpp::Duration&) {
   const auto opt = mCommandBuffer.getFromRT();
   if (!opt.has_value() || opt->jointNames.empty())
      return controller_interface::return_type::OK;
   const auto& msg = *opt;

   if (!isCommandValid(msg)) return controller_interface::return_type::OK;

   // Write through to your proxy (as you already do)
   if (mParams.control_mode == "position") {
      mCommandProxy->setReferencePosition(*msg.jointPositions);
      mCommandProxy->setReferenceVelocity(*msg.jointVelocities);
      mCommandProxy->setReferenceAcceleration(*msg.jointAccelerations);
   } else {  // velocity mode
      mCommandProxy->setReferenceVelocity(*msg.jointVelocities);
      mCommandProxy->setReferenceAcceleration(*msg.jointAccelerations);
   }

   return controller_interface::return_type::OK;
}

controller_interface::return_type
TrapezoidalJointController::update_and_write_commands(
       const rclcpp::Time& time, const rclcpp::Duration& period) {
   // Refresh params if needed
   if (mParamListener->is_old(mParams)) {
      mParams = mParamListener->get_params();
   }

   // Read state once
   const auto posNow = mStateInterfaces.getPositionValues();
   const auto velNow = mStateInterfaces.getVelocityValues();

   const double dt = period.seconds();
   const float rate_hz = static_cast<float>(mParams.control_rate);
   if (!(rate_hz > 0.0f)) {
      RCLCPP_ERROR(mLogger, "control_rate must be > 0");
      return controller_interface::return_type::ERROR;
   }

   // Get the latest references from the proxy
   const auto refPos = mCommandProxy->getReferencePosition();
   const auto refVel = mCommandProxy->getReferenceVelocity();  
   const auto refAcc = mCommandProxy->getReferenceAcceleration();

   std::vector<double> velocityCommands(mParams.joints.size(), 0.0);
   std::vector<double> positionCommands(mParams.joints.size(), 0.0);

   for (size_t i = 0; i < mParams.joints.size(); ++i) {
      auto& planner = mJointPlanners[i];

      // Update feedback to planner - use ACTUAL measured position (no unwrapping!)
      planner.feedback.position = static_cast<float>(posNow[i]);
      planner.feedback.velocity = static_cast<float>(velNow[i]);
      planner.status.velocity = static_cast<float>(velNow[i]);

      // Setup command data based on control mode
      float desiredVelocity = std::numeric_limits<float>::quiet_NaN();

      if (mParams.control_mode == "position") {
         // Simple direct position targeting - like the test cases
         planner.cmdData.position = static_cast<float>(refPos[i]);
         planner.cmdData.velocity = std::numeric_limits<float>::quiet_NaN(); // stationary goal
         planner.cmdData.velocityProfile = static_cast<float>(mParams.velocity_limits[i]);
         planner.cmdData.accelerationProfile = static_cast<float>(mParams.acceleration_limits[i]);
         // desiredVelocity stays NaN (treated as 0)

      } else { // velocity mode
         planner.cmdData.position = std::numeric_limits<float>::quiet_NaN();
         planner.cmdData.velocity = static_cast<float>(refVel[i]);
         planner.cmdData.velocityProfile = static_cast<float>(mParams.velocity_limits[i]);
         planner.cmdData.accelerationProfile = static_cast<float>(mParams.acceleration_limits[i]);
         desiredVelocity = static_cast<float>(refVel[i]);
      }

      // Run trapezoidal planner - this is the core algorithm
      const float v_cmd = trapezoidal_mp::UpdateCommand(
         planner.status,
         planner.config,
         planner.positionLimits,
         planner.feedback,
         rate_hz,
         planner.cmdData,
         desiredVelocity);

      // Store outputs
      velocityCommands[i] = static_cast<double>(v_cmd);
      
      // For position mode, use planner's internal control position
      // For velocity mode, integrate velocity
      if (mParams.control_mode == "position") {
         positionCommands[i] = static_cast<double>(planner.status.controlPosition);
      } else {
         planner.integratedPosition += static_cast<double>(v_cmd) * dt;
         positionCommands[i] = planner.integratedPosition;
      }

      // Debug output for joint 1
      if (i == 0) {
         RCLCPP_DEBUG(mLogger,
            "Joint[%zu] DEBUG - Output velocity: %.6f, Position command: %.6f, Reference position: %.6f, Reference velocity: %.6f", 
            i, v_cmd, positionCommands[i], refPos[i], refVel[i]);
         
         RCLCPP_DEBUG(mLogger,
            "Joint[%zu] DEBUG - Planner inputs: cmd_position=%.6f, cmd_velocity=%.6f, cmd_accel_profile=%.6f, cmd_vel_profile=%.6f", 
            i, planner.cmdData.position, planner.cmdData.velocity, planner.cmdData.accelerationProfile, planner.cmdData.velocityProfile);

         RCLCPP_DEBUG(mLogger,
            "Joint[%zu] DEBUG - Feedback: fb_position=%.6f, fb_velocity=%.6f",
            i, planner.feedback.position, planner.feedback.velocity);

         RCLCPP_DEBUG(mLogger,
            "Joint[%zu] DEBUG - Planner status: ctrl_position=%.6f, ctrl_velocity=%.6f, traj_finished=%s",
            i, planner.status.controlPosition, planner.status.controlVelocity,
            planner.status.trajectoryFinished ? "true" : "false");
      }
   }

   // Send commands to hardware based on control mode
   if (mParams.control_mode == "position") {
      mCommandProxy->setCommandPosition(positionCommands);
   } else {
      mCommandProxy->setCommandVelocity(velocityCommands);
   }

   updateAndPublishState(time);
   return controller_interface::return_type::OK;
}

bool TrapezoidalJointController::contains_interface_type(
       const std::vector<std::string>& interface_type_list,
       const std::string& interface_type) {
   return std::find(interface_type_list.begin(), interface_type_list.end(),
                    interface_type) != interface_type_list.end();
}

void TrapezoidalJointController::unclaimAllInterfaces() {
   mCommandProxy->unclaimInterfaces();
   mStateInterfaces.unclaimStateInterfaces();
}

void TrapezoidalJointController::unregisterInterfaces() {
   mCommandProxy->unregisterCommandInterfaces();
   mStateInterfaces.unregisterStateInterfaces();
}

controller_interface::CallbackReturn TrapezoidalJointController::on_deactivate(
       const rclcpp_lifecycle::State& /*previous_state*/) {
   holdCurrentState();
   unclaimAllInterfaces();
   return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrapezoidalJointController::on_cleanup(
       const rclcpp_lifecycle::State& /*previous_state*/) {
   unregisterInterfaces();
   return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TrapezoidalJointController::on_error(
       const rclcpp_lifecycle::State& /*previous_state*/) {
   holdCurrentState();
   unregisterInterfaces();
   return controller_interface::CallbackReturn::SUCCESS;
}

bool TrapezoidalJointController::isCommandValid(
       const JointCommand& latestCommand) const {
   if (latestCommand.jointNames.size() != mParams.joints.size()) {
      RCLCPP_ERROR(mLogger,
                   "The number of joint positions does not match the number of "
                   "joints");
      return false;
   }

   if (mParams.control_mode == "position") {
      if (!latestCommand.jointPositions.has_value() ||
          latestCommand.jointPositions->size() !=
                 latestCommand.jointNames.size() ||
          !latestCommand.jointVelocities.has_value() ||
          latestCommand.jointVelocities->size() !=
                 latestCommand.jointNames.size() ||
          !latestCommand.jointAccelerations.has_value() ||
          latestCommand.jointAccelerations->size() !=
                 latestCommand.jointNames.size()) {
         RCLCPP_ERROR(mLogger,
                      "In position mode, positions, velocities, and "
                      "accelerations must all be "
                      "available and of the correct size");
         return false;
      }
   } else if (mParams.control_mode == "velocity") {
      if (!latestCommand.jointVelocities.has_value() ||
          latestCommand.jointVelocities->size() !=
                 latestCommand.jointNames.size() ||
          !latestCommand.jointAccelerations.has_value() ||
          latestCommand.jointAccelerations->size() !=
                 latestCommand.jointNames.size()) {
         RCLCPP_ERROR(mLogger,
                      "In velocity mode, velocities and accelerations must be "
                      "available and of "
                      "the correct size");
         return false;
      }
   }

   return true;
}

bool TrapezoidalJointController::setTargetCommand(const JointCommand& command) {
   if (is_in_chained_mode()) {
      RCLCPP_ERROR(mLogger, "The controller is in chained mode");
      return false;
   }

   if (!this->isCommandValid(command)) {
      RCLCPP_ERROR(mLogger, "The command is invalid");
      return false;
   }

   mCommandBuffer.setFromNonRT((command));
   return true;
}

}  // namespace trapezoidal_joint_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(trapezoidal_joint_controller::TrapezoidalJointController,
                       controller_interface::ChainableControllerInterface)
