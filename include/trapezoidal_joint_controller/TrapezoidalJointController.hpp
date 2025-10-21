#ifndef TRAPEZOIDAL_JOINT_CONTROLLER_HPP_
#define TRAPEZOIDAL_JOINT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <trapezoidal_joint_controller_parameters.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <realtime_utils/RealTimeBoxBestEffort.hpp>

#include <trapezoidal_motion_planner/TrapezoidalMotionPlanner.hpp>

#include "trapezoidal_joint_controller/CommandProxy.hpp"
#include "trapezoidal_joint_controller/read_write_interfaces/JointStateInterfaces.hpp"
#include "trapezoidal_joint_controller/JointCommand.hpp"

namespace trapezoidal_joint_controller {

/**
 * @struct JointPlannerState
 * @brief Complete state information for a single joint's trapezoidal motion planner
 *
 * This structure encapsulates all the data needed to manage trapezoidal motion planning
 * for an individual joint, including planner status, configuration, feedback, and limits.
 */
struct JointPlannerState {
   /** @brief Current execution state of the trapezoidal motion planner */
   trapezoidal_mp::CommandStatus status;
   
   /** @brief Configuration parameters for motion interpolation and behavior */
   trapezoidal_mp::InterpolationConfig config;
   
   /** @brief Real-time feedback data from the joint (position, velocity) */
   trapezoidal_mp::CoordinateFeedback feedback;
   
   /** @brief Command data including target position/velocity and motion limits */
   trapezoidal_mp::CommandData cmdData;
   
   /** @brief Position limits specific to this joint (min/max bounds) */
   trapezoidal_mp::PositionLimits positionLimits;
   
   /** @brief Position calculated by integrating velocity commands over time */
   double integratedPosition;
};

/**
 * @class TrapezoidalJointController
 * @brief ROS 2 control framework controller implementing trapezoidal motion profiles
 *
 * This controller provides smooth, trapezoidal motion planning for multiple joints
 * simultaneously. It supports both position and velocity control modes with
 * configurable acceleration and velocity limits. Each joint has its own independent
 * trapezoidal motion planner to ensure smooth trajectories while respecting
 * individual joint constraints.
 *
 * Key Features:
 * - Independent trapezoidal motion planning per joint
 * - Support for position and velocity control modes
 * - Real-time safe operation with configurable motion limits
 * - Controller chaining support through reference interfaces
 * - Comprehensive state monitoring and debugging capabilities
 *
 * The controller inherits from ChainableControllerInterface to support
 * controller composition and cascading control architectures.
 */
class TrapezoidalJointController
         : public controller_interface::ChainableControllerInterface {
   public:
   /**
    * @brief Default constructor for TrapezoidalJointController
    * 
    * Initializes the controller with default settings and sets up the logger.
    * The actual configuration happens during the lifecycle initialization phases.
    */
   TrapezoidalJointController();

   /**
    * @brief Default destructor
    */
   ~TrapezoidalJointController() = default;

   /**
    * @brief Initialize the controller during the init lifecycle phase
    * @return SUCCESS if initialization is successful, ERROR otherwise
    * 
    * This method declares and reads the initial parameters. It's called once
    * during controller loading and should not perform any hardware-related
    * operations.
    */
   controller_interface::CallbackReturn on_init() override;

   /**
    * @brief Configure the controller during the configure lifecycle phase
    * @param previous_state The previous lifecycle state
    * @return SUCCESS if configuration is successful, ERROR otherwise
    * 
    * Validates parameters, creates publishers, and sets up internal data structures.
    * No hardware interfaces are accessed during this phase.
    */
   controller_interface::CallbackReturn on_configure(
          const rclcpp_lifecycle::State& previous_state) override;

   /**
    * @brief Activate the controller during the activate lifecycle phase
    * @param previous_state The previous lifecycle state  
    * @return SUCCESS if activation is successful, ERROR otherwise
    * 
    * claims hardware interfaces, initializes joint planners with current state,
    * and prepares for real-time operation.
    */
   controller_interface::CallbackReturn on_activate(
          const rclcpp_lifecycle::State& previous_state) override;

   /**
    * @brief Specify which command interfaces this controller requires
    * @return Configuration describing required command interfaces
    * 
    * Returns the list of command interfaces (e.g., position, velocity) that
    * this controller needs to claim from the hardware.
    */
   controller_interface::InterfaceConfiguration
   command_interface_configuration() const override;

   /**
    * @brief Specify which state interfaces this controller requires  
    * @return Configuration describing required state interfaces
    * 
    * Returns the list of state interfaces (e.g., position, velocity feedback)
    * that this controller needs to read from the hardware.
    */
   controller_interface::InterfaceConfiguration state_interface_configuration()
          const override;

   /**
    * @brief Deactivate the controller during the deactivate lifecycle phase
    * @param previous_state The previous lifecycle state
    * @return SUCCESS if deactivation is successful
    * 
    * Safely stops motion, holds current state, and releases hardware interfaces.
    */
   controller_interface::CallbackReturn on_deactivate(
          const rclcpp_lifecycle::State& previous_state) override;

   /**
    * @brief Clean up the controller during the cleanup lifecycle phase  
    * @param previous_state The previous lifecycle state
    * @return SUCCESS if cleanup is successful
    * 
    * Releases all resources and resets the controller to its initial state.
    */
   controller_interface::CallbackReturn on_cleanup(
          const rclcpp_lifecycle::State& previous_state) override;

   /**
    * @brief Handle errors during the error lifecycle phase
    * @param previous_state The previous lifecycle state
    * @return SUCCESS if error handling is successful
    * 
    * Attempts to safely stop motion and clean up resources when an error occurs.
    */
   controller_interface::CallbackReturn on_error(
          const rclcpp_lifecycle::State& previous_state) override;

   /**
    * @brief Set a target command for the joints (non-real-time safe)
    * @param command The target command containing joint positions/velocities
    * @return true if command is valid and accepted, false otherwise
    * 
    * This method validates and buffers the command for consumption by the
    * real-time control loop. It should only be called from non-real-time contexts.
    */
   bool setTargetCommand(const JointCommand& command);

   /**
    * @brief Command the controller to hold the current joint state
    * 
    * Sets the current joint positions as targets and zeros velocities/accelerations.
    * This is useful for stopping motion and maintaining the current position.
    */
   void holdCurrentState();

   protected:
   /**
    * @brief Export reference interfaces for controller chaining
    * @return Vector of reference interfaces that other controllers can write to
    * 
    * Creates reference interfaces (position, velocity, acceleration) that upstream
    * controllers can use to command this controller in a chained configuration.
    */
   std::vector<hardware_interface::CommandInterface>
   on_export_reference_interfaces() override;

   /**
    * @brief Handle changes to chained mode status
    * @param chained_mode true if controller should operate in chained mode
    * @return true if mode change is accepted
    * 
    * Currently accepts all mode changes. In chained mode, the controller
    * reads references from upstream controllers rather than subscribers.
    */
   bool on_set_chained_mode(bool chained_mode) override;

   /**
    * @brief Update references from command subscribers (real-time safe)
    * @param time Current time
    * @param period Control loop period
    * @return OK if update successful, ERROR otherwise
    * 
    * Reads new commands from the command buffer and updates internal references.
    * This runs in the real-time control loop when not in chained mode.
    */
   controller_interface::return_type update_reference_from_subscribers(
          const rclcpp::Time& time, const rclcpp::Duration& period) override;

   /**
    * @brief Main control loop - compute and write commands (real-time safe)
    * @param time Current time
    * @param period Control loop period  
    * @return OK if update successful, ERROR otherwise
    * 
    * Core control algorithm that:
    * 1. Reads joint feedback from hardware
    * 2. Updates trapezoidal motion planners
    * 3. Generates smooth position/velocity commands
    * 4. Writes commands to hardware interfaces
    * 5. Published controller state for monitoring
    */
   controller_interface::return_type update_and_write_commands(
          const rclcpp::Time& time, const rclcpp::Duration& period) override;

   protected:
   /** @brief Logger for controller messages */
   rclcpp::Logger mLogger;

   /** @brief Interface for reading joint state from hardware */
   JointStateInterfaces mStateInterfaces;

   /** @brief Proxy for handling command and reference interfaces */
   std::unique_ptr<CommandProxy> mCommandProxy;

   /** @brief Parameter listener for dynamic parameter updates */
   std::shared_ptr<trapezoidal_joint_controller::ParamListener> mParamListener;

   /** @brief Current parameter values */
   trapezoidal_joint_controller::Params mParams;

   /** @brief Real-time safe buffer for incoming commands */
   realtime_utils::RealtimeBoxBestEffort<JointCommand> mCommandBuffer;

   /** @brief Vector of planner states - one per joint */
   std::vector<JointPlannerState> mJointPlanners;

   // State publishing infrastructure
   using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
   using StatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
   using StatePublisherPtr = std::unique_ptr<StatePublisher>;
   
   /** @brief Publisher for controller state messages */
   rclcpp::Publisher<ControllerStateMsg>::SharedPtr mPublisher;
   
   /** @brief Real-time safe state publisher */
   StatePublisherPtr mStatePublisher;
   
   /** @brief Current controller state message */
   control_msgs::msg::JointTrajectoryControllerState mControllerState;

   /**
    * @brief Declare and initialize controller parameters
    * 
    * Sets up the parameter listener and reads initial parameter values.
    * Called during controller initialization phase.
    */
   void declare_parameters();

   /**
    * @brief Read and validate controller parameters
    * @return SUCCESS if parameters are valid, FAILURE otherwise
    * 
    * Validates parameter consistency (array sizes, positive limits, etc.)
    * and initializes internal data structures based on parameter values.
    */
   controller_interface::CallbackReturn read_parameters();

   /**
    * @brief Check if an interface type is present in a list
    * @param interface_type_list List of interface types to search
    * @param interface_type Target interface type to find
    * @return true if interface type is found, false otherwise
    * 
    * Helper function for validating interface configurations.
    */
   bool contains_interface_type(
          const std::vector<std::string>& interface_type_list,
          const std::string& interface_type);

   /**
    * @brief Register command interfaces for a joint component
    * @param component Joint/component name
    * @param interfaceConfig List of interface types to register
    * @return true if registration successful, false otherwise
    * 
    * Legacy method - functionality moved to CommandProxy.
    */
   bool register_actuation_commands(
          const std::string& component,
          const std::vector<std::string>& interfaceConfig);

   /**
    * @brief Register state interfaces for a joint component  
    * @param component Joint/component name
    * @param interfaceConfig List of interface types to register
    * @return true if registration successful, false otherwise
    * 
    * Legacy method - functionality moved to JointStateInterfaces.
    */
   bool register_state_interfaces(
          const std::string& component,
          const std::vector<std::string>& interfaceConfig);

   /**
    * @brief Unclaims all registered interfaces
    * 
    * Releases command and state interface registrations.
    * Called during deactivation to cleanly disconnect from hardware.
    */
   void unclaimAllInterfaces();

   /**
    * @brief Unregister all interfaces  
    * 
    * Removes interface registrations from the registry.
    * Called during cleanup phase.
    */
   void unregisterInterfaces();

   /**
    * @brief Claim interfaces to actual hardware interfaces
    * @return true if claiming successful, false otherwise
    * 
    * Connects the registered interfaces to the actual hardware interfaces
    * provided by the controller manager. Called during activation.
    */
   bool claimInterfaces();

   /**
    * @brief Update and publish controller state information
    * @param timeStamp Optional timestamp for the state message
    * 
    * Populates the controller state message with current feedback, commands,
    * references, and errors, then publishes it for monitoring.
    */
   void updateAndPublishState(const std::optional<rclcpp::Time>& timeStamp);

   /**
    * @brief Validate a joint command message
    * @param message Command message to validate
    * @return true if command is valid, false otherwise
    * 
    * Checks that the command contains the required fields for the current
    * control mode and that all arrays have consistent sizes.
    */
   bool isCommandValid(const JointCommand& message) const;
};

}  // namespace trapezoidal_joint_controller

#endif  // TRAPEZOIDAL_JOINT_CONTROLLER_HPP_
