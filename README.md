# Trapezoidal Joint Controller Documentation

## Overview

The Trapezoidal Joint Controller is a ROS 2 control framework controller that implements smooth, trapezoidal motion profiles for joint control. It provides both position and velocity control modes with configurable acceleration and velocity limits, ensuring smooth motion transitions and respecting joint constraints.

This controller is a `ros2_control` wrapper around the [Trapezoidal Motion Planner](https://github.com/Basiljamal1/trapezoidal_motion_planner)

## Architecture

### Core Components

#### 1. TrapezoidalJointController Class

The main controller class that inherits from `controller_interface::ChainableControllerInterface`. It orchestrates the motion planning and control execution for multiple joints simultaneously.

**Key Features:**
- Supports both position and velocity control modes
- Individual trapezoidal motion planners per joint
- Real-time safe operation
- Configurable motion limits and constraints
- State publishing for monitoring and debugging

**Lifecycle Methods:**
- `on_init()`: Initializes parameters and basic controller setup
- `on_configure()`: Validates parameters and sets up interfaces
- `on_activate()`: Claims interfaces and initializes joint planners
- `on_deactivate()`: Safely stops motion and unclaims interfaces
- `on_cleanup()` & `on_error()`: Resource cleanup and error handling. Unregisters the interfaces and needs reconfiguration. 

#### 2. JointCommand Structure

Represents a command message containing target values for multiple joints:

```cpp
struct JointCommand {
   std::chrono::nanoseconds timestamp;                    // Command timestamp
   std::vector<std::string> jointNames;                   // Names of target joints
   std::optional<std::vector<double>> jointPositions;    // Target positions (position mode)
   std::optional<std::vector<double>> jointVelocities;   // Target velocity profile
   std::optional<std::vector<double>> jointAccelerations; // Target acceleration profile
};
```

**Usage:**
- **Position Mode**: `jointPositions` contains target positions, `jointVelocities` and `jointAccelerations` contain motion limits
- **Velocity Mode**: `jointVelocities` contains target velocities, `jointAccelerations` contains acceleration limits
- All arrays must have the same size as `jointNames`

#### 4. CommandProxy Class

Unified interface for handling both position and velocity control modes. It abstracts the complexity of different control interfaces and provides a consistent API.

**Key Responsibilities:**
- Interface registration and management
- Reference interface configuration for chaining controllers
- Command validation and execution
- State tracking and publishing

**Main Methods:**
- `registerCommandInterfaces()`: Registers hardware command interfaces
- `referenceInterfaceConfiguration()`: Sets up reference interfaces for controller chaining. This depends on the control mode (position vs velocity)
- `setReferencePosition/Velocity/Acceleration()`: Sets motion references (from subscribers when not in chain mode)
- `setCommandPosition/Velocity()`: Sends commands to hardware
- `getReferencePosition/Velocity/Acceleration()`: Retrieves current references for monitoring

## Control Modes

### Position Mode

In position mode, the controller accepts target positions and generates smooth trapezoidal trajectories to reach them.

**Operation:**
1. Receives target position through `JointCommand` via the `setTargetCommand(const JointCommand& command)` interface. This updates the reference interfaces. 
2. Configures trapezoidal planner with position target
3. Planner generates velocity commands respecting acceleration/velocity limits in realtime, in under 0.023ms.
4. Controller integrates velocity to produce smooth position commands
5. Sends position commands to hardware interfaces

In chained mode, the references are updated directly rather than through the `setTargetCommand` interface.

**Parameters Used:**
Check out the parameters file in [Parameters file](src/trapezoidal_joint_controller_parameters.yaml)**

### Velocity Mode

In velocity mode, the controller accepts target velocities and smoothly transitions between different velocity setpoints.

**Operation:**
1. Receives target velocity through `JointCommand`
2. Configures trapezoidal planner for velocity control
3. Planner generates smooth velocity commands respecting acceleration limits
4. Sends velocity commands directly to hardware interfaces

**Parameters Used:**
- `jointVelocities`: Target velocities for each joint
- `jointAccelerations`: Maximum acceleration limits during velocity changes
- `velocity_limits`: Maximum velocity magnitudes

## Configuration Parameters

### Core Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `joints` | string_array | Names of controlled joints | [] |
| `command_interfaces` | string_array | Hardware command interface types. Either position or velocity | ["position"] |
| `state_interfaces` | string_array | Hardware state interface types | Must be atleast ["position", "velocity"] |
| `reference_interfaces` | string_array | Exposed reference interface types | ["position", "velocity", "acceleration"] |
| `control_mode` | string | Control mode: "position", "velocity" | "position" |
| `control_rate` | double | Control loop frequency in Hz | 1000.0 |

### Motion Limits

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `velocity_limits` | double_array | Maximum velocities (rad/s or m/s) | [] |
| `acceleration_limits` | double_array | Maximum accelerations (rad/s² or m/s²) | [] |
| `position_limits_min` | double_array | Minimum position limits (NaN = no limit) | [] |
| `position_limits_max` | double_array | Maximum position limits (NaN = no limit) | [] |

**Important Notes:**
- All limit arrays must have the same size as the `joints` array
- Position limits are optional; use NaN or omit for unlimited joints
- Velocity and acceleration limits are mandatory and must be positive

### Advanced Configuration

The controller uses the trapezoidal motion planner with the following internal configurations:

- **Slip Limits**: Currently disabled (`kNoClamp`) for basic operation. This causes the controller to failt if the error between the target and the robot is beyong the slip limit
- **Velocity Capture**: 5% threshold for velocity target achievement

## Real-Time Considerations

### Thread Safety
- Uses `RealtimeBoxBestEffort` for thread-safe command buffering
- All control loop operations are real-time safe
- State publishing uses real-time publishers to avoid blocking

### Performance Optimizations
- Pre-allocated vectors for motion commands
- Minimal memory allocations during control loop execution
- Efficient parameter validation during configuration phase

## Usage Examples

### Basic Position Control Configuration

```yaml

trapezoidal_position_joint_controller:
  ros__parameters:
    update_rate: 125 # Hz
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    reference_interfaces:
      - position
      - velocity
      - acceleration
    command_interfaces: 
      - position
    state_interfaces:
      - position
      - velocity
    control_mode: position
    velocity_limits: [2.1, 2.1, 3.1, 3.1, 3.1, 3.1]
    acceleration_limits: [3.0, 3.0, 3.0, 7.2, 7.2, 7.2]
```

### Velocity Control Configuration

```yaml
trapezoidal_velocity_joint_controller:
  ros__parameters:
    update_rate: 125 # Hz
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    reference_interfaces:
      - velocity
      - acceleration
    command_interfaces: 
      - velocity
    state_interfaces:
      - position
      - velocity
    control_mode: velocity
    velocity_limits: [2.1, 2.1, 3.1, 3.1, 3.1, 3.1]
    acceleration_limits: [3.0, 3.0, 3.0, 7.2, 7.2, 7.2]
```

### Programming Interface

```cpp
// Create command for position control
JointCommand cmd;
cmd.jointNames = {"joint1", "joint2"};
cmd.jointPositions = {1.57, -0.78};          // Target positions
cmd.jointVelocities = {1.0, 1.0};           // Velocity limits
cmd.jointAccelerations = {2.0, 2.0};        // Acceleration limits

// Send command (non-real-time context)
controller->setTargetCommand(cmd);
```

## State Monitoring

The controller publishes its state on the `~/controller_state` topic using `control_msgs::msg::JointTrajectoryControllerState`:

- **feedback**: Current joint positions and velocities from hardware
- **output**: Commands being sent to hardware
- **reference**: Current motion references
- **error**: Difference between feedback and output. This is the slip parameter above. 

## Integration with Other Controllers

### Controller Chaining

The controller can be chained with other controllers through reference interfaces:
- Upstream controllers write to reference interfaces
- Trapezoidal controller reads references and generates smooth motion
- Commands are sent to hardware or downstream controllers

### Hardware Interface Compatibility

Compatible with standard ROS 2 control hardware interface. Must be one of either:
- Position interfaces: `hardware_interface::HW_IF_POSITION`
- Velocity interfaces: `hardware_interface::HW_IF_VELOCITY`
- State interfaces: Position and velocity feedback required


# Extra details
More details on the motion planner underlying behaviour with trajectory plots can be found on the planner repository. 