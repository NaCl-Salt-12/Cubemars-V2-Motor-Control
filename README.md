# Cubemars-V2-Motor-Control (ROS 2)

 This package interfaces with  **CubeMars AK-series motors (V2)** using the MIT control protocol over a CAN bus. It supports various motor models with predefined limits, handles motor state feedback, and provides ROS 2 topics for publishing motor state, and error codes. The node also supports special commands for motor control (e.g., start, stop, zero, clear).

---

## Updates

### Latest Changes
- **Reverse Polarity Support**: Added `reverse_polarity` parameter to invert motor direction without changing control logic
- **Motor Shutoff**: The motor now exits MIT mode/turns off when a keyboard interupt is sent 

### Recent Improvements
- Fixed torque scaling using effective torque constants for each motor model
- Removed unused parameters 
- Improved Documentation 
- Better Error reporting
- Motor State now returns both raw current and scaled torque values 

---

## Roadmap

### Planned Features
- **Safety Features**: Torque/velocity limits, emergency stop functionality, and watchdog timers
- **Diagnostics**: Enhanced health monitoring, motor diagnostics publisher, and fault recovery
- **Additional Protocols**: Support for other Cubemars communication protocols beyond MIT

### Improvements in Progress
- **Absolute Position Accuracy**: Improved unwrapping algorithm and encoder calibration routines
- **Additional Motor Support**: Add support for currently unsuporrted AK series motors

### Known Issues
- `abs_position` tracking may drift over long periods of continuous operation
- CAN bus error handling could be more robust for network interruptions
- Thread synchronization could be optimized for higher frequency control
- Not all Ak series motors supported


## Requirements

- ROS 2 (Humble or Jazzy) and `colcon`
- Python 3.10+
- `can-utils` and a working SocketCAN interface (e.g., `can0`)
- A CAN adapter (MCP2515 HAT or USB-CAN)
- Cubemars rlink
- Cubemars rlink software (changing canID)
---

## Setup

Download the cubemars software from https://www.cubemars.com/technical-support-and-software-download.html
Get the AKseries 1.32 upper computer (for V2.0 motor). 

**When using this software, never 'write' any paramters until you first 'read' them.** If you 'write' first the motor may stop working and you will need to import and rewrite the default parameters. The entire fix process can be found here: https://www.youtube.com/watch?v=_Cj5eYb2aw8&ab_channel=CubeMars%28MotivateAdvancedRoboticSystem%29. The files can be found in the official cubemars discord server: https://www.cubemars.com/technical-support-and-software-download.html.

When opening it, switch it to english by clicking the slider on the bottom left. 

First connect the uart and can with the motor and press refresh on the right, then find the correct port and press connect. You should see the motor firmware show up at the bottom of the page. On the left, click mode switch and press MIT mode. Wait for the popup that says Motor is in MIT mode. 

Next, you will need to access the MIT mode controller with options to input position, speed, etc. Click 'Debug.' In the window that pops up there are options to calibrate the encoder and other features. Type in 'setup.' You will then see multiple commands. Type: **set_can_id (number you choose)**. Example: set_can_id 2. 

To make sure that the can id updated, return back to the MIT controller. Input your new ID in the 'ID' slot and send a command. The motor should spin.

## Installation

1) Clone into your ROS 2 workspace:

```bash
# Example workspace path
cd ~/your_ros2_ws/src
git clone https://github.com/NaCl-Salt-12/Cubemars-V2-Motor-Control.git .
cd ..
colcon build --packages-select cubemars_v2_ros --symlink-install
source install/setup.bash
```

## Parameters

The node declares the following ROS 2 parameters:

| Parameter Name    | Type   | Default Value | Description                                                                 |
|-------------------|--------|---------------|-----------------------------------------------------------------------------|
| `can_interface`   | String | `can0`        | CAN bus interface name (e.g., `can0`).                                      |
| `can_id`          | Integer| `1`           | CAN arbitration ID for the motor (11-bit standard frame, 0x000 to 0x7FF).   |
| `motor_type`      | String | `AK70-10`     | Motor model (e.g., `AK70-10`, `AK80-64`). See supported models below.       |
| `control_hz`      | Double | `20.0`        | Control loop frequency (Hz) for sending MIT control commands.               |
| `joint_name`      | String | `joint`       | Name of the motor/joint for state publishing.                              |
| `auto_start`      | Boolean| `False`       | If `True`, sends a start command (0xFC) on node initialization.             |
| `reverse_polarity`| Boolean| `False`       | If `True`, reverses the motor direction (inverts position, velocity, torque).|

### Supported Motor Types

The node supports the following CubeMars motor models, with predefined limits for position, velocity, torque, and control gains (Kp, Kd):

- `AK10-9`
- `AK60-6`
- `AK70-10`
- `AK80-6`
- `AK80-9`
- `AK80-64`
- `AK80-8`

> [!NOTE]
> Some AK motors (ones not listed above) are not supported at this moment

## Published Topics

The node publishes to the following ROS 2 topics:

| Topic Name       | Message Type                     | Description                                                                 |
|------------------|----------------------------------|-----------------------------------------------------------------------------|
| `error_code`     | `std_msgs.msg.String`           | Motor error code and human-readable error message.                         |
| `motor_state`    | `motor_interfaces.msg.MotorState`| Motor state, including name, position, absolute position, velocity, torque, temperature, and current. |

### MotorState Message

The `MotorState` message contains:
- `name` (string): Motor/joint name.
- `position` (float): Current position (rad, wrapped within ±12.5 rad).
- `abs_position` (float): Unwrapped absolute position (rad).
- `velocity` (float): Current velocity (rad/s).
- `torque` (float): Current torque (Nm), scaled using motor-specific torque constants.
- `current` (float): Current (A), raw value from motor feedback.
- `temperature` (float): Driver board temperature (°C).

> [!WARNING]
> The current implementation of `abs_position` maybe inaccurate.

> [!NOTE]
> The `torque` field is scaled using effective torque constants specific to each motor model, while `current` provides the raw current measurement.

## Subscribed Topics

The node subscribes to the following ROS 2 topics:

| Topic Name | Message Type                     | Description                                                                 |
|------------|----------------------------------|-----------------------------------------------------------------------------|
| `mit_cmd`  | `std_msgs.msg.Float64MultiArray` | MIT control command: `[position, velocity, kp, kd, torque]`.                |
| `special_cmd`  | `std_msgs.msg.String`           | Special commands: `start`, `exit`, `zero`, `clear`.                         |

### MIT Command Format

The `mit_cmd` topic expects a `Float64MultiArray` with exactly 5 elements:
1. `position` (rad): Desired position.
2. `velocity` (rad/s): Desired velocity.
3. `kp` (gain): Position gain.
4. `kd` (gain): Velocity gain.
5. `torque` (Nm): Feedforward torque.

Values are clamped to the motor's limits before being sent over CAN.

### Special Commands

The `special_cmd` topic accepts the following commands (case-insensitive):
- `start`: Sends 0xFC to enable the motor (lazy-start on first `mit_cmd` if not already started).
- `exit`: Sends 0xFD to disable the motor.
- `zero`: Sends 0xFE to set the current position as zero.
- `clear`: Sends a zero command (`[0, 0, 0, 0, 0]`) and holds it until a new `mit_cmd` is received.

## Usage

1. **Launch the Node**:
   ```bash
   ros2 run cubemars_v2_ros motor_node --ros-args \
     -p can_interface:=can0 \
     -p can_id:=1 \
     -p motor_type:=AK70-10 \
     -p control_hz:=20.0 \
     -p joint_name:=joint1 \
     -p auto_start:=false \
     -p reverse_polarity:=false
   ```

2. **Send MIT Commands**:
   ```bash
   ros2 topic pub /joint1/mit_cmd std_msgs/msg/Float64MultiArray "{data: [1.0, 0.0, 100.0, 1.0, 0.0]}"
   ```

3. **Send Special Commands**:
   ```bash
   ros2 topic pub /joint1/special_cmd std_msgs/msg/String "{data: 'start'}"
   ```

4. **Monitor State**:
   ```bash
   ros2 topic echo /joint1/motor_state
   ros2 topic echo /joint1/error_code
   ```
