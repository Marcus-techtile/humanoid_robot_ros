# General Joint Controller - G1 Control Package

## Overview

The general `joint_controller` is a highly configurable C++ ROS node that can control G1 humanoid robot joints from **any command source**. Unlike the previous GUI-specific controller, this implementation accepts commands from configurable topics and supports multiple input message types.

## Key Features

- **🔧 Configurable Topics**: Input and output topics can be set via launch file parameters
- **📡 Multiple Input Types**: Supports both `sensor_msgs/JointState` and `std_msgs/Float64MultiArray` 
- **⚡ Direct & Buffered Modes**: Immediate forwarding or controlled loop operation
- **🛡️ Safety Validation**: Built-in joint command validation and limits
- **🤖 Robot Agnostic**: Configurable joint names for different robots
- **⏱️ Flexible Control Rate**: Adjustable control loop frequency

## Architecture

### File Structure
```
src/humanoid_robot_ros/g1_control/
├── include/g1_control/
│   ├── joint_controller.h           # General controller header
│   └── joint_gui_controller.h       # (Legacy - deprecated)
├── src/
│   ├── joint_controller.cpp         # General controller implementation
│   ├── joint_controller_main.cpp    # Main for general controller
│   └── joint_gui_controller_main.cpp # Main for legacy compatibility
├── launch/
│   ├── joint_controller_gui.launch  # Controller with GUI
│   ├── joint_controller_direct.launch # Controller with direct commands
│   └── joint_gui_control.launch     # Legacy GUI controller
└── README_general_joint_controller.md
```

### Class Design
- **Namespace**: `g1_control::JointController`
- **Configurable Parameters**: All topics and behaviors configurable via ROS parameters
- **Smart Input Detection**: Automatically detects input message type based on topic name
- **Safety First**: Validates all joint commands before execution

## Configuration Parameters

### Core Parameters
```yaml
control_rate: 50.0              # Control loop frequency (Hz)
max_joint_velocity: 1.0         # Maximum joint velocity (rad/s)
position_tolerance: 0.001       # Position tolerance (rad)
direct_mode: false              # If true, forward commands immediately
```

### Topic Configuration
```yaml
joint_command_topic: "/joint_states"     # Input command topic
joint_output_topic: "/joint_commands"    # Output command topic
```

### Joint Names (Optional)
```yaml
robot_joint_names:              # Custom joint names array
  - "joint1"
  - "joint2"
  # ... (defaults to G1 joints if not specified)
```

## Supported Input Formats

### 1. JointState Messages (sensor_msgs/JointState)
**Use Case**: joint_state_publisher_gui, MoveIt!, custom joint controllers

**Example Topic**: `/joint_states`
```yaml
header:
  stamp: {now}
  frame_id: "world"
name: ["left_hip_pitch_joint", "left_hip_roll_joint", ...]
position: [0.1, 0.0, 0.2, ...]
velocity: [0.0, 0.0, 0.0, ...]
effort: [0.0, 0.0, 0.0, ...]
```

### 2. Float64MultiArray Messages (std_msgs/Float64MultiArray)
**Use Case**: Direct numerical control, trajectory controllers, custom algorithms

**Example Topic**: `/direct_joint_commands`
```yaml
data: [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, ...]  # 29 joint positions for G1
```

## Usage Examples

### Example 1: GUI Control (Joint State Publisher)
```bash
# Launch with joint_state_publisher_gui
roslaunch g1_control joint_controller_gui.launch

# Optional: With RViz
roslaunch g1_control joint_controller_gui.launch rviz:=true
```

### Example 2: Direct Numerical Control
```bash
# Launch controller for direct commands
roslaunch g1_control joint_controller_direct.launch

# Send commands via command line
rostopic pub /direct_joint_commands std_msgs/Float64MultiArray \
  "data: [0.0, 0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

### Example 3: Custom Configuration
```bash
# Create custom launch file with your parameters
roslaunch g1_control joint_controller_gui.launch \
  control_rate:=100 \
  joint_command_topic:="/my_joint_commands" \
  joint_output_topic:="/robot/joint_commands" \
  direct_mode:=true
```

### Example 4: MoveIt! Integration
```xml
<!-- In your custom launch file -->
<node name="joint_controller" pkg="g1_control" type="joint_controller" output="screen">
    <param name="joint_command_topic" value="/move_group/fake_controller_joint_states"/>
    <param name="joint_output_topic" value="/joint_commands"/>
    <param name="direct_mode" value="false"/>
    <param name="control_rate" value="50"/>
</node>
```

## Operating Modes

### 1. Buffered Mode (direct_mode: false)
- Commands are stored internally
- Published at regular intervals (control_rate)
- Good for: Smooth control, filtering, rate limiting
- **Default behavior**

### 2. Direct Mode (direct_mode: true)  
- Commands forwarded immediately upon receipt
- No internal control loop
- Good for: Real-time control, low-latency applications
- **Use with caution** - no rate limiting

## Safety Features

### Joint Command Validation
- **Range Checking**: Commands limited to ±π radians
- **NaN/Infinity Detection**: Invalid commands rejected
- **Joint Name Validation**: Only known joints accepted
- **Configurable Limits**: Extend with custom validation

### Error Handling
- Graceful handling of malformed messages
- Logging of rejected commands
- Automatic fallback to safe positions

## Integration Examples

### With MuJoCo Simulation
```bash
# Terminal 1: Start simulation (without joint state publishing)
roslaunch g1_mujoco_sim mujoco_sim.launch publish_joint_states:=false

# Terminal 2: Start joint controller
roslaunch g1_control joint_controller_gui.launch
```

### With Custom Algorithm
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

def custom_controller():
    rospy.init_node('custom_controller')
    pub = rospy.Publisher('/direct_joint_commands', Float64MultiArray, queue_size=1)
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Your custom algorithm here
        commands = Float64MultiArray()
        commands.data = [0.0] * 29  # G1 has 29 joints
        # ... calculate joint commands ...
        
        pub.publish(commands)
        rate.sleep()

if __name__ == '__main__':
    custom_controller()
```

### With Trajectory Controllers
```yaml
# In your trajectory controller config
trajectory_controller:
  joint_command_topic: "/trajectory_commands"
  # Controller publishes JointState messages to this topic
  
# Joint controller subscribes to same topic
joint_controller:
  joint_command_topic: "/trajectory_commands"
  joint_output_topic: "/joint_commands"
```

## Default Joint Names (G1 Humanoid)

The controller defaults to G1 humanoid joint names (29 DOF):

**Left Leg (6 DOF):**
- left_hip_pitch_joint, left_hip_roll_joint, left_hip_yaw_joint
- left_knee_joint, left_ankle_pitch_joint, left_ankle_roll_joint

**Right Leg (6 DOF):**
- right_hip_pitch_joint, right_hip_roll_joint, right_hip_yaw_joint  
- right_knee_joint, right_ankle_pitch_joint, right_ankle_roll_joint

**Waist (3 DOF):**
- waist_yaw_joint, waist_roll_joint, waist_pitch_joint

**Left Arm (7 DOF):**
- left_shoulder_pitch_joint, left_shoulder_roll_joint, left_shoulder_yaw_joint
- left_elbow_joint, left_wrist_roll_joint, left_wrist_pitch_joint, left_wrist_yaw_joint

**Right Arm (7 DOF):**
- right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint
- right_elbow_joint, right_wrist_roll_joint, right_wrist_pitch_joint, right_wrist_yaw_joint

## Migration from Legacy Controller

### Old Way (GUI-specific):
```bash
roslaunch g1_control joint_gui_control.launch
```

### New Way (General):
```bash
# Same functionality, more flexible
roslaunch g1_control joint_controller_gui.launch
```

### Benefits of Migration:
- ✅ Any command source (not just GUI)
- ✅ Configurable topics
- ✅ Multiple input message types
- ✅ Better error handling
- ✅ More flexible deployment

## Troubleshooting

### No Joint Movement
```bash
# Check if commands are being received
rostopic echo /joint_commands

# Check if controller is receiving input
rostopic echo /joint_states  # or your command topic

# Verify controller is running
rosnode info joint_controller
```

### Wrong Joint Names
```bash
# Check what joints the controller expects
rosparam get /joint_controller/robot_joint_names

# Verify your command source publishes correct joint names
rostopic echo /your_command_topic
```

### Performance Issues
```bash
# Check message rates
rostopic hz /joint_commands
rostopic hz /joint_states

# Adjust control rate
roslaunch ... control_rate:=100  # Increase frequency
```

## Advanced Configuration

### Custom Robot Configuration
```yaml
# custom_robot.yaml
joint_controller:
  robot_joint_names:
    - "base_rotation"
    - "shoulder_pan"
    - "shoulder_lift" 
    - "elbow_flex"
    # ... your robot's joints
  control_rate: 200
  max_joint_velocity: 2.0
```

### Multiple Controllers
```xml
<!-- Run multiple controllers for different robot parts -->
<node name="arm_controller" pkg="g1_control" type="joint_controller">
    <param name="joint_command_topic" value="/arm_commands"/>
    <param name="joint_output_topic" value="/arm_joint_commands"/>
    <rosparam param="robot_joint_names">
        ["shoulder_pan", "shoulder_lift", "elbow_flex"]
    </rosparam>
</node>

<node name="leg_controller" pkg="g1_control" type="joint_controller">
    <param name="joint_command_topic" value="/leg_commands"/>
    <param name="joint_output_topic" value="/leg_joint_commands"/>
    <!-- Different joint set -->
</node>
```

This general joint controller provides the flexibility to integrate with any control system while maintaining the safety and reliability needed for robotic applications! 🤖✨ 