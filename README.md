# RoboSoccer Control - TSID Controller

**Task Space Inverse Dynamics (TSID) Controller for Humanoid Robot Soccer**

This package implements a state-of-the-art TSID controller for precise humanoid robot control in soccer applications. The controller operates in task space, allowing intuitive control of the robot's center of mass, foot positions, and joint postures while respecting contact constraints.

## 🎯 **What is TSID?**

TSID (Task Space Inverse Dynamics) is a modern control approach that:
- **Controls tasks in Cartesian space** (CoM position, foot poses, etc.)
- **Handles multiple tasks with priorities** (CoM > feet > posture)
- **Respects contact constraints** (feet on ground, friction limits)
- **Computes optimal joint torques** using inverse dynamics
- **Runs in real-time** at 1000Hz

## 🏗️ **Architecture**

```
Motion Planning → TSID Commands → TSID Controller → Joint Commands → Hardware
     |                |              |               |              |
 Footsteps        Task Space      Inverse          Position      STM32
 CoM Traj         Targets        Dynamics         Commands      UART
```

## 📦 **Package Contents**

### **Core Components**
- **`TSIDController`** - Main TSID implementation with Pinocchio
- **`TSIDControllerNode`** - ROS2 wrapper for the controller
- **`TSIDCommand.msg`** - Task space command message
- **Configuration files** - Tunable parameters and gains

### **Key Features**
- ✅ **Center of Mass control** - 3D position and orientation
- ✅ **End-effector control** - Precise foot/hand positioning  
- ✅ **Contact constraints** - Friction limits and force bounds
- ✅ **Posture regulation** - Joint-level control for stability
- ✅ **Real-time performance** - 1000Hz control loop
- ✅ **Safety limits** - Velocity and acceleration bounds

## 🚀 **Usage**

### **1. Launch TSID Controller**
```bash
# Launch with default configuration
ros2 launch robosoccer_control tsid_control.launch.py

# Launch with custom config
ros2 launch robosoccer_control tsid_control.launch.py config_file:=/path/to/custom_config.yaml

# Launch with simulation time
ros2 launch robosoccer_control tsid_control.launch.py use_sim_time:=true
```

### **2. Send Task Space Commands**

#### **Complete Command (from Motion Planning)**
```bash
ros2 topic pub /tsid_commands robosoccer_control/msg/TSIDCommand "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'base_link'
desired_com_pose:
  position: {x: 0.0, y: 0.0, z: 0.85}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
desired_com_velocity:
  linear: {x: 0.0, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}
left_foot_pose:
  position: {x: 0.0, y: 0.1, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
right_foot_pose:
  position: {x: 0.0, y: -0.1, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
left_foot_contact: true
right_foot_contact: true
joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joint_velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"
```

#### **Individual Commands**
```bash
# CoM target
ros2 topic pub /com_target geometry_msgs/msg/PoseStamped "
header: {frame_id: 'base_link'}
pose:
  position: {x: 0.05, y: 0.0, z: 0.85}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"

# Left foot target
ros2 topic pub /left_foot_target geometry_msgs/msg/PoseStamped "
header: {frame_id: 'base_link'}
pose:
  position: {x: 0.1, y: 0.1, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

### **3. Monitor Controller Output**
```bash
# Monitor joint commands to hardware
ros2 topic echo /robosoccer_position_controller/commands

# Monitor computed torques
ros2 topic echo /computed_torques

# Monitor joint states
ros2 topic echo /joint_states
```

## ⚙️ **Configuration**

### **Controller Gains**
Edit `config/tsid_config.yaml`:
```yaml
controller_gains:
  kp_com: 200.0      # CoM position gain
  kd_com: 40.0       # CoM damping gain
  kp_ee: 150.0       # End-effector position gain
  kd_ee: 30.0        # End-effector damping gain
  kp_posture: 50.0   # Joint posture gain
  kd_posture: 10.0   # Joint posture damping gain
```

### **Task Priorities**
```yaml
task_weights:
  com_weight: 10.0          # Medium priority
  left_foot_weight: 100.0   # High priority (when in contact)
  right_foot_weight: 100.0  # High priority (when in contact)
  posture_weight: 1.0       # Low priority (regularization)
```

### **Safety Limits**
```yaml
safety:
  max_joint_velocity: 4.0      # rad/s
  max_joint_acceleration: 20.0 # rad/s²
  max_com_acceleration: 10.0   # m/s²
```

## 🔄 **Integration with Robosoccer System**

### **Input Interface (from `robosoccer_motion`)**
- **`/tsid_commands`** - Complete task space commands
- **`/com_target`** - Center of mass targets
- **`/left_foot_target`**, **`/right_foot_target`** - Foot targets

### **Output Interface (to Hardware)**
- **`/robosoccer_position_controller/commands`** - Joint position commands
- **`/computed_torques`** - Feedforward torques (optional)

### **Feedback Interface**
- **`/joint_states`** - Current robot state from hardware

## 🧮 **Algorithm Details**

### **TSID Formulation**
The controller solves the following optimization problem:

```
minimize: ||J*qdd - task_acceleration||² + λ||qdd||²
subject to: M*qdd + h = τ
           J_contact*qdd = 0  (for active contacts)
           τ_min ≤ τ ≤ τ_max
```

Where:
- `qdd` - Joint accelerations (optimization variable)
- `J` - Task Jacobians (CoM, end-effectors)
- `M` - Mass matrix, `h` - Coriolis/gravity forces
- `J_contact` - Contact constraint Jacobians
- `τ` - Joint torques

### **Task Space Error Computation**
```cpp
// CoM error
pos_error = desired_com_pos - current_com_pos
vel_error = desired_com_vel - current_com_vel
desired_acc = desired_com_acc + Kp*pos_error + Kd*vel_error

// End-effector error (6D: position + orientation)
pos_error = desired_ee_pos - current_ee_pos
rot_error = log(desired_rot * current_rot^T)  // SO(3) error
desired_acc = [linear_acc; angular_acc] + K*[pos_error; rot_error] + D*vel_error
```

## 🔧 **Tuning Guidelines**

### **Start Conservative**
1. **Low gains first**: Start with `kp=50, kd=10` for all tasks
2. **Increase gradually**: Double gains until desired performance
3. **Balance priorities**: CoM > Feet > Posture weights

### **Common Issues**
- **Robot shaking**: Reduce gains or increase damping
- **Slow response**: Increase proportional gains
- **Unstable contacts**: Check contact constraints and friction
- **Joint limits**: Verify URDF joint limits match real robot

### **Performance Tuning**
- **1000Hz**: Target for real-time performance
- **<1ms**: Computation time per control cycle
- **Smooth motion**: No discontinuities in commands

## 📊 **Debugging**

### **Useful Topics**
```bash
# Check if TSID is computing
ros2 topic hz /computed_torques

# Verify joint state updates
ros2 topic hz /joint_states

# Monitor command output
ros2 topic echo /robosoccer_position_controller/commands --once
```

### **Common Problems**
- **No joint states**: Check hardware interface is running
- **High torques**: Verify task targets are reasonable
- **No commands**: Check message timestamps and frame IDs

## 🤝 **Integration Example**

```python
# Example: Simple motion planning node
import rclpy
from robosoccer_control.msg import TSIDCommand

class SimpleMotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.cmd_pub = self.create_publisher(TSIDCommand, '/tsid_commands', 10)
        
    def send_walking_step(self):
        cmd = TSIDCommand()
        # Set CoM target
        cmd.desired_com_pose.position.x = 0.05  # Step forward
        cmd.desired_com_pose.position.z = 0.85  # Standing height
        
        # Set foot targets
        cmd.left_foot_pose.position.y = 0.1
        cmd.right_foot_pose.position.y = -0.1
        cmd.left_foot_contact = True
        cmd.right_foot_contact = True
        
        self.cmd_pub.publish(cmd)
```

---

**This TSID controller provides the foundation for sophisticated humanoid robot control in your robosoccer system!** 🤖⚽

