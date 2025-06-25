# RoboSoccer Control - TSID Controller

**Task Space Inverse Dynamics (TSID) Controller for Humanoid Robot Soccer**

This package implements a state-of-the-art TSID controller for precise humanoid robot control in soccer applications. The controller operates in task space, allowing intuitive control of the robot's center of mass, foot positions, and joint postures while respecting contact constraints.

## tsid:

control approach that:
- **Controls tasks in Cartesian space** (CoM position, foot poses, etc)
- **Handles multiple tasks with priorities** (CoM > feet > posture)
- **Respects contact constraints** (feet on ground, friction limits)
- **Computes optimal joint torques** using inverse dynamics
- **Runs in real-time** at 1000Hz

## **Architecture**

```
Motion Planning → TSID Commands → TSID Controller → Joint Commands → Hardware
     |                |              |               |              |
 Footsteps        Task Space      Inverse          Position      STM32
 CoM Traj         Targets        Dynamics         Commands      UART
```

## **Package Contents**

- **`TSIDController`** - Main TSID implementation with Pinocchio
- **`TSIDControllerNode`** - ROS2 wrapper for the controller
- **`TSIDCommand.msg`** - Task space command message
- **Configuration files** - Tunable parameters and gains

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

#### **Mock Command (from Motion Planning)**
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
## **Integration**

### **Input Interface (from `robosoccer_motion`)**
- **`/tsid_commands`** - Complete task space commands
- **`/com_target`** - Center of mass targets
- **`/left_foot_target`**, **`/right_foot_target`** - Foot targets

### **Output Interface (to hardware)**
- **`/robosoccer_position_controller/commands`** - Joint position commands
- **`/computed_torques`** - Feedforward torques (optional)

### **Feedback Interface**
- **`/joint_states`** - Current robot state from hardware

## **Algorithm Details**

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

