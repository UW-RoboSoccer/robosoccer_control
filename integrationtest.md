# TSID Controller Integration

## Complete Message Flow Architecture

This guide explains how the TSID controller integrates with the mujoco_ros2_control system to create a complete humanoid robot control system.

### Architecture Overview

```
TSID Controller Node
    ↓ (publishes joint commands)
/robosoccer/joint_commands (Float64MultiArray)
    ↓
Enhanced TSID to Trajectory Bridge
    ↓ (converts to trajectory with velocity/acceleration)
/position_controller/joint_trajectory (JointTrajectory)
    ↓
ros2_control Position Controller
    ↓ (sends to hardware interface)
mujoco_ros2_control (MujocoSystem)
    ↓ (applies to simulation)
MuJoCo Physics Engine
    ↓ (publishes joint states)
/joint_states (JointState)
    ↓ (feedback loop)
TSID Controller Node
```

### Key Components

#### 1. TSID Controller Node (`tsid_controller_node`)
- **Purpose**: Computes task-space inverse dynamics control
- **Inputs**: 
  - `/joint_states` - Current robot state
  - `/tsid_commands` - High-level task commands
  - `/com_target` - Center of mass targets
  - `/left_foot_target`, `/right_foot_target` - Foot pose targets
- **Outputs**: `/robosoccer/joint_commands` - Joint position commands
- **Frequency**: 1000 Hz (configurable)

#### 2. Enhanced TSID to Trajectory Bridge (`tsid_to_trajectory_bridge`)
- **Purpose**: Converts TSID commands to ros2_control trajectory format
- **Features**:
  - Velocity and acceleration calculation
  - Joint state feedback for smooth motion
  - Configurable trajectory duration and limits
  - Status monitoring
- **Inputs**: `/robosoccer/joint_commands`, `/joint_states`
- **Outputs**: `/position_controller/joint_trajectory`, `/bridge_status`

#### 3. Test Commander (`tsid_test_commander`)
- **Purpose**: Sends test commands to verify integration
- **Test Modes**:
  - `standing`: Maintain standing pose
  - `com_tracking`: Circular CoM motion
  - `foot_tracking`: Alternating foot lifting
  - `simple_motion`: Forward/backward motion

### Launch Files

#### 1. `integrated_control.launch.py`
Complete system startup with proper timing:
```bash
ros2 launch robosoccer_control integrated_control.launch.py
```

**Startup Sequence**:
1. **t=0s**: Start mujoco_ros2_control system
2. **t=2s**: Start TSID controller
3. **t=4s**: Start enhanced bridge
4. **t=6s**: Start monitoring tools

#### 2. `test_integration.launch.py`
Complete system with test commander:
```bash
# Test standing
ros2 launch robosoccer_control test_integration.launch.py test_mode:=standing

# Test CoM tracking
ros2 launch robosoccer_control test_integration.launch.py test_mode:=com_tracking

# Test foot tracking
ros2 launch robosoccer_control test_integration.launch.py test_mode:=foot_tracking

# Test simple motion
ros2 launch robosoccer_control test_integration.launch.py test_mode:=simple_motion
```

### Configuration Parameters

#### Bridge Parameters
- `trajectory_duration`: 0.05s (trajectory execution time)
- `max_velocity`: 10.0 rad/s (joint velocity limit)
- `max_acceleration`: 50.0 rad/s² (joint acceleration limit)
- `use_sim_time`: true (use simulation time)

#### TSID Controller Parameters
- `control_frequency`: 1000.0 Hz (TSID computation rate)
- `robot_description_package`: my_robot_description
- `urdf_file`: robot.urdf

#### Test Commander Parameters
- `test_mode`: standing/com_tracking/foot_tracking/simple_motion
- `command_frequency`: 10.0 Hz (test command rate)

### Monitoring and Debugging

#### Key Topics to Monitor
```bash
# Joint states from simulation
ros2 topic echo /joint_states

# TSID commands
ros2 topic echo /robosoccer/joint_commands

# Bridge status
ros2 topic echo /bridge_status

# Trajectory commands
ros2 topic echo /position_controller/joint_trajectory
```

#### Visualization Tools
- **RViz**: Robot visualization (included in integrated launch)
- **rqt_plot**: Real-time plotting of joint positions and commands
- **rqt_graph**: Topic connection visualization

### Troubleshooting

#### Common Issues

1. **No joint states received**
   - Check if mujoco_ros2_control is running
   - Verify URDF file path
   - Check joint names match between URDF and controller config

2. **Bridge not publishing trajectories**
   - Verify joint state feedback is received
   - Check joint name order matches
   - Monitor `/bridge_status` for timing issues

3. **TSID controller not computing**
   - Check if robot state is properly initialized
   - Verify TSID configuration parameters
   - Check for TSID library errors

4. **Robot not moving**
   - Verify position controller is spawned
   - Check trajectory messages are being sent
   - Monitor joint command values

#### Debug Commands
```bash
# Check node status
ros2 node list
ros2 node info /tsid_controller_node
ros2 node info /enhanced_tsid_bridge

# Check topic connections
ros2 topic list
ros2 topic info /joint_states
ros2 topic info /robosoccer/joint_commands

# Monitor system performance
ros2 topic hz /joint_states
ros2 topic hz /robosoccer/joint_commands
```

### Testing Strategy

#### Phase 1: Basic Integration
1. Start with `standing` mode
2. Verify robot maintains stable pose
3. Check all topics are publishing correctly

#### Phase 2: Motion Testing
1. Test `simple_motion` mode
2. Verify CoM follows desired trajectory
3. Check joint limits are respected

#### Phase 3: Advanced Features
1. Test `com_tracking` mode
2. Verify circular motion execution
3. Test `foot_tracking` mode

#### Phase 4: Performance Testing
1. Monitor control frequencies
2. Check computation times
3. Verify stability under different loads

 