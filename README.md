# Mobile Robot Project with ROS 2 Jazzy and Nav2

## Overview
This project aims to develop a small forklift mobile robot using ROS 2 Jazzy on a Raspberry Pi running Ubuntu. The robot uses rear-wheel steering with front-wheel drive, and will utilize Nav2 for navigation with velocity control and feedback systems implemented via UART communication.

## Setup
- **ROS 2 Distribution**: Jazzy
- **Platform**: Raspberry Pi with Ubuntu
- **Navigation Stack**: Nav2

## Robot Configuration
### Physical Parameters
- **Type**: Small forklift with Ackermann steering
- **Dimensions**: Length: 0.5m, Width: 0.4m, Height: 0.2m
- **Wheelbase**: 0.3m (distance between front and rear axles)
- **Track Width**: 0.35m (distance between left and right wheels)
- **Front Wheel Radius**: 0.07m
- **Rear Wheel Radius**: 0.05m
- **Wheel Width**: 0.03m
- **Max Steer Angle**: 45 degrees (0.7854 radians)
- **Mass**: 5.0kg (base) + 0.5kg per wheel

### Drive System
- **Steering**: Rear-wheel steering (forklift-style)
- **Front Wheels**: Drive only (forward/backward movement)
- **Rear Wheels**: Steering only (turning)

### Sensor Configuration
- **IMU**: Connected via UART for orientation feedback
- **Velocity Feedback**: Actual linear and angular velocities read via UART

## Navigation with Nav2
- Use Nav2 stack for autonomous navigation
- Robot description: `src/mobile_robot_description/urdf/robot.urdf.xacro`
- Launch file: `src/mobile_robot_description/launch/nav2_simulation.launch.py`
- Configuration: `src/mobile_robot_description/config/nav2_params.yaml`
- RViz config: `src/mobile_robot_description/rviz/nav2_default_view.rviz`
- Set up navigation parameters (costmaps, planners, controllers)

### Running Nav2 Simulation
```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build the workspace
cd /home/alan/mobile-robot
colcon build

# Source the workspace
source install/setup.bash

# Launch Nav2 simulation
ros2 launch mobile_robot_description nav2_simulation.launch.py
```

## Velocity Control
- Monitor linear and angular velocity commands from Nav2
- Print velocity commands to console for debugging
- Velocity monitor node: `src/mobile_robot_description/mobile_robot_description/velocity_monitor.py`
- Subscribes to `/cmd_vel` topic and displays velocity values

## Feedback Systems
### IMU Integration
- IMU sensor included in URDF for simulation
- Gazebo IMU plugin provides simulated IMU data
- Publishes to `/imu/data` topic

### Velocity Feedback
- Odometry data provided by Gazebo Ackermann controller
- Publishes to `/odom` topic
- Velocity monitoring through console output

## Implementation Plan
1. **Robot Description**: ✅ Created URDF/Xacro files with robot dimensions and parameters (`urdf/robot.urdf.xacro`)
2. **Hardware Interface**: ✅ Created velocity monitor node for console output instead of UART
3. **IMU Node**: ✅ IMU simulation through Gazebo plugin
4. **Velocity Control Node**: ✅ Velocity monitoring and display implemented
5. **Nav2 Integration**: ✅ Created ROS 2 workspace and Nav2 configuration for forklift robot
6. **Testing**: Unit tests and integration tests for each component
7. **Simulation**: ✅ Gazebo simulation ready for testing

## Next Steps
- Define specific robot dimensions and parameters
- Implement UART communication protocols
- Develop ROS 2 nodes for control and feedback
- Integrate with Nav2 stack
- Test on actual hardware

## Dependencies
- ROS 2 Jazzy
- Nav2 packages
- UART communication libraries
- IMU drivers (if applicable)
