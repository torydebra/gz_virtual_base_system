# gz_virtual_base_system

A `gz_ros2_control` system plugin and companion controller for simulating a holonomic mobile base in Gazebo using three virtual joints:

- `virtual_joint_x` (world X translation)
- `virtual_joint_y` (world Y translation)
- `virtual_joint_yaw` (yaw rotation)

The package exposes a body-frame velocity interface (`vx`, `vy`, `wz`) through ROS 2 Control GPIO interfaces, allowing the base to be commanded with standard mobile robot velocity commands while keeping the simulation implementation independent from wheel kinematics. URDF macro are added for convenience. Odom topic is provided with zero covariance (ground truth)

Compiled and tested in ROS2 Jazzy

## Why
- Useful for simulating omindirectional bases, simplifying the simulation without using the caster wheels. 
- This makes the simulation more coherent with some commercial mobile platform, which directly exposes APIs for controlling linear and angular velocity (e.g. Kuka mobile bases KMR)

## How
- Add the xacro macro in your robot (`virtual_base_macro.urdf.xacro`)
- Define your controller configuration file (e.g. `controller_config_example.yaml`)
- Enjoy!

### Example
- ```ros2 launch gz_virtual_base_system mobile_base_example.launch.py```

- Virtual base controller starts active, exposing cmd_vel topic:
  ~~~sh
  ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped 'header:
  stamp:
  sec: 0
  nanosec: 0
  frame_id: ''
  twist:
  linear:
  x: 0.2
  y: 0.1
  z: 0.0
  angular:
  x: 0.0
  y: 0.0
  z: 0.2
  ' -r 100
  ~~~

- odom topic: `ros2 topic echo /odom`

## Features

- Body-frame velocity interface:
  - `mobile_base/vx`
  - `mobile_base/vy`
  - `mobile_base/wz`
- Virtual planar base implementation using Gazebo joints
- Optional acceleration limiting
- ROS 2 controller for `TwistStamped` commands
- Command watchdog timeout
- Odometry publishing (ground truth from virtual joint states)
- Compatible with `gz_ros2_control`

## Architecture

```text
TwistStamped
      |
      v
GazeboVirtualBaseVelocityController
      |
      v
mobile_base/vx
mobile_base/vy
mobile_base/wz
      |
      v
GazeboVirtualBaseSystem
      |
      v
virtual_joint_x
virtual_joint_y
virtual_joint_yaw
      |
      v
Gazebo Physics
```

## Alternatives
- Mecanum wheels simulation and relative controller
- Planar base plugin, direcly calling gazebo API `SetLinkVelocity` on the base link. I tried this option for ros2 jazzy (https://github.com/torydebra/gazebo_planar_move_plugin), but it does not work so well (like actual velocities are much lower than commanded ones).
- Keeping virtual joints, and directly commanding them with an additional script that convert the angles. This is done internally in the implemented custom hardware interface, but simpler architectures may want to avoid this.
