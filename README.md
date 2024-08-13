# ss24-gazebo-simulation-freddy

## Simulation of the Freddy robot in Gazebo Harmonic

This repository provides a simulation of the mobile dual-manipulator robot Freddy in the Gazebo Harmonic simulator, providing independent control of the Robile base, left Kinova arm, and right Kinova arm components of the robot. The simulation offers independent control of:
- position, velocity, and effort joint control for each of the two driven wheels of each Kelo drive of the Robile base
- position and velocity trajectory control, and effort joint control, for each Kinova arm

The simulation can be launched using the following command:
```
ros2 launch freddy_gazebo freddy_gazebo.launch.py
```

By default, the `base_controller` for controlling the Robile base is a velocity controller. This can be changed by providing a `base_controller` launch argument. For example:
```
ros2 launch freddy_gazebo freddy_gazebo.launch.py base_controller:=position
ros2 launch freddy_gazebo freddy_gazebo.launch.py base_controller:=velocity
ros2 launch freddy_gazebo freddy_gazebo.launch.py base_controller:=effort
```

For purposes of demonstration, commands can be given to each joint of each component of the robot through keyboard input. The executable for commanding the robot's joints can be launched using the following command:
```
ros2 run freddy_gazebo freddy_gazebo
```

### References

- https://automaticaddison.com/how-to-control-a-robotic-arm-using-ros-2-control-and-gazebo/#Gazebo_new_version

- https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2/

- https://github.com/ros-controls/gz_ros2_control/tree/master/gz_ros2_control_demos

- https://control.ros.org/rolling/doc/ros2_control/doc/index.html

- https://github.com/gazebosim/gz-sim/tree/gz-sim7

- https://github.com/Kinovarobotics/ros2_kortex/tree/main
