# Simulation of the Eddie robot in Gazebo

This repository provides a simulation of the mobile dual-manipulator robot Eddie in the Gazebo
simulator. The simulation offers independent control of the mobile base and the dual arms of the
robot.

## ROS and Gazebo

- **ROS Distribution**: Jazzy (on Ubuntu 24.04) -
  [Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- **Gazebo Version**: Harmonic

  ```bash
  sudo apt-get install ros-jazzy-ros-gz
  ```

  - Ensure that the environment variable `GZ_VERSION` is set to `harmonic` when building the
    workspace:

    ```bash
    export GZ_VERSION=harmonic
    ```

### Linked Repositories

The following repositories were tested with specific versions or commits:

- [eddie_description](https://github.com/secorolab/eddie_description.git): `main` branch
- [ros2_kortex](https://github.com/secorolab/ros2_kortex): `main` branch
- [ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper.git): `main` branch

### Features

- Position, velocity, and effort control for each of the two driven wheels of each Kelo wheel drive
  unit of the mobile base
- Position and velocity trajectory control, and effort joint control for each Kinova arm
- Selectable controllers for the mobile base and Kinova arms using launch arguments

## Setup

### Prerequisites

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

### Workspace

Create a new workspace and clone the following repositories:

```bash
mkdir -p ~/eddie_ws/src && cd ~/eddie_ws/src

git clone https://github.com/secorolab/eddie_gazebo.git
```

#### Clone ependendent packages

```bash
cd ~/eddie_ws/src

vcs import < eddie_gazebo/dep.repos
```

### Build

```bash
cd ~/eddie_ws

source /opt/ros/jazzy/setup.bash

colcon build
```

### Source the Workspace

```bash
source ~/eddie_ws/install/setup.bash
```

## Docker

### Setup NVIDIA Runtime for Docker

Install the [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) and configure it for the Docker.

### Build the Docker Image

```bash
cd ~/eddie_ws/src/eddie_gazebo

docker build -t eddie_gazebo .
```

### Run the Docker Container

```bash
# to view the GUI on host machine
xhost +local:root

# to run the container with GUI support and interactively
sudo docker run -it --rm --runtime=nvidia --gpus all  \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    eddie_gazebo:latest
```

## Usage

### Launch the simulation

```bash
ros2 launch eddie_gazebo eddie_gazebo.launch.py
```

- To move the base using `Teleop`, select the `Teleop` from the `Tools` menu in the Gazebo GUI.
- Change the `topic` to `/model/robot/cmd_vel` and Enter.
- Then, you can move the robot using the selected controls.

1. By default, the `base_controller` for controlling the mobile base is a velocity controller and
   the `arm_controller` for controlling the Kinova arms is a `trajectory_controller`. You can
   customize the base_controller and arm_controller by providing launch arguments, for example:

    ```bash
    ros2 launch eddie_gazebo eddie_gazebo.launch.py arm_controller:=joint_trajectory base_controller:=position
    ```

    - Valid values for `arm_controller` are `joint_trajectory` and `effort`.
    - Valid values for `base_controller` are `position`, `velocity`, and `effort`.

2. The `joint_trajectory` controller can accept position as well as velocity commands.

### Commanding the Robot's Joints

For purposes of demonstration, commands can be given to each joint of each component of the robot
through keyboard input. The executable for commanding the robot's joints can be launched using the
following command:

```bash
ros2 run eddie_gazebo eddie_control
```

The robot can be controlled using the keyboard as follows:

```bash
Increment state using     w   e   r   t   y   u   i   o
                          ↑   ↑   ↑   ↑   ↑   ↑   ↑   ↑
                Joint     1   2   3   4   5   6   7   8
                          ↓   ↓   ↓   ↓   ↓   ↓   ↓   ↓
Decrement state using     s   d   f   g   h   j   k   l

Increment state increment step using    +
                                        ↑
                                        ↓
Decrement state decrement step using    -

Choose robot component to command using
                                        q : arm_left
                                        a : arm_right
                                        z : base
```

---
> [!WARNING]
> The below instructions are not yet tested and may not work as expected.

The behaviour of this executable can be changed to match the controllers used when launching
`eddie_gazebo.launch.py`. This can be done by providing `arm_controller` and `base_controller`
arguments as follows:

```bash
ros2 run eddie_gazebo eddie_control --ros-args -p arm_controller:=joint_trajectory_velocity -p base_controller:=velocity
```

Valid values for `arm_controller` are `joint_trajectory_position`, `joint_trajectory_velocity`, and
`effort`. Valid values for `base_controller` are `position`, `velocity`, and `effort`. When using a
`joint_trajectory_velocity` command scheme, joint positions are additionally rolled out (or
integrated) using the commanded velocity.

### Switching the Controllers

The default controllers are loaded automatically, and additional controllers can be loaded using ROS
arguments as mentioned above. Alternatively, you can use rqt_controller_manager to switch
controllers without restarting the simulation. To switch controllers, first unload the current
controller through the UI, then load the desired one, please follow these steps to make sure the
correct hardware interfaces are "claimed" and commands run smoothly.

To download rqt_controller_manager, use the following command:

```bash
apt install ros-humble-rqt-controller-manager
```

To run the controller_manager UI, use the following command:

```bash
ros2 run rqt_controller_manager rqt_controller_manager --force-discover
```

In the UI, if controller_manager is not already selected, choose it from the dropdown menu. ![Select
the namespace(controller_manager)](/doc/images/namespace.png)

To unload a controller, right-click on the desired controller and select "De-activate and Unload".
![Unload the controller for left arm/right arm/base](/doc/images/unload.png)

To load the desired controller, right-click on it and select 'Load, Configure and activate'. ![Load
the controller for left arm/right arm/base](/doc/images/load.png)

### Note

The Robotiq gripper has not been integrated or tested for the simulation.
