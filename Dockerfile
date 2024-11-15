# Use an official ROS image as a parent image
FROM osrf/ros:jazzy-desktop-full

# Set environment variables
ENV GZ_VERSION=harmonic

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joint-state-broadcaster

# Create workspace directory
RUN mkdir -p /root/eddie_ws/src

# Clone repositories
WORKDIR /root/eddie_ws/src
RUN git clone https://github.com/secorolab/eddie_gazebo.git

# Import dependent packages
RUN vcs import < /root/eddie_ws/src/eddie_gazebo/dep.repos

# Build the workspace
WORKDIR /root/eddie_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source the workspace
RUN echo "source /root/eddie_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]