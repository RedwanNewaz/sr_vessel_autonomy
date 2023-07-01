# Use the ROS 2 Humble base image
FROM osrf/ros:humble-desktop

# Install additional dependencies if needed
# RUN apt-get update && apt-get install -y <package_name>

# Create a workspace directory
WORKDIR /colcon_ws

# Copy the package(s) into the workspace
COPY . /colcon_ws/src/sr_vessel_autonomy

RUN rosdep install --from-paths src

# Build the packages using colcon
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install

# Source the workspace setup file
RUN echo "source /colcon_ws/install/setup.bash" >> /root/.bashrc

# Set the default command to start a bash shell
CMD ["/bin/bash"]

# docker run -it --rm --net host -v $SRV:/colcon_ws/src/sr_vessel_autonomy osrf/ros:humble-desktop bash
