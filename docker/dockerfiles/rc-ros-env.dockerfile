FROM osrf/ros:humble-desktop-full

# Set the display environment variable to display 0 (resolved by XServer)
ENV DISPLAY=:0

# Update the bashrc to source the ros2 setup script
RUN /bin/bash -c 'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc'