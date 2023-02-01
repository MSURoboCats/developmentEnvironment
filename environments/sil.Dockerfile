FROM osrf/ros:humble-desktop-full

# Set the display environment variable to display 0 (resolved by XServer) and the
# pulse_server environment variable to the mounted pulse server (for audio support)
ENV DISPLAY=:0
ENV PULSE_SERVER=/tmp/PulseServer

# Update the bashrc to source the ros2 setup script
RUN /bin/bash -c 'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc'