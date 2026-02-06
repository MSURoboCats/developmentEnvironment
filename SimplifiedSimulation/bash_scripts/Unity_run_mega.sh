#!/bin/bash

# each of these commands starts up a new terminal tab and runs the commands. Optionally add 'bash' as the last command so the tab will stay open after then final command is completed.

sleep 5

gnome-terminal \
	--tab --title="ROS_TCP" \
		-- bash -c 'cd robotCode/competitionCode2024/ros2_ws; source install/setup.bash; ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0; bash'


gnome-terminal \
	--tab --title="vision" \
		-- bash -c 'cd robotCode/competitionCode2024/ros2_ws; source install/setup.bash; ros2 run the_sub twist2action_node; bash'

gnome-terminal \
	--tab --title="sub_status" \
		-- bash -c 'cd robotCode/competitionCode2024/ros2_ws; source install/setup.bash; ros2 run the_sub sub_status_observer_node; bash'
		
gnome-terminal \
	--tab --title="depth_controller" \
		-- bash -c 'cd robotCode/competitionCode2024/ros2_ws; source install/setup.bash; ros2 run the_sub depth_controller_node; bash'

gnome-terminal \
	--tab --title="heading_controller" \
		-- bash -c 'cd robotCode/competitionCode2024/ros2_ws; source install/setup.bash; ros2 run the_sub heading_controller_node; bash'

gnome-terminal \
	--tab --title="tasks" \
		-- bash -c "cd robotCode/competitionCode2024/ros2_ws; source install/setup.bash; ros2 run the_sub buoy_task_node .4; ros2 topic pub /motor_powers interfaces/msg/MotorPowers '{motor1: 0, motor2: 0, motor3: 0, motor4: 0, motor5: 0, motor6: 0, motor7: 0, motor8: 0}'; bash"