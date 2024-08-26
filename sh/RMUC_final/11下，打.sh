#!/bin/bash
source_common="source /opt/ros/humble/setup.bash; source /opt/intel/openvino_2022.3.1/setupvars.sh; source ~/github_code/auto_sentry_ws_/install/setup.bash"
source ~/.bashrc
source /opt/ros/humble/setup.bash

gnome-terminal --window --title=""\
--tab --title="auto_decision" -e "bash -c '$source_common;ros2 launch auto_decision_RMUC attack_outpost_front_11.launch.py; exec bash'" \
--tab --title="auto_base" -e "bash -c '$source_common;ros2 launch auto_base robot_base.launch.py; exec bash'" \
--tab --title="point_lio_cxr" -e "bash -c '$source_common;sleep 3; ros2 launch point_lio_cxr RMUC_left.launch.py; exec bash'" \
--tab --title="pointcloud_cough" -e "bash -c '$source_common;ros2 launch pointcloud_cough pointcloud_cough.launch.py'" \
--tab --title="rm_navigation" -e "bash -c '$source_common;ros2 launch rm_navigation RMUC.launch.py; exec bash'" \
--tab --title="livox_ros_driver2" -e "bash -c '$source_common;ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash'" \
--tab --title="pointcloud_to_laserscan" -e "bash -c '$source_common;ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py;exec bash'" \
--tab --title="remove_pointcloud" -e "bash -c '$source_common;ros2 launch remove_pointcloud remove_pointcloud.launch.py; exec bash'" \
--tab --title="send_goal_" -e "bash -c '$source_common;ros2 launch send_goal_ send_goal.launch.py; exec bash'" \
--tab --title="pid_follow" -e "bash -c '$source_common;ros2 launch pid_follow pid_follow.launch.py; exec bash'" \
--tab --title="rm_vision_bringup" -e "bash -c 'source ~/github_code/rmvision_ws/install/setup.bash;ros2 launch rm_vision_bringup test.launch.py ; exec bash'" \
--tab --title="omnidirectional_perception" -e "bash -c '$source_common;ros2 launch omnidirectional_perception omnidirectional_perception.launch.py; exec bash'" \
#--tab --title="auto_decision" -e "bash -c '$source_common;ros2 run auto_decision_RMUC navigation_check_node; exec bash'" \
#--tab --title="rm_vision_bringup" -e "bash -c 'source ~/github_code/rmvision_ws/install/setup.bash;ros2 launch rm_vision_bringup three_camera.launch.py ; exec bash'" \

#--tab --title="vision_test" -e "bash -c '$source_common;ros2 launch vision_test vision_test.launch.py; exec bash '" \
--hide-menubar
#sleep 5

#NODE_NAME="camera_node"
#RESTARTS=0

#while ((RESTARTS<3)); do
#  if ! ros2 node list | grep -q "${NODE_NAME}"; then
#    echo "Node '${NODE_NAME}' is not running, restarting..."
#    gnome-terminal -- bash -c 'source ~/github_code/rmvision_ws/install/setup.bash;ros2 launch  rm_vision_bringup #vision_bringup.launch.py'
#    ((RESTARTS++))
#  fi
#  sleep 3
#done

