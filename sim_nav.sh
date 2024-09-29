# colcon build --symlink-install

cmds=(
	"ros2 launch pb_rm_simulation rm_simulation.launch.py world:=RMUC \
		use_sim_time:=true
	"
	"ros2 launch imu_complementary_filter complementary_filter.launch.py \
		use_sim_time:=true
	"
    "ros2 launch point_lio mapping_mid360.launch.py \
		use_sim_time:=true rviz:=false
	"
	"ros2 launch remove_pointcloud remove_pointcloud.launch.py \
		use_sim_time:=true
	"
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py \
		use_sim_time:=true
	"
	"ros2 launch rm_navigation bringup_launch.py \
		use_sim_time:=true
	"
	"ros2 run pid_follow_ pid_follow__node"


    # "ros2 launch fast_lio mapping_mid360.launch.launch.py \
	# 	use_sim_time:=true rviz:=false
	# "
	# "ros2 launch linefit_ground_segmentation_ros segmentation.launch.py \
	# 	use_sim_time:=true
	# " 
	# "ros2 launch icp_localization_ros2 bringup.launch.py \
	# 	use_sim_time:=true
	# "
)

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done