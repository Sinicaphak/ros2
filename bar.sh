root_path="/home/apollo/disk/ros2"

cd $root_path
colcon build
source $root_path/install/setup.bash
# clear
# ros2 launch car car_launch.py
ros2 launch mpc_planner gzaebo.launch.py