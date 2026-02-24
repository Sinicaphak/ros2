root_path="/home/apollo/disk/ros2"
# root_path="/home/phak/ros2"
cd $root_path
clear
colcon build --packages-select car mpc_planner
# colcon build
source $root_path/install/setup.bash
# ros2 launch car car_launch.py
# ros2 launch mpc_planner gzaebo.launch.py
# ros2 launch hunter_base hunter_base.launch.py
ros2 launch all_launcher all.launch.py