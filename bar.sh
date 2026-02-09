root_path="/home/apollo/disk/ros2"
# root_path="/home/phak/ros2"
clear
cd $root_path
rm -rf ./build/all_launcher ./build/car ./build/mpc_planner
rm -rf ./install/all_launcher ./install/car ./install/mpc_planner
# 检查 gs_usb 模块是否加载
if ! lsmod | grep -q "gs_usb"; then
  echo "加载 gs_usb 模块..."
  sudo modprobe gs_usb
else
  echo "gs_usb 模块已经加载。"
fi
# 检查 can2 接口是否启动，如果没有，则启动它
if ! ip link show can2 | grep -q "state UP"; then
  echo "启动 can2 接口..."
  bash $root_path/src/ugv_sdk/scripts/bringup_can2usb_500k.bash
else
  echo "can2 接口已经启动。"
fi

colcon build --packages-select car mpc_planner all_launcher
# colcon build
source $root_path/install/setup.bash
ros2 launch all_launcher all.launch.py
# ros2 launch car car_launch.py log_level:=info
