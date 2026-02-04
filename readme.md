cd /home/apollo/disk/ros2/src
pip install casadi
sudo apt install ros-humble-osqp-vendor
git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone https://github.com/agilexrobotics/hunter_ros2.git
cd ..
colcon build