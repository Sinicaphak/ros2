cd /home/apollo/disk/ros2/src
pip install casadi
sudo apt install ros-humble-osqp-vendor
git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone https://github.com/agilexrobotics/hunter_ros2.git
cd ..
colcon build


sudo apt install v4l-utils
v4l2-ctl --list-devices    
nvgstcapture-1.0 --camsrc=0 --cap-dev-node=0
