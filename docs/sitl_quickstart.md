# SITL Quickstart (WSL2 + PX4 SITL + ROS 2 Humble)

Baseline “known-good” setup:
- PX4 SITL (Gazebo / gz)
- Micro XRCE-DDS Agent (UDP 8888)
- ROS 2 node streaming OffboardControlMode + TrajectorySetpoint (~20 Hz)

## One-time: setup + build
```bash
cd ~/autonomous-security-drone/ros_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
git checkout release/1.16 || git checkout main
cd ..

cd ..
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

```
