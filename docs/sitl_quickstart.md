# SITL Quickstart (WSL2 + PX4 SITL + ROS 2 Humble)

Baseline “known-good” setup:
- PX4 SITL (Gazebo / gz)
- Micro XRCE-DDS Agent (UDP 8888)
- ROS 2 offboard node streaming `OffboardControlMode` + `TrajectorySetpoint` (~20 Hz)

## Prereqs
- ROS 2 Humble installed
- Gazebo (gz) installed and working
- Repo cloned at `~/autonomous-security-drone`
- Scripts exist and are executable:
  - `scripts/run_agent.sh`
  - `scripts/run_sitl.sh`
  - `scripts/run_offboard.sh`

Make scripts executable (one-time):
```bash
cd ~/autonomous-security-drone
chmod +x scripts/*.sh

# One-time: setup + build (ROS workspace)
# This repo expects px4_msgs to be built in the ROS workspace
# bash
cd ~/autonomous-security-drone/ros_ws/src

# Get px4_msgs (pick release/1.16 if available, otherwise main)
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
git checkout release/1.16 || git checkout main
cd ../..

# Clean and build
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source the overlay
source install/setup.bash

# Sanity checks
ros2 pkg prefix px4_msgs
ros2 interface list | grep -m 5 px4_msgs/msg || true

# RUN 3 TERMINALS
# TERMINAL A
cd ~/autonomous-security-drone
./scripts/run_agent.sh 8888

# TERMINAL B - PX4 SITL (gz)
cd ~/autonomous-security-drone
./scripts/run_sitl.sh gz_x500

# TERMINAL C - ROS2 Offboard Commander
cd ~/autonomous-security-drone
./scripts/run_offboard.sh asd_px4 offboard_commander_node

# Verify (optional)
# In a separate terminal after Terminal C is running: 
source /opt/ros/humble/setup.bash
source ~/autonomous-security-drone/ros_ws/install/setup.bash

ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/trajectory_setpoint
# Expected: both topics publish ~20 Hz while the offboard commander is running

