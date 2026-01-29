# Autonomous Security Drone (PX4 + ROS 2 + Jetson + SIYI A8 + Isaac Sim)

Autonomous home security drone project using:
- **Flight Controller:** Pixhawk 6C (PX4)
- **Onboard Compute:** NVIDIA Jetson Orin Nano
- **Camera:** SIYI A8
- **Middleware:** ROS 2 Humble + PX4 uXRCE-DDS bridge
- **Simulation:** PX4 SITL + Gazebo (gz) and **NVIDIA Isaac Sim** (photorealistic)

## Repo Layout
- `ros_ws/` ROS 2 workspace (`ros_ws/src` contains packages)
- `sim/isaac/` Isaac Sim scenes, scripts, synthetic-data tools
- `sim/px4_gz/` PX4 SITL + Gazebo assets/launch
- `ml/` training, evaluation, export (ONNX/TensorRT)
- `results/` benchmark tables + sim run manifests (no large binaries)

## Status
- [ ] Repo scaffold
- [ ] ROS2 bringup + PX4 offboard MVP in SITL
- [ ] Perception pipeline (TensorRT)
- [ ] Autonomy behaviors (patrol/investigate/RTL)
- [ ] Isaac Sim scenario suite + benchmarks
