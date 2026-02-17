# Autonomous Security Drone (PX4 + ROS 2 + Jetson + SIYI A8 + Isaac Sim)

NVIDIA Cosmos Cookoff Entry, see link for details: https://luma.com/nvidia-cosmos-cookoff?tk=auN5se

Scout Drone: Edge-deployed physical AI agent using Cosmos Reason 2 deployed to Jetson for real-time threat reasoning and autonomous response in home security.

Hardware:
- **Flight Controller:** Pixhawk 6C (PX4) with embedded IMU + barometer (pinout: https://docs.px4.io/main/en/flight_controller/pixhawk6c.html)
- **Onboard Computer:** NVIDIA Jetson Orin Nano (Dev Kit)
- **Camera:** SIYI A8 mini 4k gimbal camera (RTSP stream: rtsp://[IP]:8554/main.264; docs: https://siyi.biz/en/support/download)
- **GPS:** Holybro M10 GPS
- **Motors:** T-motor F90 2806.5 1300 kV x4
- **Propellers:** Gemfan Cinelifter 7" 7035 7x3.5x3 Tri-blade carbon nylon x4
- **ESC:** Tekko32 F4 metal 4-in-1 65A ESC
- **Battery:** Tattu Plus 22000mAh 22.2V 25C 6s1P Lipo Smart Battery Pack with XT90-S Plug
- **Telemetry:** Holybro SiK Telemetry Radio V3
- **Power Regulation:** PM07 Power Module, DC-DC buck converter
- **Frame:** Custom F450 Variant using SolidWorks -> PrusaSlicer -> 3D Printed (PETG-CF)

Software:
- **Middleware:** ROS 2 Humble + PX4 uXRCE-DDS bridge
- **Simulation:** PX4 SITL + Gazebo (gz) and **NVIDIA Isaac Sim** (photorealistic)
  
## Repo Layout
- `ros_ws/` ROS 2 workspace (`ros_ws/src` contains packages)
- `sim/isaac/` Isaac Sim scenes, scripts, synthetic-data tools
- `sim/px4_gz/` PX4 SITL + Gazebo assets/launch
- `ml/` training, evaluation, export (ONNX/TensorRT)
- `results/` benchmark tables + sim run manifests (no large binaries)
  
- ## Cosmos Inference Status
- Model: nvidia/Cosmos-Reason2-2B
- Status: GPU inference successful (RTX 5080, FP16)
- Example output: Full <think>/<answer> chain on suspicious fence approach scenario
- Next: Integrate camera frame input and ROS2 node

## Drone Status
- [x] Repo scaffold
- [x] ROS2 bringup + PX4 offboard MVP in SITL
- [ ] Perception pipeline (TensorRT)
- [ ] Autonomy behaviors (patrol/investigate/RTL)
- [ ] Isaac Sim scenario suite + benchmarks

## Cosmos Reason Status
- [X] Cosmos-Reason2-2B inference on GPU (RTX 5080)
- [ ] Image input from SIYI A8
- [ ] ROS2 node for frame -> Cosmos reasoning -> action pipeline

## Demo Video 
Live reasoning pipeline test with SIYI A8 frame loop (unlisted): https://youtu.be/LEmaBaJEKnU
