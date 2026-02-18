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
- **Frame:** Custom Quadcopter build using SolidWorks -> PrusaSlicer -> 3D Printed (PETG-CF)

Software:
- **Middleware:** ROS 2 Humble + PX4 uXRCE-DDS bridge
- **Simulation:** PX4 SITL + Gazebo (gz) and **NVIDIA Isaac Sim** (photorealistic)
  
## Repo Layout
- `ros_ws/` ROS 2 workspace (`ros_ws/src` contains packages)
- `sim/isaac/` Isaac Sim scenes, scripts, synthetic-data tools
- `sim/px4_gz/` PX4 SITL + Gazebo assets/launch
- `ml/` training, evaluation, export (ONNX/TensorRT)
- `results/` benchmark tables + sim run manifests (no large binaries)

## Cosmos Reason 2 - Live Inference
- **Model:** nvidia/Cosmos-Reason2-2B
- **Status:** ✅ Live GPU inference (RTX 5080, FP16)
- **Input:** SIYI A8 camera frames
- **Output:** Chain-of-thought physical reasoning -> parsed threat level + action

### Real Output on Threat Scenario (SIYI A8 frame):

<think>
Okay, let's break this down. The video shows a person in a dark hoodie holding a long, thin object that looks like a knife or similar tool. They're leaning over a desk with a laptop and other items. The setting is indoors, maybe an office or home workspace. The person's posture is aggressive, with the object raised and pointed, which could indicate intent to harm. The presence of a knife or similar tool is a clear security concern because knives are dangerous weapons. The person's behavior, combined with the tool they're holding, suggests they might be preparing to attack someone or something. The desk setup includes a laptop and other items, which could be targets if the person is trying to cause damage. The lighting is dim, which might make it harder to see potential threats, but the overall scene still conveys a sense of danger. Since the person is alone and the video does not show others, it's likely a personal attack scenario. The threat level here is high because of the weapon and aggressive posture.
</think><answer>threat_level: high, action: alert</answer>

**Parsed output:** 
{"threat_level": "high", "action": "alert"}

## Drone Status
- ✅ Repo scaffold
- ✅ ROS2 bringup + PX4 offboard MVP in SITL
- ✅ Cosmos-Reason2-2B live inference on GPU
- ✅ Image input from SIYI A8
- ✅ ROS2 node: frame -> Cosmos reasoning -> /threat_reasoning topic
- ✅ Isaac Sim synthetic data generation (500 frames, Rivermark scene)
- 🔄 Fine-tune on synthetic + real data
- 🔄 PX4 SITL action triggering
- 🔄 Jetson Orin Nano deployment
