# Baldwin - Scout Drone (V1) - NVIDIA Cosmos Cookoff Entry

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Jetson](https://img.shields.io/badge/Jetson-Orin%20Nano-green)](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
[![NVIDIA](https://img.shields.io/badge/NVIDIA-Cosmos%20Reason2-orange)](https://huggingface.co/nvidia/Cosmos-Reason2-2B)
[![PX4](https://img.shields.io/badge/PX4-v1.14-purple)](https://px4.io/)

**NVIDIA Cosmos Cookoff:** https://luma.com/nvidia-cosmos-cookoff?tk=auN5se

**Demo Video:**
[![Demo Video](https://img.youtube.com/vi/S2ii9YnADrw/0.jpg)](https://youtu.be/S2ii9YnADrw)
**Watch the 2-minute live demo → autonomous threat detection & reasoning on Jetson Orin Nano**

## Project Overview
Edge-deployed physical AI agent (Baldwin) using Cosmos Reason2 for real-time threat reasoning and autonomous response in home security. The drone "reasons" visually (via SIYI A8 camera feed), publishes structured JSON threat assessments (e.g., '{"threat_level": "high", "action": "alert"}'), and integrates with PX4 for offboard flight control.

## Conceptual Design
Baldwin is an embodied AI agent with closed loop communication: hardware as body, software as mind.
**Perception**: SIYI A8 4K gimbal captures video/frames. 
**Reasoning**: Jetson runs Cosmos Reason2 (VLM) with OpenCV for visual CoT (threat ID, actions). Outputs JSON via ROS2.
**Decision/Action**: ROS2 bridge to PX4 for autonomy (hover/alert on high threat). Telemetry/SiK + ELRS for control.
**Endurance**: 22Ah battery + 1300kV motors = 40-60 min patrols; inductive charging for indefinite operation in sensor networks.
**Learning**: Isaac Sim frames + SIYI data for fine-tuning

Key innovation: Low-latency, cloud-free edge AI combining VLM with drone control - scalable for farms, property security, research/military applications. Built for secure, privacy-focused environments. 

## Hardware:
- **Flight Controller:** Pixhawk 6C (PX4)
- **Onboard Computer:** NVIDIA Jetson Orin Nano Dev Kit
- **Camera:** SIYI A8 mini 4k gimbal camera (RTSP: rtsp://[IP]:8554/main.264)
- **GPS/Telemetry:** Holybro M10 GPS + SiK V3
- **Frame:** Custom UFO SolidWorks design, PETG-CF 3D printed
- **Powerplant:** Tattu 22Ah 6S battery, T-Motor F90 1300kV motors, Gemfan 7035 tri-blade props, Tekko32 ESC
- **Note:** Optimized for cinematic/endurance 7-8" drones; modular for variants.

## Software:
- **Middleware:** ROS 2 Humble + PX4 uXRCE-DDS bridge
- **Simulation:** PX4 SITL + Gazebo, NVIDIA Isaac Sim 5.1.0
- **AI:** nvidia/Cosmos-Reason2-2B (VLM physical reasoning)
  
## Repo Layout
- `ros_ws/` — ROS 2 workspace (cosmos_agent package)
- `data/synthetic/` — 500 Isaac Sim frames for fine-tuning
- `data/clips/` — Real SIYI A8 footage
- `data/frames/` — Extracted threat scenario frames
- `ml/` — Training, evaluation, TensorRT export
- `sim/isaac/` — Isaac Sim scenes and synthetic data scripts
- `sim/px4_gz/` PX4 SITL + Gazebo assets/launch
- `results/` benchmark tables + sim run manifests (no large binaries)
- `cad/` — SolidWorks SLDPRT/STLs for frame V2

## Quick Start
**Requirements:** Ubuntu 22.04, ROS 2 Humble, Python 3.10, CUDA GPU

**1. Clone and install dependencies:**
```bash
git clone https://github.com/JacobFlorio/scout-drone-cosmos-cookoff.git
cd scout-drone-cosmos-cookoff
pip install transformers torch pillow
```

**2. Test Cosmos on a threat frame:**
```bash
python3 test_cosmos_image.py data/frames/machete_frame.jpg
```

**3. Run the full ROS2 pipeline:**
```bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 run cosmos_agent threat_reasoner &
ros2 run cosmos_agent image_publisher
```

## Performance (Pre/Post TensorRT)
| Platform | Inference Time (Pre) | Inference Time (Post) | Notes |
|----|----|----|----|
| RTX 5080 (FP16) | ~18s/frame | ~10s/frame Development machine |
| Jetson Orin Nano | ~60s/frame | ~30s/frame Target (estimated) |

## Cosmos Reason 2 - Live Inference
- **Model:** nvidia/Cosmos-Reason2-2B
- **Status:** ✅ Live on GPU/Jetson
- **Input:** SIYI A8 frames
- **Output:** CoT reasoning -> parsed threat JSON

### Example Output on threat frame:
```
<think>
Okay, let's break this down. The video shows a person in a dark hoodie holding a long, thin object that looks like a knife or similar tool. They're leaning over a desk with a laptop and other items. The setting is indoors, maybe an office or home workspace. The person's posture is aggressive, with the object raised and pointed, which could indicate intent to harm. The presence of a knife or similar tool is a clear security concern because knives are dangerous weapons. The person's behavior, combined with the tool they're holding, suggests they might be preparing to attack someone or something. The desk setup includes a laptop and other items, which could be targets if the person is trying to cause damage. The lighting is dim, which might make it harder to see potential threats, but the overall scene still conveys a sense of danger. Since the person is alone and the video does not show others, it's likely a personal attack scenario. The threat level here is high because of the weapon and aggressive posture.
</think>
<answer>threat_level: high, action: alert</answer>
```

**Parsed JSON:** `{"threat_level": "high", "action": "alert", "confidence": 0.9}`

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
