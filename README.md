# Scout Drone - NVIDIA Cosmos Cookoff Entry

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Jetson](https://img.shields.io/badge/Jetson-Orin%20Nano-green)](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
[![NVIDIA](https://img.shields.io/badge/NVIDIA-Cosmos%20Reason2-orange)](https://huggingface.co/nvidia/Cosmos-Reason2-2B)
[![PX4](https://img.shields.io/badge/PX4-v1.14-purple)](https://px4.io/)

**NVIDIA Cosmos Cookoff:** https://luma.com/nvidia-cosmos-cookoff?tk=auN5se
**Demo Video:**
[![Demo Video](https://img.youtube.com/vi/S2ii9YnADrw/0.jpg)](https://youtu.be/S2ii9YnADrw)
**Watch the 2-minute live demo → autonomous threat detection & reasoning on Jetson Orin Nano**

Edge-deployed physical AI agent using Cosmos Reason 2 deployed to Jetson Orin Nano for real-time threat reasoning and autonomous response in home security.

## Project Overview
This project deploys an edge AI agent for real-time home security using NVIDIA's Cosmos Reason 2 on a Jetson Orin Nano. The drone scouts autonomously, processes live camera feeds via a Vision Language Model (VLM) for physical threat reasoning, and triggers autonomous responses. 
Key innovation: Low-latency, on-device inference without cloud dependency, combining ROS 2, PX4 flight control, and Isaac Sim synthetic data for robust training. Built for secure, privacy-focused environments - with commercial scale potential for farm applications and property security. 

## Hardware:
- **Flight Controller:** Pixhawk 6C (PX4) with embedded IMU + barometer
- **Onboard Computer:** NVIDIA Jetson Orin Nano (Dev Kit)
- **Camera:** SIYI A8 mini 4k gimbal camera (RTSP: rtsp://[IP]:8554/main.264)
- **GPS:** Holybro M10 GPS
- **Motors:** T-motor F90 2806.5 1300kV x4
- **Propellers:** Gemfan 7035 Tri-blade carbon nylon x4
- **ESC:** Tekko32 F4 metal 4-in-1 65A ESC
- **Battery:** Tattu Plus 22000mAh 22.2V 25C 6S LiPo
- **Telemetry:** Holybro SiK Telemetry Radio V3
- **Frame:** Custom SolidWorks design, 3D printed PETG-CF

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

## Cosmos Reason 2 - Live Inference
- **Model:** nvidia/Cosmos-Reason2-2B
- **Status:** ✅ Live GPU inference (RTX 5080, FP16)
- **Input:** SIYI A8 camera frames
- **Output:** Chain-of-thought physical reasoning -> parsed threat level + action

### Real Output on Threat Scenario (SIYI A8 frame):
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
