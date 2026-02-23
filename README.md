# Baldwin – Scout Drone (V1) – NVIDIA Cosmos Cookoff Entry

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) ![Jetson](https://img.shields.io/badge/Jetson-Orin_Nano-green) ![NVIDIA](https://img.shields.io/badge/NVIDIA-Cosmos_Reason2-76B900) ![PX4](https://img.shields.io/badge/PX4-Autopilot-orange)

**NVIDIA Cosmos Cookoff:** https://luma.com/nvidia-cosmos-cookoff?tk=auN5se

> **Demo Video:** [Watch the live demo →](#) Autonomous threat detection & reasoning with Cosmos Reason 2

---

## Project Overview

Edge-deployed physical AI agent (**Baldwin**) using NVIDIA Cosmos Reason 2-2B for real-time visual threat reasoning and autonomous drone response in home security. Unlike traditional object detection (YOLO, etc.), Baldwin *reasons* about what it sees — analyzing context, behavior, spatial layout, and object intent before deciding how to act.

The drone ingests live video from a SIYI A8 4K gimbal, runs chain-of-thought reasoning through Cosmos Reason 2 on a Jetson Orin Nano, publishes structured JSON threat assessments via ROS2, and executes autonomous flight commands through PX4 offboard control — **all on-device, no cloud**.

**Key innovation:** Closed-loop perception-to-action pipeline combining a vision-language model with real-time drone flight control. Scalable for farms, property security, wildlife monitoring, and defense applications. Built for privacy-first, cloud-free edge AI.

---

## Architecture

```
┌──────────────┐    RTSP     ┌───────────────────────┐   JSON    ┌──────────────────┐
│  SIYI A8 4K  │ ──────────► │  Jetson Orin Nano     │ ────────► │  PX4 Offboard    │
│  Gimbal Cam  │             │  Cosmos Reason2-2B    │           │  Flight Control  │
└──────────────┘             │  ROS2 threat_reasoner │           │  offboard_ctrl   │
                             └───────────────────────┘           └────────┬─────────┘
                                        │                                 │
                                   <think>                          uXRCE-DDS
                                   reasoning                              │
                                   </think>                        ┌──────▼─────────┐
                                   <answer>                        │  Pixhawk 6C    │
                                   threat + action                 │  PX4 Autopilot │
                                   </answer>                       └────────────────┘
```

---

## Performance Benchmarks

| Configuration | Platform | Inference Time | Notes |
|---|---|---|---|
| **FP16 + SDPA** | RTX 5080 (16GB) | **3.7 s/frame** | Benchmarked, warmed up, avg of 3 runs |
| FP16 + SDPA | Jetson Orin Nano 8GB | ~25-30 s/frame | Estimated (deployment in progress) |
| + TensorRT FP16 | Jetson Orin Nano 8GB | ~10-15 s/frame | Planned optimization |

### LoRA Fine-Tuning Results

| Metric | Value |
|---|---|
| Trainable params | 17.4M / 2.46B (0.71%) |
| Training loss | 1.317 → 0.057 (3 epochs) |
| Training time | 68 seconds on RTX 5080 |
| Training data | 102 examples (100 Isaac Sim + 2 real frames) |
| Adapter size | 66 MB (safetensors) |

---

## Hardware

| Component | Part |
|---|---|
| Flight Controller | Pixhawk 6C (PX4) |
| Onboard Computer | NVIDIA Jetson Orin Nano Dev Kit |
| Camera | SIYI A8 mini 4K gimbal (RTSP) |
| GPS / Telemetry | Holybro M10 GPS + SiK V3 |
| Frame | Custom "UFO" design — SolidWorks CAD, PETG-CF 3D printed |
| Battery | Tattu 22Ah 6S LiPo |
| Motors | T-Motor F90 1300kV |
| Props | Gemfan 7035 tri-blade |
| ESC | Tekko32 4-in-1 |

Optimized for endurance 7-8" patrols (40-60 min); modular for sensor variants (FLIR, LiDAR).

---

## Software Stack

- **Middleware:** ROS 2 Humble + PX4 uXRCE-DDS bridge
- **Simulation:** PX4 SITL + Gazebo, NVIDIA Isaac Sim 5.1.0
- **AI Model:** nvidia/Cosmos-Reason2-2B (vision-language model, physical reasoning)
- **Fine-tuning:** LoRA (PEFT) with auto-labeled Isaac Sim synthetic data
- **Optimization:** FP16 + SDPA attention (planned: TensorRT)

---

## Repo Layout

```
ros_ws/src/cosmos_agent/       — ROS 2 package
  cosmos_agent/
    threat_reasoner.py         — Cosmos Reason2 inference node → /threat_reasoning
    offboard_controller.py     — Threat → PX4 offboard commands (hover/patrol/alert)
    image_publisher.py         — Publish test frames to /siyi_a8/image_raw
scripts/
    auto_label.py              — Batch auto-label frames with Cosmos
    build_dataset.py           — Build balanced training JSONL
    finetune_lora.py           — LoRA fine-tuning pipeline
    benchmark.py               — Base vs LoRA inference benchmarking
    optimize_compile.py        — torch.compile / TensorRT optimization
    run_sitl.sh                — Launch PX4 SITL simulation
    run_offboard.sh            — Launch offboard controller
    run_agent.sh               — Launch full agent pipeline
models/cosmos_reason2_lora/    — LoRA adapter weights (66MB)
data/
    synthetic/                 — 500 Isaac Sim frames (Rivermark scene)
    frames/                    — Real threat scenario frames
    clips/                     — SIYI A8 footage
    labels.jsonl               — Auto-generated training labels
    train.jsonl                — Fine-tuning dataset (102 examples)
    benchmark_results.json     — RTX 5080 benchmark data
cad/                           — SolidWorks SLDPRT/STL files
```

---

## Quick Start

**Requirements:** Ubuntu 22.04, ROS 2 Humble, Python 3.10, CUDA GPU

### 1. Clone and install

```bash
git clone https://github.com/JacobFlorio/scout-drone-cosmos-cookoff.git
cd scout-drone-cosmos-cookoff
pip install transformers torch pillow peft trl datasets
```

### 2. Test Cosmos inference on a frame

```bash
python3 test_cosmos_image.py data/frames/machete_frame.jpg
```

### 3. Run full ROS2 pipeline (4 terminals)

```bash
# T1: PX4 SITL
bash scripts/run_sitl.sh

# T2: MicroXRCE-DDS agent
MicroXRCEAgent udp4 -p 8888

# T3: Build workspace & launch offboard controller
cd ros_ws && colcon build --packages-select px4_msgs cosmos_agent
source install/setup.bash
ros2 run cosmos_agent offboard_controller

# T4: Launch threat reasoner (or publish test threat)
ros2 run cosmos_agent threat_reasoner
# OR test manually:
ros2 topic pub --once /threat_reasoning std_msgs/String \
  "{data: '{\"threat_level\": \"high\", \"confidence\": 0.9, \"action\": \"investigate\"}'}"
```

### 4. Fine-tune with LoRA

```bash
python3 scripts/auto_label.py --frames data/synthetic/ --output data/labels.jsonl --limit 100
python3 scripts/build_dataset.py --labels data/labels.jsonl --output data/train.jsonl
python3 scripts/finetune_lora.py --data data/train.jsonl --epochs 3 --output models/cosmos_reason2_lora
```

---

## Cosmos Reason 2 – Live Inference Example

**Model:** nvidia/Cosmos-Reason2-2B | **Status:** ✅ Live on GPU

**Input:** SIYI A8 frame (machete threat scenario)

**Output:**
```
<think>
The video shows a person in a dark hoodie holding a long, thin object that looks like
a knife or similar tool. They're leaning over a desk with a laptop and other items.
The person's posture is aggressive, with the object raised and pointed, which could
indicate intent to harm. The presence of a knife or similar tool is a clear security
concern. Since the person is alone and the video does not show others, it's likely a
personal attack scenario. The threat level here is high because of the weapon and
aggressive posture.
</think>
<answer>threat_level: high, action: alert</answer>
```

**Parsed JSON → ROS2 → PX4:**
```json
{"threat_level": "high", "action": "alert", "confidence": 0.9}
```
→ Offboard controller receives → drone holds position → begins recording → alerts owner

---

## Project Status

### ✅ Complete
- [x] Custom 3D-printed PETG-CF drone frame (SolidWorks CAD)
- [x] ROS2 Humble + PX4 offboard MVP in SITL
- [x] Cosmos Reason2-2B live inference on GPU
- [x] SIYI A8 camera input (RTSP)
- [x] ROS2 threat reasoning pipeline (`threat_reasoner` node)
- [x] ROS2 offboard controller — threat level → PX4 flight commands (closed loop)
- [x] PX4 SITL action loop closed — high threat → hover, clear → resume patrol
- [x] Isaac Sim synthetic data generation (500 frames, Rivermark scene)
- [x] LoRA fine-tuning pipeline — auto-label → dataset → train (102 examples)
- [x] Benchmarks — 3.7 s/frame FP16+SDPA on RTX 5080

### 🔄 In Progress
- [ ] Demo video recording
- [ ] Jetson Orin Nano deployment
- [ ] TensorRT FP16/INT8 optimization
- [ ] Diverse Isaac Sim scenarios for balanced fine-tuning

---

## License

MIT

---

*Built by [Jacob Florio](https://github.com/JacobFlorio) for the [NVIDIA Cosmos Cookoff](https://luma.com/nvidia-cosmos-cookoff?tk=auN5se) — Submission #181*
