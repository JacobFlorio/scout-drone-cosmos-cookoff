#!/usr/bin/env python3
"""
Auto-label synthetic frames with Cosmos Reason2-2B for fine-tuning dataset.
Runs inference on all frames and saves structured labels as JSONL.

Usage:
    python3 scripts/auto_label.py --frames data/synthetic/ --output data/labels.jsonl --limit 100
    nohup python3 scripts/auto_label.py --frames data/synthetic/ --output data/labels.jsonl &
"""

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

import torch
from PIL import Image
from transformers import Qwen3VLForConditionalGeneration, Qwen3VLProcessor

SECURITY_PROMPT = (
    "From drone camera feed: Analyze this image for security threats. "
    "Consider: people present, objects of concern (weapons, tools, vehicles), "
    "behavioral patterns (lurking, rushing, hiding), lighting conditions, "
    "entry points, and spatial layout. "
    "Reason step-by-step using <think>detailed reasoning</think> then "
    "<answer>threat_level: [none|low|medium|high|critical], action: [continue|monitor|hover|alert]</answer>"
)

THREAT_LEVELS = {"none", "low", "medium", "high", "critical"}
ACTIONS = {"continue", "monitor", "hover", "alert", "return_to_base"}


def parse_response(response: str) -> dict:
    """Parse Cosmos output into structured label."""
    result = {"threat_level": "low", "action": "monitor", "confidence": 0.5, "reasoning": ""}

    # Extract reasoning
    think_match = re.search(r"<think>(.*?)</think>", response, re.DOTALL)
    if think_match:
        result["reasoning"] = think_match.group(1).strip()

    # Extract answer
    answer_match = re.search(r"<answer>(.*?)</answer>", response, re.DOTALL)
    answer_text = answer_match.group(1).strip().lower() if answer_match else response.lower()

    # Parse threat level
    for level in THREAT_LEVELS:
        if level in answer_text:
            result["threat_level"] = level
            break

    # Parse action
    for action in ACTIONS:
        if action in answer_text:
            result["action"] = action
            break

    # Confidence based on reasoning quality
    word_count = len(result["reasoning"].split())
    result["confidence"] = min(0.5 + (word_count / 200), 0.95)

    return result


def main():
    parser = argparse.ArgumentParser(description="Auto-label frames with Cosmos Reason2")
    parser.add_argument("--frames", default="data/synthetic/", help="Frame directory")
    parser.add_argument("--output", default="data/labels.jsonl", help="Output JSONL path")
    parser.add_argument("--limit", type=int, default=0, help="Max frames (0=all)")
    parser.add_argument("--resume", action="store_true", help="Skip already-labeled frames")
    args = parser.parse_args()

    # Find frames
    frame_dir = Path(args.frames)
    frames = sorted(frame_dir.glob("*.png")) + sorted(frame_dir.glob("*.jpg"))
    if args.limit > 0:
        frames = frames[:args.limit]
    print(f"Found {len(frames)} frames")

    # Check already labeled
    already_labeled = set()
    if args.resume and os.path.exists(args.output):
        with open(args.output) as f:
            for line in f:
                d = json.loads(line)
                already_labeled.add(d.get("image"))
        print(f"Resuming — {len(already_labeled)} already labeled")

    frames = [f for f in frames if str(f) not in already_labeled]
    if not frames:
        print("All frames already labeled!")
        return

    # Load model
    print("Loading Cosmos Reason2-2B...")
    processor = Qwen3VLForConditionalGeneration is not None and Qwen3VLProcessor.from_pretrained("nvidia/Cosmos-Reason2-2B")
    processor = Qwen3VLProcessor.from_pretrained("nvidia/Cosmos-Reason2-2B")
    model = Qwen3VLForConditionalGeneration.from_pretrained(
        "nvidia/Cosmos-Reason2-2B",
        torch_dtype=torch.float16,
        device_map="auto",
        attn_implementation="sdpa",
    )
    model.eval()
    print("Model loaded!")

    # Process frames
    total_time = 0
    with open(args.output, "a") as out_f:
        for i, frame_path in enumerate(frames):
            start = time.time()

            messages = [{
                "role": "user",
                "content": [
                    {"type": "image", "image": str(frame_path)},
                    {"type": "text", "text": SECURITY_PROMPT},
                ]
            }]

            text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
            inputs = processor(text=[text], return_tensors="pt").to("cuda")

            with torch.no_grad():
                generated = model.generate(
                    **inputs, max_new_tokens=1024, do_sample=False, temperature=0.0,
                    eos_token_id=processor.tokenizer.eos_token_id,
                )

            trimmed = generated[:, inputs.input_ids.shape[1]:]
            response = processor.batch_decode(trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]

            elapsed = time.time() - start
            total_time += elapsed

            # Parse into structured label
            label = parse_response(response)
            label["image"] = str(frame_path)
            label["raw_response"] = response[:500]

            out_f.write(json.dumps(label) + "\n")
            out_f.flush()

            avg = total_time / (i + 1)
            remaining = avg * (len(frames) - i - 1) / 60
            print(f"[{i+1}/{len(frames)}] {frame_path.name}: "
                  f"{label['threat_level']} ({label['confidence']:.2f}) "
                  f"— {elapsed:.1f}s (ETA: {remaining:.0f}min)")

    print(f"\nDone! {len(frames)} frames labeled → {args.output}")
    print(f"Total time: {total_time/60:.1f} min")


if __name__ == "__main__":
    main()
