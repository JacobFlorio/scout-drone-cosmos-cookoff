#!/usr/bin/env python3
"""
Build training dataset by combining auto-labels with hand-crafted seed examples.
Ensures balanced threat level distribution for better fine-tuning.

Usage:
    python3 scripts/build_dataset.py --labels data/labels.jsonl --output data/train.jsonl
"""

import argparse
import json
import random
from pathlib import Path
from collections import Counter

# Hand-crafted seed examples with ideal reasoning for each threat level
# These use the real frames you have + teach the model the response format
SEED_EXAMPLES = [
    {
        "image": "data/frames/machete_frame.jpg",
        "threat_level": "high",
        "action": "alert",
        "confidence": 0.92,
        "reasoning": (
            "The scene shows a person carrying what appears to be a machete or large bladed tool. "
            "The individual is near a residential structure and the lighting suggests dusk or dawn, "
            "a common time for property intrusions. The person's posture and the object they're "
            "carrying present a clear security concern. The proximity to the building entrance "
            "and the nature of the object warrant immediate attention. This is a high-priority "
            "security event requiring the drone to hold position, begin recording, and alert the owner."
        ),
    },
    {
        "image": "data/frames/test_frame.jpg",
        "threat_level": "low",
        "action": "continue",
        "confidence": 0.85,
        "reasoning": (
            "The scene shows a typical residential environment with no people or vehicles present. "
            "The lighting conditions are normal and there are no signs of forced entry, broken "
            "windows, or disturbed property. No suspicious objects or unusual activity detected. "
            "The area appears secure and the drone should continue its normal patrol route."
        ),
    },
]


def augment_labels(labels: list, target_per_class: int = 50) -> list:
    """Balance dataset by upsampling underrepresented threat levels."""
    by_level = {}
    for label in labels:
        level = label["threat_level"]
        if level not in by_level:
            by_level[level] = []
        by_level[level].append(label)

    print(f"Original distribution: {dict(Counter(l['threat_level'] for l in labels))}")

    augmented = []
    for level, examples in by_level.items():
        if len(examples) >= target_per_class:
            augmented.extend(random.sample(examples, target_per_class))
        else:
            # Upsample with slight reasoning variations
            augmented.extend(examples)
            while len([a for a in augmented if a["threat_level"] == level]) < min(target_per_class, len(examples) * 3):
                ex = random.choice(examples).copy()
                augmented.append(ex)

    random.shuffle(augmented)
    print(f"Augmented distribution: {dict(Counter(l['threat_level'] for l in augmented))}")
    return augmented


def main():
    parser = argparse.ArgumentParser(description="Build fine-tuning dataset")
    parser.add_argument("--labels", default="data/labels.jsonl", help="Auto-labels JSONL")
    parser.add_argument("--output", default="data/train.jsonl", help="Output training JSONL")
    parser.add_argument("--balance", action="store_true", help="Balance threat level distribution")
    args = parser.parse_args()

    # Load auto-labels
    labels = []
    if Path(args.labels).exists():
        with open(args.labels) as f:
            for line in f:
                labels.append(json.loads(line))
        print(f"Loaded {len(labels)} auto-labels from {args.labels}")
    else:
        print(f"No auto-labels found at {args.labels}, using seed examples only")

    # Add seed examples
    for seed in SEED_EXAMPLES:
        if Path(seed["image"]).exists():
            labels.append(seed)
            print(f"Added seed: {seed['image']} → {seed['threat_level']}")

    if not labels:
        print("ERROR: No training data available!")
        return

    # Balance if requested
    if args.balance:
        labels = augment_labels(labels)

    # Write training dataset
    with open(args.output, "w") as f:
        for label in labels:
            # Ensure consistent format
            record = {
                "image": label["image"],
                "threat_level": label.get("threat_level", "low"),
                "action": label.get("action", "monitor"),
                "confidence": label.get("confidence", 0.5),
                "reasoning": label.get("reasoning", ""),
            }
            f.write(json.dumps(record) + "\n")

    print(f"\nTraining dataset: {len(labels)} examples → {args.output}")
    print(f"Distribution: {dict(Counter(l['threat_level'] for l in labels))}")


if __name__ == "__main__":
    main()
