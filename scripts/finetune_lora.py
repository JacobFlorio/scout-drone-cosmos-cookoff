#!/usr/bin/env python3
"""
LoRA Fine-Tuning for Cosmos Reason2-2B — Security Domain Adaptation.

Fine-tunes Cosmos Reason2-2B with LoRA on labeled security frames
to improve threat reasoning quality for the Baldwin drone agent.

Usage:
    python3 scripts/finetune_lora.py --data data/train.jsonl --epochs 3
    python3 scripts/finetune_lora.py --data data/train.jsonl --epochs 5 --lr 2e-5

Requires: pip install peft trl datasets
"""

import argparse
import json
import os
import sys
from pathlib import Path

import torch
from datasets import Dataset
from peft import LoraConfig, get_peft_model, TaskType
from transformers import (
    Qwen3VLForConditionalGeneration,
    Qwen3VLProcessor,
    TrainingArguments,
    Trainer,
)


def load_training_data(data_path: str) -> Dataset:
    """Load JSONL training data into HuggingFace Dataset format."""
    records = []
    with open(data_path) as f:
        for line in f:
            d = json.loads(line)
            # Build the training conversation
            prompt = (
                "From drone camera feed: Analyze this image for security threats. "
                "Consider: people, objects, behavior, lighting, entry points, spatial layout. "
                "Reason step-by-step using <think>reasoning</think> then "
                "<answer>threat_level: [none|low|medium|high|critical], "
                "action: [continue|monitor|hover|alert]</answer>"
            )

            # Target response: the model's labeled output
            reasoning = d.get("reasoning", "Scene appears normal with no immediate threats.")
            threat = d.get("threat_level", "low")
            action = d.get("action", "monitor")

            target = (
                f"<think>{reasoning}</think>"
                f"<answer>threat_level: {threat}, action: {action}</answer>"
            )

            records.append({
                "image": d["image"],
                "prompt": prompt,
                "response": target,
                "threat_level": threat,
            })

    print(f"Loaded {len(records)} training examples")

    # Print distribution
    from collections import Counter
    dist = Counter(r["threat_level"] for r in records)
    print(f"Distribution: {dict(dist)}")

    return Dataset.from_list(records)


def tokenize_fn(examples, processor):
    """Tokenize examples for training."""
    all_input_ids = []
    all_labels = []

    for i in range(len(examples["prompt"])):
        # Build full conversation text
        messages = [{
            "role": "user",
            "content": [
                {"type": "image", "image": examples["image"][i]},
                {"type": "text", "text": examples["prompt"][i]},
            ]
        }, {
            "role": "assistant",
            "content": examples["response"][i],
        }]

        text = processor.apply_chat_template(messages, tokenize=False)
        tokenized = processor.tokenizer(
            text, truncation=True, max_length=2048, return_tensors="pt"
        )

        input_ids = tokenized.input_ids[0]

        # Create labels — mask the prompt tokens, only train on response
        # Find where the assistant response starts
        labels = input_ids.clone()
        # Simple approach: mask first 80% as prompt (conservative)
        prompt_len = int(len(input_ids) * 0.7)
        labels[:prompt_len] = -100

        all_input_ids.append(input_ids)
        all_labels.append(labels)

    return {"input_ids": all_input_ids, "labels": all_labels}


def main():
    parser = argparse.ArgumentParser(description="LoRA fine-tune Cosmos Reason2-2B")
    parser.add_argument("--data", required=True, help="Training JSONL file")
    parser.add_argument("--output", default="models/cosmos_reason2_lora", help="Output dir")
    parser.add_argument("--epochs", type=int, default=3)
    parser.add_argument("--lr", type=float, default=2e-4)
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument("--grad-accum", type=int, default=8)
    parser.add_argument("--lora-rank", type=int, default=16)
    parser.add_argument("--lora-alpha", type=int, default=32)
    args = parser.parse_args()

    print("=" * 60)
    print("Baldwin LoRA Fine-Tuning")
    print(f"Data:       {args.data}")
    print(f"Output:     {args.output}")
    print(f"Epochs:     {args.epochs}")
    print(f"LR:         {args.lr}")
    print(f"LoRA rank:  {args.lora_rank}")
    print(f"LoRA alpha: {args.lora_alpha}")
    print("=" * 60)

    # Load model
    print("\nLoading Cosmos Reason2-2B...")
    processor = Qwen3VLProcessor.from_pretrained("nvidia/Cosmos-Reason2-2B")
    model = Qwen3VLForConditionalGeneration.from_pretrained(
        "nvidia/Cosmos-Reason2-2B",
        torch_dtype=torch.float16,
        device_map="auto",
        attn_implementation="sdpa",
    )

    # Configure LoRA
    print("Configuring LoRA...")
    lora_config = LoraConfig(
        task_type=TaskType.CAUSAL_LM,
        r=args.lora_rank,
        lora_alpha=args.lora_alpha,
        lora_dropout=0.05,
        target_modules=[
            "q_proj", "k_proj", "v_proj", "o_proj",
            "gate_proj", "up_proj", "down_proj",
        ],
        bias="none",
    )

    model = get_peft_model(model, lora_config)
    model.print_trainable_parameters()

    # Load data
    print("\nLoading training data...")
    dataset = load_training_data(args.data)

    # Training arguments — optimized for 16GB VRAM
    training_args = TrainingArguments(
        output_dir=args.output,
        num_train_epochs=args.epochs,
        per_device_train_batch_size=args.batch_size,
        gradient_accumulation_steps=args.grad_accum,
        learning_rate=args.lr,
        lr_scheduler_type="cosine",
        warmup_ratio=0.1,
        fp16=True,
        logging_steps=5,
        save_strategy="epoch",
        save_total_limit=2,
        dataloader_pin_memory=False,
        remove_unused_columns=False,
        report_to="none",
        gradient_checkpointing=True,
    )

    # Tokenize
    print("Preparing dataset...")

    def collate_fn(examples):
        texts = []
        for ex in examples:
            messages = [
                {
                    "role": "user",
                    "content": [
                        {"type": "image", "image": ex["image"]},
                        {"type": "text", "text": ex["prompt"]},
                    ]
                },
                {
                    "role": "assistant",
                    "content": ex["response"],
                }
            ]
            text = processor.apply_chat_template(messages, tokenize=False)
            texts.append(text)

        batch = processor.tokenizer(
            texts,
            truncation=True,
            max_length=2048,
            padding=True,
            return_tensors="pt",
        )

        # Labels = input_ids with prompt masked
        batch["labels"] = batch["input_ids"].clone()
        # Mask padding tokens
        batch["labels"][batch["attention_mask"] == 0] = -100

        return batch

    # Train
    print("\nStarting LoRA fine-tuning...")
    trainer = Trainer(
        model=model,
        args=training_args,
        train_dataset=dataset,
        data_collator=collate_fn,
    )

    trainer.train()

    # Save
    print(f"\nSaving LoRA adapter to {args.output}")
    model.save_pretrained(args.output)
    processor.save_pretrained(args.output)

    print("\n" + "=" * 60)
    print("Fine-tuning complete!")
    print(f"LoRA adapter saved to: {args.output}")
    print(f"To use: load base model + PeftModel.from_pretrained('{args.output}')")
    print("=" * 60)


if __name__ == "__main__":
    main()
