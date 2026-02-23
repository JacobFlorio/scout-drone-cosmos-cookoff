#!/usr/bin/env python3
"""
Optimize Cosmos Reason2-2B with torch.compile and benchmark speedup.
Works without TensorRT — uses Inductor backend for GPU kernel fusion.

Usage:
    python3 scripts/optimize_compile.py
"""
import time, json, torch
from transformers import Qwen3VLForConditionalGeneration, Qwen3VLProcessor

FRAMES = ["data/frames/machete_frame.jpg", "data/frames/test_frame.jpg"]
PROMPT = "From drone camera feed: Analyze this image for security threats. Use <think>reasoning</think><answer>threat_level: [none|low|medium|high|critical], action: [continue|monitor|hover|alert]</answer>"
WARMUP = 1
RUNS = 3

def bench(model, processor, label):
    results = {}
    for frame in FRAMES:
        messages = [{"role": "user", "content": [
            {"type": "image", "image": frame},
            {"type": "text", "text": PROMPT}
        ]}]
        text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        inputs = processor(text=[text], return_tensors="pt").to("cuda")

        # Warmup
        for _ in range(WARMUP):
            with torch.no_grad():
                model.generate(**inputs, max_new_tokens=512, do_sample=False,
                             eos_token_id=processor.tokenizer.eos_token_id)

        # Timed runs
        times = []
        for _ in range(RUNS):
            torch.cuda.synchronize()
            t0 = time.time()
            with torch.no_grad():
                out = model.generate(**inputs, max_new_tokens=1024, do_sample=False,
                                   eos_token_id=processor.tokenizer.eos_token_id)
            torch.cuda.synchronize()
            times.append(time.time() - t0)

        avg = sum(times) / len(times)
        trimmed = out[:, inputs.input_ids.shape[1]:]
        resp = processor.batch_decode(trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]

        print(f"  [{label}] {frame}: {avg:.2f}s avg ({RUNS} runs)")
        results[frame] = {"avg_time": round(avg, 3), "times": [round(t, 3) for t in times]}
    return results

print("=" * 60)
print("Cosmos Reason2-2B Optimization Benchmark")
print(f"GPU: {torch.cuda.get_device_name(0)}")
print(f"VRAM: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
print(f"PyTorch: {torch.__version__}")
print("=" * 60)

# Load model
print("\nLoading model...")
processor = Qwen3VLProcessor.from_pretrained("nvidia/Cosmos-Reason2-2B")
model = Qwen3VLForConditionalGeneration.from_pretrained(
    "nvidia/Cosmos-Reason2-2B", torch_dtype=torch.float16,
    device_map="auto", attn_implementation="sdpa"
)
model.eval()

# --- Baseline FP16 ---
print("\n--- BASELINE (FP16 + SDPA) ---")
base_results = bench(model, processor, "FP16")

# --- torch.compile ---
print("\n--- COMPILING with torch.compile (inductor backend) ---")
print("This may take 1-2 min on first run...")
model.model = torch.compile(model.model, mode="reduce-overhead")
print("Compilation triggered. Running warmup + benchmark...")
compiled_results = bench(model, processor, "COMPILED")

# --- Summary ---
print("\n" + "=" * 60)
print(f"{'Frame':<35} {'FP16 (s)':<12} {'Compiled (s)':<12} {'Speedup':<10}")
print("-" * 69)
for f in FRAMES:
    b = base_results[f]["avg_time"]
    c = compiled_results[f]["avg_time"]
    speedup = b / c if c > 0 else 0
    print(f"{f:<35} {b:<12.2f} {c:<12.2f} {speedup:<10.2f}x")

# VRAM usage
vram_used = torch.cuda.max_memory_allocated() / 1e9
print(f"\nPeak VRAM: {vram_used:.2f} GB")

# Save
output = {
    "gpu": torch.cuda.get_device_name(0),
    "vram_gb": round(torch.cuda.get_device_properties(0).total_memory / 1e9, 1),
    "torch_version": torch.__version__,
    "baseline_fp16": base_results,
    "torch_compile": compiled_results,
    "peak_vram_gb": round(vram_used, 2),
    "warmup_runs": WARMUP,
    "timed_runs": RUNS,
}
with open("data/optimization_results.json", "w") as f:
    json.dump(output, f, indent=2)
print("Saved to data/optimization_results.json")
