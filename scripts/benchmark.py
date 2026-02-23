#!/usr/bin/env python3
"""Quick benchmark: base vs LoRA on test frames."""
import time, torch, json, sys
from transformers import Qwen3VLForConditionalGeneration, Qwen3VLProcessor
from peft import PeftModel

FRAMES = ["data/frames/machete_frame.jpg", "data/frames/test_frame.jpg"]
PROMPT = "From drone camera feed: Analyze this image for security threats. Reason step-by-step using <think>reasoning</think><answer>threat_level: [none|low|medium|high|critical], action: [continue|monitor|hover|alert]</answer>"
RUNS = 3

def run_inference(model, processor, frame, label=""):
    messages = [{"role": "user", "content": [{"type": "image", "image": frame}, {"type": "text", "text": PROMPT}]}]
    text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    inputs = processor(text=[text], return_tensors="pt").to("cuda")
    times = []
    for i in range(RUNS):
        torch.cuda.synchronize()
        t0 = time.time()
        with torch.no_grad():
            out = model.generate(**inputs, max_new_tokens=1024, do_sample=False, eos_token_id=processor.tokenizer.eos_token_id)
        torch.cuda.synchronize()
        times.append(time.time() - t0)
    trimmed = out[:, inputs.input_ids.shape[1]:]
    response = processor.batch_decode(trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]
    avg = sum(times) / len(times)
    print(f"  [{label}] {frame}: {avg:.2f}s avg ({RUNS} runs)")
    print(f"  Response: {response[:200]}...\n")
    return avg, response

print("Loading base model...")
processor = Qwen3VLProcessor.from_pretrained("nvidia/Cosmos-Reason2-2B")
base_model = Qwen3VLForConditionalGeneration.from_pretrained("nvidia/Cosmos-Reason2-2B", torch_dtype=torch.float16, device_map="auto", attn_implementation="sdpa")
base_model.eval()

print("\n=== BASE MODEL ===")
base_results = {}
for f in FRAMES:
    avg, resp = run_inference(base_model, processor, f, "BASE")
    base_results[f] = {"avg_time": avg, "response": resp[:300]}

print("Loading LoRA adapter...")
lora_model = PeftModel.from_pretrained(base_model, "models/cosmos_reason2_lora")
lora_model.eval()

print("\n=== LoRA MODEL ===")
lora_results = {}
for f in FRAMES:
    avg, resp = run_inference(lora_model, processor, f, "LoRA")
    lora_results[f] = {"avg_time": avg, "response": resp[:300]}

print("\n=== SUMMARY ===")
print(f"{'Frame':<30} {'Base (s)':<12} {'LoRA (s)':<12}")
print("-" * 54)
for f in FRAMES:
    print(f"{f:<30} {base_results[f]['avg_time']:<12.2f} {lora_results[f]['avg_time']:<12.2f}")

# Save results
results = {"base": base_results, "lora": lora_results, "gpu": torch.cuda.get_device_name(0), "runs_per_frame": RUNS}
with open("data/benchmark_results.json", "w") as f:
    json.dump(results, f, indent=2, default=str)
print("\nSaved to data/benchmark_results.json")
