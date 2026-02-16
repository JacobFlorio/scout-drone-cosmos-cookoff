from transformers import AutoProcessor, AutoModelForCausalLM
import torch

model_id = "nvidia/Cosmos-Reason2-2B"  # 2B for speed; swap to -8B later if VRAM allows

print("Loading processor...")
processor = AutoProcessor.from_pretrained(model_id)

print("Loading model (first run will download ~4-5 GB)...")
model = AutoModelForCausalLM.from_pretrained(
    model_id,
    torch_dtype=torch.float16,
    device_map="auto",          # auto = GPU if available
    low_cpu_mem_usage=True
)

print("Model loaded on:", next(model.parameters()).device)

prompt = "From a drone's egocentric camera view, analyze this scene: a person is approaching a backyard fence at night with a backpack and looking around suspiciously. Reason step-by-step about whether this is a potential security threat and what action the drone should take."

messages = [{"role": "user", "content": [{"type": "text", "text": prompt}]}]

print("Preparing inputs...")
inputs = processor.apply_chat_template(
    messages,
    add_generation_prompt=True,
    return_tensors="pt"
).to("cuda")

print("Generating response...")
with torch.no_grad():
    outputs = model.generate(
        inputs,
        max_new_tokens=256,
        do_sample=False,           # deterministic for reproducibility
        temperature=0.0
    )

response = processor.decode(outputs[0], skip_special_tokens=True)
print("\n=== Cosmos Reason 2 Output ===\n")
print(response)