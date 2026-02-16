from transformers import Qwen3VLForConditionalGeneration, AutoProcessor
import torch

print("Loading processor...")
processor = AutoProcessor.from_pretrained("nvidia/Cosmos-Reason2-2B")

print("Loading model (first run downloads ~4-5 GB)...")
model = Qwen3VLForConditionalGeneration.from_pretrained(
    "nvidia/Cosmos-Reason2-2B",
    torch_dtype=torch.float16,
    device_map="auto",
    attn_implementation="sdpa"  # recommended in card for efficiency
)

print("Model loaded on:", next(model.parameters()).device)

# Simple text-only test (no video yet)
messages = [
    {
        "role": "user",
        "content": [
            {"type": "text", "text": "From a drone's egocentric view: A person approaches a backyard fence at night with a backpack, looking around suspiciously. Reason step-by-step if this is a security threat and what the drone should do. Use <think>reasoning</think> <answer>final decision</answer> format."}
        ]
    }
]

print("Preparing inputs...")
text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
inputs = processor(text=[text], return_tensors="pt").to("cuda")

print("Generating...")
with torch.no_grad():
    generated_ids = model.generate(
        **inputs,
        max_new_tokens=1024,           # doubled again – should capture full reasoning
        do_sample=False,
        temperature=0.0,
        eos_token_id=processor.tokenizer.eos_token_id  # explicit stop at EOS
    )

generated_ids_trimmed = generated_ids[:, inputs.input_ids.shape[1]:]
response = processor.batch_decode(generated_ids_trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]

print("\n=== Cosmos Reason 2 Output ===\n")
print(response)