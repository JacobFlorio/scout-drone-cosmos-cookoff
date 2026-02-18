from transformers import Qwen3VLForConditionalGeneration, AutoProcessor
from PIL import Image
import torch
import sys

image_path = sys.argv[1] if len(sys.argv) > 1 else "data/frames/machete_frame.jpg"

print("Loading processor...")
processor = AutoProcessor.from_pretrained("nvidia/Cosmos-Reason2-2B")
print("Loading model...")
model = Qwen3VLForConditionalGeneration.from_pretrained(
    "nvidia/Cosmos-Reason2-2B",
    torch_dtype=torch.float16,
    device_map="auto",
    attn_implementation="sdpa"
)
print("Model loaded on:", next(model.parameters()).device)

messages = [
    {
        "role": "user",
        "content": [
            {"type": "image", "image": image_path},
            {"type": "text", "text": "You are an autonomous security drone AI. Analyze this camera frame for physical security threats. Look carefully for: weapons (knives, machetes, firearms), aggressive posture, suspicious behavior, intruders, or dangerous objects. Reason step by step about what you observe. Output format: <think>detailed physical reasoning about what you see</think><answer>threat_level: [none/low/high], action: [monitor/investigate/alert]</answer>"}
        ]
    }
]

print("Preparing inputs...")
text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
image = Image.open(image_path)
inputs = processor(text=[text], images=[image], return_tensors="pt").to("cuda")

print("Generating...")
with torch.no_grad():
    generated_ids = model.generate(
        **inputs,
        max_new_tokens=1024,
        do_sample=False,
        eos_token_id=processor.tokenizer.eos_token_id
    )
generated_ids_trimmed = generated_ids[:, inputs.input_ids.shape[1]:]
response = processor.batch_decode(generated_ids_trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]
print("\n=== Cosmos Reason 2 Output ===\n")
print(response)
