import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from transformers import Qwen3VLForConditionalGeneration, Qwen3VLProcessor
import torch
import cv2
import json
from datetime import datetime

class CosmosThreatReasoner(Node):
    def __init__(self):
        super().__init__('cosmos_threat_reasoner')
        self.subscription = self.create_subscription(
            Image, '/siyi_a8/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/threat_reasoning', 10)
        self.bridge = CvBridge()
        self.processor = Qwen3VLProcessor.from_pretrained("nvidia/Cosmos-Reason2-2B")
        self.model = Qwen3VLForConditionalGeneration.from_pretrained(
            "nvidia/Cosmos-Reason2-2B", torch_dtype=torch.float16, device_map="auto", attn_implementation="sdpa"
        )
        self.frame_counter = 0
        self.process_every = 10  # Process every 10th frame to reduce latency
        self.get_logger().info('Cosmos loaded on GPU. Ready for frames.')

    def image_callback(self, msg):
        self.frame_counter += 1
        if self.frame_counter % self.process_every != 0:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        temp_path = f"/tmp/drone_frame_{datetime.now().strftime('%H%M%S')}.jpg"
        cv2.imwrite(temp_path, cv_image)

        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": temp_path},
                    {"type": "text", "text": "From drone camera feed: Analyze this image for security threat. Reason step-by-step using <think>full reasoning here</think><answer>final threat level and action</answer> format. Consider lighting, objects, behavior, spatial layout, potential intrusion risks."}
                ]
            }
        ]

        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        inputs = self.processor(text=[text], return_tensors="pt").to("cuda")

        with torch.no_grad():
            generated_ids = self.model.generate(
                **inputs,
                max_new_tokens=1024,
                do_sample=False,
                temperature=0.0,
                eos_token_id=self.processor.tokenizer.eos_token_id
            )

        generated_ids_trimmed = generated_ids[:, inputs.input_ids.shape[1]:]
        response = self.processor.batch_decode(generated_ids_trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]

        # Improved parsing from <answer> tag
        threat_level = "unknown"
        action = "monitor"
        try:
            answer_start = response.find("<answer>") + len("<answer>")
            answer_end = response.rfind("</answer>")
            if answer_start > len("<answer>") and answer_end > answer_start:
                answer_text = response[answer_start:answer_end].strip().lower()
                # More robust keyword extraction (expand with regex/NLP later)
                if any(word in answer_text for word in ["high", "severe", "dangerous", "intrusion", "breach", "threat"]):
                    threat_level = "high"
                    action = "investigate"  # or "alert owner", "patrol waypoint", "engage RTL"
                elif any(word in answer_text for word in ["medium", "moderate", "suspicious", "potential"]):
                    threat_level = "medium"
                    action = "monitor closely"
                elif any(word in answer_text for word in ["low", "none", "safe", "benign"]):
                    threat_level = "low"
                    action = "monitor"
                else:
                    threat_level = "low"
                    action = "monitor"
        except Exception as e:
            self.get_logger().warn(f"Parsing error: {str(e)}")

        reasoning_json = json.dumps({
            "threat_level": threat_level,
            "action": action,
            "reasoning": response[:500],  # truncate for payload
            "raw_response": response[:200]  # optional debug field
        })

        msg = String()
        msg.data = reasoning_json
        self.publisher.publish(msg)
        self.get_logger().info(f'Published reasoning: {response[:100]}...')

def main(args=None):
    rclpy.init(args=args)
    node = CosmosThreatReasoner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()