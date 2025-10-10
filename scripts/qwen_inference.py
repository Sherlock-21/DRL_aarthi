#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformers import AutoModelForCausalLM, AutoTokenizer

class Qwen25InferenceNode(Node):
    def __init__(self):
        super().__init__('qwen25_inference_node')
        
        self.declare_parameter('model_name', 'Qwen/Qwen2.5-0.5B-Instruct')
        self.declare_parameter('max_new_tokens', 5000)
        self.declare_parameter('temperature', 0.3)
        self.declare_parameter('top_p', 0.9)
        
        model_name = self.get_parameter('model_name').value
        self.max_new_tokens = self.get_parameter('max_new_tokens').value
        self.temperature = self.get_parameter('temperature').value
        self.top_p = self.get_parameter('top_p').value
        
        self.get_logger().info('Loading Qwen2.5-0.5B-Instruct model...')
        
        # Load tokenizer and model with device mapping
        self.tokenizer = AutoTokenizer.from_pretrained(
            model_name,
            trust_remote_code=True,
            force_download=True
        )
        self.model = AutoModelForCausalLM.from_pretrained(
            model_name,
            device_map='auto',
            trust_remote_code=True,
            force_download=True
        )
        self.model.eval()
        
        self.get_logger().info('Model loaded successfully')

        # ROS2 Subscriber and Publisher
        self.subscription = self.create_subscription(
            String,
            'query',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            'query_inference',
            10
        )
        self.get_logger().info('Qwen25 Inference Node ready')
    
    def listener_callback(self, msg):
        query = msg.data
        self.get_logger().info(f'Received query: {query}')
        try:
            response = self.generate_inference(query)
            resp_msg = String()
            resp_msg.data = response
            self.publisher.publish(resp_msg)
            self.get_logger().info(f'Published inference (first 100 chars): {response[:100]}')
        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')
    
    def generate_inference(self, query):
        messages = [
            { "role": "user", "content": query }
        ]
        inputs = self.tokenizer.apply_chat_template(
            messages,
            add_generation_prompt=True,
            tokenize=True,
            return_dict=True,
            return_tensors="pt"
        ).to(self.model.device)
        
        outputs = self.model.generate(
            **inputs,
            max_new_tokens=self.max_new_tokens,
            temperature=self.temperature,
            top_p=self.top_p,
            do_sample=True,
        )
        # Slice output tokens to get generated tokens only (exclude input tokens)
        generated_tokens = outputs[0][inputs["input_ids"].shape[-1]:]
        response = self.tokenizer.decode(generated_tokens, skip_special_tokens=True)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Qwen25InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
