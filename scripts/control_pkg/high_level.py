import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from transformers import AutoTokenizer, AutoModelForCausalLM
import torch
import json
import sys

# --- Configuration ---
MODEL_PATH = "./gpt_finetuned" 
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

class LLMJointController(Node):
    def __init__(self):
        super().__init__('llm_joint_controller')
        self.get_logger().info(f"Loading model from {MODEL_PATH}...")

        try:
            self.tokenizer = AutoTokenizer.from_pretrained(MODEL_PATH)
            self.model = AutoModelForCausalLM.from_pretrained(
                MODEL_PATH,
                torch_dtype=torch.float16,
                device_map="auto"
            )
            # Set a pad token if the tokenizer doesn't have one
            if self.tokenizer.pad_token is None:
                self.tokenizer.pad_token = self.tokenizer.eos_token
                
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            sys.exit(1)

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info("JointState publisher created.")

    def get_angles_from_command(self, command: str) -> list[float] | None:
        """
        Runs inference using the model's chat template.
        """
        # 1. Define the conversation messages
        messages = [
            {
                "role": "system",
                
                "content": """
                You are a 5 DOF Robotic arm joint states generator. 
                Your task is to convert natural language commands into a JSON array of 5 joint angles in radians.
                Only output the JSON array."""
            },
            {
                "role": "user",
                "content": command
            }
        ]
        
        # 2. Apply the chat template
        inputs = self.tokenizer.apply_chat_template(
            messages,
            add_generation_prompt=True,
            return_tensors="pt"
        ).to(self.model.device)

        # 3. Generate the output
        outputs = self.model.generate(
            inputs, 
            max_new_tokens=50,
            pad_token_id=self.tokenizer.pad_token_id
        )
        
        # 4. Decode the response (only the newly generated tokens)
        response_text = self.tokenizer.decode(outputs[len(inputs):], skip_special_tokens=True)
        self.get_logger().info(f"Model raw output: '{response_text}'")

        # 5. Parse the JSON output
        try:
            start = response_text.find('[')
            end = response_text.rfind(']') + 1
            if start == -1 or end == 0:
                raise ValueError("JSON array not found in model output.")
                
            json_string = response_text[start:end]
            joint_angles = json.loads(json_string)
            
            if isinstance(joint_angles, list) and all(isinstance(x, (int, float)) for x in joint_angles):
                return [float(angle) for angle in joint_angles]
            else:
                raise TypeError("Parsed JSON is not a list of numbers.")

        except (json.JSONDecodeError, ValueError, TypeError) as e:
            self.get_logger().error(f"Failed to parse model output: {e}")
            return None

    def run_inference_loop(self):
        self.get_logger().info("Ready for commands. Type 'quit' to exit.")
        while rclpy.ok():
            try:
                command = input("Enter command: ")
                if command.lower() == 'quit':
                    break

                angles = self.get_angles_from_command(command)

                if angles:
                    if len(angles) != len(JOINT_NAMES):
                        self.get_logger().warn(f"Model returned {len(angles)} angles, expected {len(JOINT_NAMES)}. Publishing anyway.")

                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = JOINT_NAMES[:len(angles)]
                    msg.position = angles
                    
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published to /joint_states: {angles}")

            except (EOFError, KeyboardInterrupt):
                break

def main(args=None):
    rclpy.init(args=args)
    llm_node = LLMJointController()
    try:
        llm_node.run_inference_loop()
    except Exception as e:
        llm_node.get_logger().error(f"An exception occurred: {e}")
    finally:
        llm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
