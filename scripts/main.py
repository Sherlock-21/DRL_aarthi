from modules import system_prompt_hermone
from ik_Aathi import ik as inverse_kinematics
from toolace_infer import  load_qwen_model
from transformers import AutoModelForCausalLM, AutoTokenizer
import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import JointState


class MovementsTools(Node):
    def __init__(self):
        super().__init__('manipulator_controller')
        self.position = [0, 0, 46]  # x, y, z
        
        self.t1= 90
        self.t2=90
        self.t3=90
        self.t4= 90
        self.t5 = 0


        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )


        # Create publisher for joint_states topic
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', qos_profile)

    def publish_joint_states(self, t1, t2, t3, t4, t5):
        # for i in range(10):
        #     i +=1
        #     msg = JointState()
        #     msg.header.stamp = self.get_clock().now().to_msg()
        #     msg.name = ['joint1', 'joint2', 'joint3', 'joint4','joint5', 'joint6']
        #     msg.position = [(t1/10)*i, (t2/10)*i, (t3/10)*i, 90.0, (t4/10)*i,t5]
        #     self.joint_state_publisher.publish(msg)
        #     self.get_logger().info(f'Published joint states: {msg.position}')
        #     time.sleep(1)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4','joint5', 'joint6']
        msg.position = [(t1), (t2), (t3), 90.0, (t4),t5]
        self.joint_state_publisher.publish(msg)
        self.get_logger().info(f'Published joint states: {msg.position}')
        time.sleep(1)

    def move_up(self):
        self.position[2] += 1
        t1, t2, t3, t4 = inverse_kinematics(self.position[0], self.position[1], self.position[2])
        self.publish_joint_states(t1, t2, t3, t4, self.t5)
        self.t1= t1
        self.t2=t2
        self.t3=t3
        self.t4= t4
        

        print(f"Moved up to position: {self.position}")
        print(f"Joint angles {t1}, {t2}, {t3}, {t4}, {self.t5}")

    def move_down(self):
        self.position[2] -= 1
        t1, t2, t3, t4 = inverse_kinematics(self.position[0], self.position[1], self.position[2])
        self.publish_joint_states(t1, t2, t3, t4, self.t5)
        self.t1= t1
        self.t2=t2
        self.t3=t3
        self.t4= t4
        
        print(f"Moved down to position: {self.position}")
        print(f"Joint angles {t1}, {t2}, {t3}, {t4}, {self.t5}")

    def move_left(self):
        self.position[0] -= 1
        t1, t2, t3, t4 = inverse_kinematics(self.position[0], self.position[1], self.position[2])
        self.publish_joint_states(t1, t2, t3, t4, self.t5)
        self.t1= t1
        self.t2=t2
        self.t3=t3
        self.t4= t4
        
        print(f"Moved left to position: {self.position}")
        print(f"Joint angles {t1}, {t2}, {t3}, {t4}, {self.t5}")

    def move_right(self):
        self.position[0] += 1
        t1, t2, t3, t4 = inverse_kinematics(self.position[0], self.position[1], self.position[2])
        self.publish_joint_states(t1, t2, t3, t4, self.t5)
        self.t1= t1
        self.t2=t2
        self.t3=t3
        self.t4= t4
       
        print(f"Moved right to position: {self.position}")
        print(f"Joint angles {t1}, {t2}, {t3}, {t4}, {self.t5}")

    def move_forward(self):
        self.position[1] += 1
        t1, t2, t3, t4 = inverse_kinematics(self.position[0], self.position[1], self.position[2])
        self.publish_joint_states(t1, t2, t3, t4, self.t5)
        self.t1= t1
        self.t2=t2
        self.t3=t3
        self.t4= t4
       
        print(f"Moved forward to position: {self.position}")
        print(f"Joint angles {t1}, {t2}, {t3}, {t4}, {self.t5}")

    def move_back(self):
        self.position[1] -= 1
        t1, t2, t3, t4 = inverse_kinematics(self.position[0], self.position[1], self.position[2])
        self.publish_joint_states(t1, t2, t3, t4, self.t5)
        self.t1= t1
        self.t2=t2
        self.t3=t3
        self.t4= t4
        
        print(f"Moved back to position: {self.position}")
        print(f"Joint angles {t1}, {t2}, {t3}, {t4}, {self.t5}")

    def gripper_close(self):
        self.t5 = 90
        self.publish_joint_states(self.t1,self.t2,self.t3,self.t4,self.t5)
        print("Gripper closed")

    def gripper_open(self):
        self.t5 = 0
        self.publish_joint_states(self.t1,self.t2,self.t3,self.t4,self.t5)
        print("Gripper opened")


def input_creater() -> str:
    """Prompt user for natural language command."""
    raw_input = input("Enter your command (e.g., 'move in circle for 2 times') or 'quit' to exit: ")
    return raw_input.strip()





def code_generate(
    system_prompt: str,
    user_input: str,
    model: AutoModelForCausalLM,
    tokenizer: AutoTokenizer,
) -> str:
    """
    Generates Python code based on system prompt and user input using Qwen model.

    Args:
        system_prompt (str): The system prompt with rules and available functions
        user_input (str): The user's natural language command
        model: The loaded Qwen model
        tokenizer: The loaded tokenizer

    Returns:
        str: Generated Python code
    """
    manipulator = MovementsTools()
    # Construct messages in chat format (similar to OpenAI API)
    messages = [
        {"role": "system", "content": f"{system_prompt}\n Current position of end effector (x,y,z) : {manipulator.position}\n Current gripper state : {manipulator.t5}\n"},
        {"role": "user", "content": f"{user_input}\n"}
    ]

    # Apply chat template
    text = tokenizer.apply_chat_template(
        messages,
        tokenize=False,
        add_generation_prompt=True
    )

    # Tokenize
    inputs = tokenizer([text], return_tensors="pt")

    # Generate
    outputs = model.generate(
        **inputs,
        max_new_tokens=8192,
        temperature=0.5,
        do_sample=True,
        pad_token_id=tokenizer.eos_token_id
    )

    # Decode output
    new_tokens = outputs[0][len(inputs['input_ids'][0]):]
    generated_text = tokenizer.decode(new_tokens, skip_special_tokens=True)

    # Extract only the assistant's response (code)
    # The generated text includes the entire conversation, we need only the last part
    if "<|im_start|>assistant" in generated_text:
        code = generated_text.split("<|im_start|>assistant")[-1].split("<|im_end|>")[0].strip()
    else:
        # Fallback: try to extract after the user message
        code = generated_text.split(user_input)[-1].strip()
    
    # Clean up code block markers if present
    if "```python" in code:
        code = code.split("```python", 1)[1].split("```", 1)[0].strip()
    elif "```" in code:
        code = code.split("```", 1)[1].split("```", 1)[0].strip()

    return code


def execute(
    generated_code: str,
    manipulator: MovementsTools,
) -> bool:
    """
    Executes the generated code with access to manipulator functions.

    Args:
        generated_code (str): The Python code to execute
        manipulator (MovementsTools): Instance of the manipulator controller

    Returns:
        bool: True if execution successful, False otherwise
    """
    # Create local execution environment with manipulator functions
    local_vars = {
        "move_up": manipulator.move_up,
        "move_down": manipulator.move_down,
        "move_left": manipulator.move_left,
        "move_right": manipulator.move_right,
        "move_forward": manipulator.move_forward,
        "move_back": manipulator.move_back,
        "gripper_close": manipulator.t5,
        "gripper_open": manipulator.t5,
    }

    print("\n=== GENERATED CODE ===")
    print(generated_code)
    print("\n=== GENERATED CODE ===")

    try:
        print("\n=== EXECUTING GENERATED CODE ===")
        exec(generated_code, {"range": range}, local_vars)
        print("\n=== EXECUTING GENERATED CODE ===")
        return True
    except Exception as e:
        print(f"\n=== EXECUTION FAILED ===")
        print(f"Error: {e}")
        return False


def main():
    rclpy.init()
    print("Loading Qwen model...")
    model, tokenizer = load_qwen_model()
    print("Model loaded successfully!\n")

    # Create system prompt
    system_prompt = system_prompt_hermone

    # Initialize manipulator (state persists across iterations)
    manipulator = MovementsTools()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(manipulator)

    print("Manipulator initialized. Position: [0, 0, 0], Gripper: open\n")

    try:
        while True:
            # Process ROS2 messages (if needed)
            rclpy.spin_once(manipulator, timeout_sec=0)

            # Get user input
            user_input = input_creater()

            # Check for quit condition
            if user_input.lower() == "quit":
                print("Exiting manipulator control loop.")
                break

            # Generate code
            print("\n=== GENERATING CODE ===")
            generated_code = code_generate(system_prompt, user_input, model, tokenizer)

            # Execute the generated code
            success = execute(generated_code, manipulator)

            # Print current state after execution (success or failure)
            print(f"\nCurrent manipulator position: {manipulator.position}")
            print(f"Current gripper state: {manipulator.t5}\n")
            print("-" * 50)  # Separator for readability
    finally:
        manipulator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
