# main.py

from modules import system_prompt_hermone
from transformers import AutoModelForCausalLM, AutoTokenizer
import math, time
class MovementsTools:
    def __init__(self):
        self.position = [0, 0, 0]  # x, y, z
        self.gripper_state = "open"

    def move_up(self):
        self.position[2] += 1
        print(f"Moved up to position: {self.position}")


    def move_down(self):
        self.position[2] -= 1
        print(f"Moved down to position: {self.position}")

    def move_left(self):
        self.position[0] -= 1
        print(f"Moved left to position: {self.position}")

    def move_right(self):
        self.position[0] += 1
        print(f"Moved right to position: {self.position}")

    def move_forward(self):
        self.position[1] += 1
        print(f"Moved forward to position: {self.position}")

    def move_back(self):
        self.position[1] -= 1
        print(f"Moved back to position: {self.position}")

    def gripper_close(self):
        self.gripper_state = "closed"
        print("Gripper closed")

    def gripper_open(self):
        self.gripper_state = "open"
        print("Gripper opened")


def input_creater():
    raw_input = input("Enter your command (e.g., 'move in circle for 2 times'): ")
    return raw_input


def load_qwen_model():
    model_name = "Qwen/Qwen2.5-0.5B-Instruct"
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    model = AutoModelForCausalLM.from_pretrained(model_name)
    return model, tokenizer

def code_generate(system_prompt, user_input, model, tokenizer):
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
    # Construct messages in chat format (similar to OpenAI API)
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": f"{user_input}\n\nGenerate ONLY the Python code to accomplish this task. Do not include explanations."}
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
        temperature=0.3,
        do_sample=True,
        pad_token_id=tokenizer.eos_token_id
    )
    
    # Decode output
    generated_text = tokenizer.decode(outputs[0], skip_special_tokens=True)
    
    # Extract only the assistant's response (code)
    # The generated text includes the entire conversation, we need only the last part
    if "<|im_start|>assistant" in generated_text:
        code = generated_text.split("<|im_start|>assistant")[-1].split("<|im_end|>")[0].strip()
    else:
        # Fallback: try to extract after the user message
        code = generated_text.split(user_input)[-1].strip()
    
    # Clean up code block markers if present
    # Clean up code block markers if present
# Clean up code block markers if present
    if "```python" in code:
        code = code.split("```python", 1)[1].split("```", 1)[0].strip()
    elif "```" in code:
        code = code.split("```", 1)[1].split("```", 1)[0].strip()

    
    return code



def execute(generated_code, manipulator):
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
        "gripper_close": manipulator.gripper_close,
        "gripper_open": manipulator.gripper_open,
    }
    
    print("\n=== EXECUTING GENERATED CODE ===")
    print(generated_code)
    
    
    try:
        exec(generated_code, {"range": range}, local_vars)
        print("\n=== EXECUTION SUCCESSFUL ===")
        return True
    except Exception as e:
        print(f"\n=== EXECUTION FAILED ===")
        print(f"Error: {e}")
        return False




if __name__ == "__main__":
    
    print("Loading Qwen model...")
    model, tokenizer = load_qwen_model()
    print("Model loaded successfully!\n")
    
    # Create system prompt
    system_prompt = system_prompt_hermone
    
    # Initialize manipulator
    manipulator = MovementsTools()
    
    # Get user input
    user_input = input_creater()
    
    # Generate code
    print("\n=== GENERATING CODE ===")
    generated_code = code_generate(system_prompt, user_input, model, tokenizer)
    
    # Execute the generated code
    success = execute(generated_code, manipulator)
    
    # Print final state
    print(f"\nFinal manipulator position: {manipulator.position}")
    print(f"Gripper state: {manipulator.gripper_state}")