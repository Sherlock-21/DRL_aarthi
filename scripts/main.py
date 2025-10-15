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


def input_creater() -> str:
    """Prompt user for natural language command."""
    raw_input = input("Enter your command (e.g., 'move in circle for 2 times') or 'quit' to exit: ")
    return raw_input.strip()


def load_qwen_model() -> tuple[AutoModelForCausalLM, AutoTokenizer]:
    """Load the Qwen model and tokenizer."""
    model_name = "Team-ACE/ToolACE-2-Llama-3.1-8B"
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    model = AutoModelForCausalLM.from_pretrained(model_name)
    return model, tokenizer

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
        {"role": "system", "content": f"{system_prompt}\n Current position of end effector (x,y,z) : {manipulator.position}\n Current gripper state : {manipulator.gripper_state}\n"},
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
        "gripper_close": manipulator.gripper_close,
        "gripper_open": manipulator.gripper_open,
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




if __name__ == "__main__":
    print("Loading Qwen model...")
    model, tokenizer = load_qwen_model()
    print("Model loaded successfully!\n")
    
    # Create system prompt
    system_prompt = system_prompt_hermone
    
    # Initialize manipulator (state persists across iterations)
    manipulator = MovementsTools()
    
    print("Manipulator initialized. Position: [0, 0, 0], Gripper: open\n")
    
    while True:
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
        print(f"Current gripper state: {manipulator.gripper_state}\n")
        print("-" * 50)  # Separator for readability