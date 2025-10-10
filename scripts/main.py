# main.py

from modules import action_modules
from transformers import AutoModelForCausalLM, AutoTokenizer
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

    def move_front(self):
        self.position[1] += 1
        print(f"Moved front to position: {self.position}")

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
        "move_front": manipulator.move_front,
        "move_back": manipulator.move_back,
        "gripper_close": manipulator.gripper_close,
        "gripper_open": manipulator.gripper_open,
    }
    
    print("\n=== EXECUTING GENERATED CODE ===")
    print(generated_code)
    print("\n=== EXECUTION OUTPUT ===")
    
    try:
        exec(generated_code, {"range": range}, local_vars)
        print("\n=== EXECUTION SUCCESSFUL ===")
        return True
    except Exception as e:
        print(f"\n=== EXECUTION FAILED ===")
        print(f"Error: {e}")
        return False



# Generate system prompt with imported action modules
def create_system_prompt():
    # Build the modules documentation string
    modules_doc = "\n".join([
        f"{module.name}(): {module.description}\n  Example: {module.example_code}"
        for module in action_modules
    ])
    
    system_prompt = f"""
        You are an embodied AI agent for a robotic manipulator.
        Your primary role is to generate Python code to control the manipulator and achieve the user's requested task.
        You must use only the provided movement and gripper functions.
        Do not invent new functions or use any functions not listed below.
        Always generate code that is safe, clear, and efficient.
        Use loops and conditionals when necessary to achieve complex tasks.
        If the user requests a repeated or patterned movement, use appropriate control structures.
        If the user requests gripper actions, use gripper_open() or gripper_close() as needed.
        Do not include explanations or comments in the generated code unless explicitly requested.
        Assume the manipulator starts at position [0, 0, 0] and gripper is open.
        Do not move outside the allowed workspace unless the user specifies.
        If the user input is ambiguous, make reasonable assumptions and proceed.
        Always use integer steps for movement functions.
        Do not use floating point values for movement unless specified.
        If the user requests a shape (circle, square, etc.), approximate using available movement functions.
        If the user requests a pick-and-place, use gripper_close() to pick and gripper_open() to release.
        Do not ask the user for clarification; generate code based on the input.
        Do not use external libraries or modules in the generated code.
        Do not use print statements unless the user requests output.
        Do not use recursion; use loops for repeated actions.
        Do not use global variables; keep all code within the main scope.
        Do not use class definitions in the generated code.
        Do not use try/except blocks unless the user requests error handling.
        Do not use input() statements in the generated code.

        Below are the available functions and their documentation:

        {modules_doc}

        Always follow these rules and use only the functions listed above to generate code for the manipulator.
"""
    return system_prompt


# Create the system prompt
system_prompt = create_system_prompt()


if __name__ == "__main__":
    
    print("Loading Qwen model...")
    model, tokenizer = load_qwen_model()
    print("Model loaded successfully!\n")
    
    # Create system prompt
    system_prompt = create_system_prompt()
    
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