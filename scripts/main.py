import math
from transformers import AutoModelForCausalLM, AutoTokenizer

# Manipulator control simulator
class Manipulator5DOF:
    def __init__(self):
        self.position = [0, 0]
        self.trajectory = []

    def move_up(self):
        self.position[1] += 1
        self.trajectory.append(list(self.position))
        print(f"Moved up to {self.position}")

    def move_down(self):
        self.position[1] -= 1
        self.trajectory.append(list(self.position))
        print(f"Moved down to {self.position}")

    def move_left(self):
        self.position[0] -= 1
        self.trajectory.append(list(self.position))
        print(f"Moved left to {self.position}")

    def move_right(self):
        self.position[0] += 1
        self.trajectory.append(list(self.position))
        print(f"Moved right to {self.position}")

# Initialize manipulator
manipulator = Manipulator5DOF()

# Define callable tools
tools = {
    "move_up": manipulator.move_up,
    "move_down": manipulator.move_down,
    "move_left": manipulator.move_left,
    "move_right": manipulator.move_right,
}

# Load Qwen 500M model locally
def load_qwen_500m():
    model_name = "Qwen/Qwen2.5-0.5B-Instruct"  # Or your local path
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    model = AutoModelForCausalLM.from_pretrained(model_name)
    return model, tokenizer

# Generate code from natural language using Qwen 500M
def generate_code_with_qwen(nl_command, model, tokenizer):
    # Construct prompt with function definitions and task
    prompt = f"""You are a robot control code generator. You have access to these functions:
- move_up(): moves the manipulator up by 1 unit
- move_down(): moves the manipulator down by 1 unit
- move_left(): moves the manipulator left by 1 unit
- move_right(): moves the manipulator right by 1 unit

Generate Python code using loops and conditionals to accomplish this task: {nl_command}

The code should use the available move functions to approximate the desired motion pattern.

Generate ONLY the Python code without explanations:
"""

    # Tokenize and generate
    inputs = tokenizer(prompt, return_tensors="pt")
    outputs = model.generate(
        **inputs,
        max_new_tokens=512,
        temperature=0.7,
        do_sample=True,
        pad_token_id=tokenizer.eos_token_id
    )
    
    # Decode output
    generated_text = tokenizer.decode(outputs[0], skip_special_tokens=True)
    
    # Extract code portion (after the prompt)
    code = generated_text[len(prompt):].strip()
    
    # Clean up code block markers if present
    
    return code

# Execute generated code safely
def execute_generated_code(code, tools):
    local_vars = tools.copy()
    try:
        exec(code, {"range": range, "math": math}, local_vars)
        print("\nCode executed successfully!")
        return True
    except Exception as e:
        print(f"\nError executing code: {e}")
        return False

# Main execution
if __name__ == "__main__":
    # Load model
    print("Loading Qwen 500M model...")
    model, tokenizer = load_qwen_500m()
    
    # Test commands
    test_commands = [
        "move the manipulator up 3 steps, then right 2 steps",
        "make a round",
        "make a square pattern with side length 4",
        "move in a circular pattern with radius 5"
    ]
    
    for command in test_commands:
        print(f"\n{'='*60}")
        print(f"Command: {command}")
        print(f"{'='*60}")
        
        # Reset manipulator position
        manipulator.position = [0, 0]
        manipulator.trajectory = []
        
        # Generate code
        print("\nGenerating code...")
        generated_code = generate_code_with_qwen(command, model, tokenizer)
        print("\nGenerated code:")
        print(generated_code)
        
        # Execute code
        print("\nExecuting code:")
        execute_generated_code(generated_code, tools)
        
        print(f"\nFinal position: {manipulator.position}")
        print(f"Trajectory points: {len(manipulator.trajectory)}")
