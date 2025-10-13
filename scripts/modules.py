import json
from dataclasses import dataclass


# Define available action modules
@dataclass
class ActionModule:
    name: str
    id: int
    description: str
    example_code: str


action_modules = [
    ActionModule(
        name="move_up",
        id=0,
        description="Moves the manipulator up by 1 unit along the Z-axis.",
        example_code="move_up()  # Move up by 1 unit"
    ),
    ActionModule(
        name="move_down",
        id=1,
        description="Moves the manipulator down by 1 unit along the Z-axis.",
        example_code="move_down()  # Move down by 1 unit"
    ),
    ActionModule(
        name="move_left",
        id=2,
        description="Moves the manipulator left by 1 unit along the X-axis.",
        example_code="move_left()  # Move left by 1 unit"
    ),
    ActionModule(
        name="move_right",
        id=3,
        description="Moves the manipulator right by 1 unit along the X-axis.",
        example_code="move_right()  # Move right by 1 unit"
    ),
    ActionModule(
        name="move_forward",
        id=4,
        description="Moves the manipulator forward by 1 unit along the Y-axis.",
        example_code="move_front()  # Move forward by 1 unit"
    ),
    ActionModule(
        name="move_back",
        id=5,
        description="Moves the manipulator backward by 1 unit along the Y-axis.",
        example_code="move_back()  # Move backward by 1 unit"
    ),
    ActionModule(
        name="gripper_open",
        id=6,
        description="Opens the gripper to release objects.",
        example_code="gripper_open()  # Open the gripper"
    ),
    ActionModule(
        name="gripper_close",
        id=7,
        description="Closes the gripper to grasp objects.",
        example_code="gripper_close()  # Close the gripper"
    ),
]


# Convert action modules to Hermes-style tool format
def convert_to_hermes_tools(action_modules):
    tools = []
    for module in action_modules:
        tool = {
            "type": "function",
            "function": {
                "name": module.name,
                "description": module.description,
                "parameters": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            }
        }
        tools.append(tool)
    return tools


# Generate the tools JSON
tools = convert_to_hermes_tools(action_modules)

# Create the modules_doc XML format
modules_doc = "<tools>\n"
for tool in tools:
    modules_doc += json.dumps(tool, indent=2) + "\n"
modules_doc += "</tools>"

# System prompt with Hermes-style tool calling format
system_prompt_hermone = f"""


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
        All the tools available does not take any arguments, they can be called to perfom that action once. 
        Do NOT write the code with arguments for the functions (tools).
        Functions (tools) DO NOT TAKE ANY ARGUMENTS.
        For moving in repeated patterns, call the tools (functions) using looping statements to achieve the user given task.

Here are the available tools:
{modules_doc}


"""


