import json
from dataclasses import dataclass


# Define available action modules
@dataclass
class ActionModule:
    name: str
    id: int
    description: str
    example_code: str
    argument_type: str
    argument_content : str


action_modules = [
    ActionModule(
        name="move_up",
        id=0,
        description="Moves the manipulator up by 1 unit along the Z-axis.",
        example_code="move_up()  # Move up by 1 unit",
        argument_type = "No arguments needed",
        argument_content = "None"
    ),
    ActionModule(
        name="move_down",
        id=1,
        description="Moves the manipulator down by 1 unit along the Z-axis.",
        example_code="move_down()  # Move down by 1 unit",
        argument_type = "No arguments needed",
        argument_content = "None"
    ),
    ActionModule(
        name="move_left",
        id=2,
        description="Moves the manipulator left by 1 unit along the X-axis.",
        example_code="move_left()  # Move left by 1 unit",
        argument_type = "No arguments needed",
        argument_content = "None"
    ),
    ActionModule(
        name="move_right",
        id=3,
        description="Moves the manipulator right by 1 unit along the X-axis.",
        example_code="move_right()  # Move right by 1 unit",
        argument_type = "No arguments needed",
        argument_content = "None"
    ),
    ActionModule(
        name="move_forward",
        id=4,
        description="Moves the manipulator forward by 1 unit along the Y-axis.",
        example_code="move_front()  # Move forward by 1 unit",
        argument_type = "No arguments needed",
        argument_content = "None"
    ),
    ActionModule(
        name="move_back",
        id=5,
        description="Moves the manipulator backward by 1 unit along the Y-axis.",
        example_code="move_back()  # Move backward by 1 unit",
        argument_type = "No arguments needed",
        argument_content = "None"
    ),
    ActionModule(
        name="gripper_open",
        id=6,
        description="Opens the gripper to release objects.",
        example_code="gripper_open()  # Open the gripper",
        argument_type = "No arguments needed",
        argument_content = "None"
    ),
    ActionModule(
        name="gripper_close",
        id=7,
        description="Closes the gripper to grasp objects.",
        example_code="gripper_close()  # Close the gripper",
        argument_type = "No arguments needed",
        argument_content = "None"
    ),
    ActionModule(
        name="messenger",
        id=8,
        description="If you want to say something to the user, use only this function",
        example_code="message('I cannnot perform this task')  # print this message",
        argument_type = "string",
        argument_content = "[any message to the user]"
    )
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
                "example usage": module.example_code,
                "arguments": {
                    "Type" : module.argument_type,
                    "Content" : module.argument_content
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
        Your primary role is to generate Python code to control the END EFFECTOR of the manipulator and achieve the user's requested task.
        The (x,y,z) is the position of the end effector.
        You must use only the provided movement and gripper functions.
        Do not invent new functions or use any functions not listed below.
        Use your intuition to outline a method first to achieve the user given task.
        Always generate code that is safe, clear, and efficient.
        Use loops and conditionals when necessary to achieve complex tasks.
        If the user requests a repeated or patterned movement, use appropriate control structures.
        If the user requests gripper actions, use gripper_open() or gripper_close() as needed.
        Do not include explanations or comments in the generated code unless explicitly requested.
        Do not move outside the allowed workspace unless the user specifies.
        If the user input is ambiguous, make reasonable assumptions and proceed.
        Always use integer steps for movement functions.
        Do not use floating point values for movement unless specified.
        If the user requests a shape (circle, square, etc.), approximate using available movement functions.
        If the user requests a pick-and-place, use gripper_close() to pick and gripper_open() to release.
        Try to relate ambigous language of direction representation in terms of the available functions (tools) to move the manipulator.
        If you feel the given task is not achievable OR the given task is out of the capability of a fixed robotic arm then just print out of capability task.
        Do not ask the user for clarification; generate code based on the input.
        Do not use external libraries or modules in the generated code.
        Do not use print statements unless the user requests output.
        Do not use recursion; use loops for repeated actions.
        Do not use global variables; keep all code within the main scope.
        Do not use class definitions in the generated code.
        Do not use try/except blocks unless the user requests error handling.
        Do not use input() statements in the generated code.
        YOU SHOULD GENERATE ONLY PYTHON CODE. Because whatever you generate will be given to python compiler. So if you generate anything other than python code then it will result in error. 
        IF YOU WANT TO TALK or CONVEY SOMETHING TO THE USER, USE print statements.
        All the tools available does not take any arguments, they can be called to perfom that action once according to the description.
        Do NOT write the code with arguments for the functions (tools).
        ALL Functions (tools) DO NOT TAKE ANY ARGUMENTS.
        For moving in repeated patterns, call the tools (functions) using looping statements to achieve the user given task.
        You should only generate python code and should NOT output any string. All the string should only be as comments after #.

Here are the available tools:
{modules_doc}


"""

print(system_prompt_hermone)
