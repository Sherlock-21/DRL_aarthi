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
        name="move_front",
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
