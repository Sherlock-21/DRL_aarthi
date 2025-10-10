# main.py

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

    movement_functions_info = {
        "move_up": {
            "description": "Moves the manipulator up by 1 unit along the Z-axis",
            "usage": "Call this function to increase the vertical position",
            "syntax": "move_up()",
            "example": "move_up()  # No parameters needed",
            "parameters": "None"
        },
        "move_down": {
            "description": "Moves the manipulator down by 1 unit along the Z-axis",
            "usage": "Call this function to decrease the vertical position",
            "syntax": "move_down()",
            "example": "move_down()  # No parameters needed",
            "parameters": "None"
        },
        "move_left": {
            "description": "Moves the manipulator left by 1 unit along the X-axis",
            "usage": "Call this function to move the manipulator in the negative X direction",
            "syntax": "move_left()",
            "example": "move_left()  # No parameters needed",
            "parameters": "None"
        },
        "move_right": {
            "description": "Moves the manipulator right by 1 unit along the X-axis",
            "usage": "Call this function to move the manipulator in the positive X direction",
            "syntax": "move_right()",
            "example": "move_right()  # No parameters needed",
            "parameters": "None"
        },
        "move_front": {
            "description": "Moves the manipulator forward by 1 unit along the Y-axis",
            "usage": "Call this function to move the manipulator away from the base",
            "syntax": "move_front()",
            "example": "move_front()  # No parameters needed",
            "parameters": "None"
        },
        "move_back": {
            "description": "Moves the manipulator backward by 1 unit along the Y-axis",
            "usage": "Call this function to move the manipulator closer to the base",
            "syntax": "move_back()",
            "example": "move_back()  # No parameters needed",
            "parameters": "None"
        },
        "gripper_close": {
            "description": "Closes the gripper to grasp objects",
            "usage": "Call this function when you want to grip or hold an object",
            "syntax": "gripper_close()",
            "example": "gripper_close()  # No parameters needed",
            "parameters": "None"
        },
        "gripper_open": {
            "description": "Opens the gripper to release objects",
            "usage": "Call this function when you want to release a gripped object",
            "syntax": "gripper_open()",
            "example": "gripper_open()  # No parameters needed",
            "parameters": "None"
        }
    }

    @classmethod
    def get_function_info(cls, function_name):
        """Return the documentation for a movement function."""
        return cls.movement_functions_info.get(function_name, "Function not found.")


if __name__ == "__main__":
    print("hi")
