import numpy as np


def dh_transform(theta, d, a, alpha):
    """
    Create transformation matrix from DH parameters
    
    Args:
        theta: joint angle (radians)
        d: link offset along z-axis
        a: link length along x-axis
        alpha: link twist (radians)
    
    Returns:
        4x4 homogeneous transformation matrix
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ])
    
    return T


def forward_kinematics(theta1, theta2, theta3, theta4, l1=10.0, l2=12.0, l3=10.0, l4=4.0, l5=10.0):
    """
    Forward kinematics for 5-DOF manipulator
    
    DH Parameters:
    Joint 1: theta1, l1, 0, 90°
    Joint 2: theta2, 0, l2, 0°
    Joint 3: 90+theta3, 0, l3, 90°
    Joint 4: -90+theta4, 0, l4, 90°
    Joint 5: 0, 0, l5, 0°
    
    Args:
        theta1, theta2, theta3, theta4: joint angles in degrees
        l1-l5: link lengths
    
    Returns:
        4x4 transformation matrix from base to end-effector
        position (x, y, z)
        rotation matrix (3x3)
    """
    
    # Convert degrees to radians
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    theta3_rad = np.radians(theta3)
    theta4_rad = np.radians(theta4)
    
    # Joint 1: theta1, d=l1, a=0, alpha=90°
    T1 = dh_transform(theta1_rad, l1, 0, np.radians(90))
    
    # Joint 2: theta2, d=0, a=l2, alpha=0°
    T2 = dh_transform(theta2_rad, 0, l2, 0)
    
    # Joint 3: (90+theta3), d=0, a=l3, alpha=90°
    T3 = dh_transform(np.radians(90) + theta3_rad, 0, l3, np.radians(90))
    
    # Joint 4: (-90+theta4), d=0, a=l4, alpha=90°
    T4 = dh_transform(np.radians(-90) + theta4_rad, 0, l4, np.radians(90))
    
    # Joint 5: 0, d=0, a=l5, alpha=0°
    T5 = dh_transform(0, 0, l5, 0)
    
    # Calculate cumulative transformation
    T_total = T1 @ T2 @ T3 @ T4 @ T5
    
    # Extract position
    position = T_total[0:3, 3]
    
    # Extract rotation matrix
    rotation = T_total[0:3, 0:3]
    
    return T_total, position, rotation


def print_results(T, position, rotation):
    """Print forward kinematics results nicely"""
    print("Transformation Matrix:")
    print(T)
    print(f"\nEnd-effector Position:")
    print(f"x = {position[0]:.4f}")
    print(f"y = {position[1]:.4f}")
    print(f"z = {position[2]:.4f}")
    print(f"\nRotation Matrix:")
    print(rotation)


# Example usage
theta1, theta2, theta3, theta4 = 90,90,90,90  # degrees

T, pos, rot = forward_kinematics(theta1, theta2, theta3, theta4)
print_results(T, pos, rot)

# You can also get individual transformation matrices
print("\n--- Individual Transformations ---")
print(f"Joint angles: θ1={theta1}°, θ2={theta2}°, θ3={theta3}°, θ4={theta4}°")
