import numpy as np


def inverse_kinematics(x, y, z, roll=0, pitch=0, yaw=0, l1=10.0, l2=12.0, l3=10.0, l4=4.0, l5=10.0):
    """
    Inverse kinematics for 5-DOF manipulator
    DH Parameters:
    Joint 1: theta1, l1, 0, 90°
    Joint 2: theta2, 0, l2, 0°
    Joint 3: 90+theta3, 0, l3, 90°
    Joint 4: -90+theta4, 0, l4, 90°
    Joint 5: 0, 0, l5, 0°
    
    Args:
        x, y, z: End-effector position
        roll, pitch, yaw: End-effector orientation (optional, in radians)
        l1-l5: Link lengths
    
    Returns:
        theta1, theta2, theta3, theta4 in degrees
    """
    
    # Calculate end-effector rotation matrix from roll-pitch-yaw
    R = rotation_matrix_from_euler(roll, pitch, yaw)
    
    # Wrist center position (subtract l5 along tool axis - last column of R)
    # For your manipulator, l5 is along the final z-axis direction
    p_wrist = np.array([
        x - l5 * R[0, 2],
        y - l5 * R[1, 2],
        z - l5 * R[2, 2]
    ])
    
    # Joint 1: Base rotation
    theta1 = np.arctan2(p_wrist[1], p_wrist[0])
    
    # Calculate distance in x-y plane to wrist center
    r = np.sqrt(p_wrist[0]**2 + p_wrist[1]**2)
    
    # Height from joint 1 to wrist center
    s = p_wrist[2] - l1
    
    # Distance from joint 2 to wrist center
    D = np.sqrt(r**2 + s**2)
    
    # Check if target is reachable
    if D > (l2 + l3) or D < abs(l2 - l3):
        print(f"Warning: Target unreachable. D={D:.2f}, l2+l3={l2+l3:.2f}")
        return None
    
    # Joint 3: Elbow angle using cosine law
    # Note: theta3 in DH is (90 + theta3), so we solve for the actual joint angle
    cos_theta3 = (D**2 - l2**2 - l3**2) / (2 * l2 * l3)
    cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
    
    # Two solutions: elbow up (+) and elbow down (-)
    theta3 = np.arccos(cos_theta3)  # Taking positive solution (elbow up)
    # For elbow down: theta3 = -np.arccos(cos_theta3)
    
    # Joint 2: Shoulder angle
    beta = np.arctan2(s, r)
    gamma = np.arctan2(l3 * np.sin(theta3), l2 + l3 * np.cos(theta3))
    theta2 = beta - gamma
    
    # Joint 4: Wrist orientation
    # Calculate R_0^3 (rotation from base to joint 3)
    R_0_1 = np.array([
        [np.cos(theta1), -np.sin(theta1), 0],
        [np.sin(theta1), np.cos(theta1), 0],
        [0, 0, 1]
    ]) @ np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])  # Rz(theta1) * Rx(90)
    
    R_1_2 = np.array([
        [np.cos(theta2), -np.sin(theta2), 0],
        [np.sin(theta2), np.cos(theta2), 0],
        [0, 0, 1]
    ])  # Rz(theta2)
    
    theta3_dh = theta3 + np.pi/2  # DH parameter is (90 + theta3)
    R_2_3 = np.array([
        [np.cos(theta3_dh), -np.sin(theta3_dh), 0],
        [np.sin(theta3_dh), np.cos(theta3_dh), 0],
        [0, 0, 1]
    ]) @ np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])  # Rz(90+theta3) * Rx(90)
    
    R_0_3 = R_0_1 @ R_1_2 @ R_2_3
    
    # Remaining rotation needed
    R_3_5 = R_0_3.T @ R
    
    # Extract theta4 from R_3_5
    # DH parameter for joint 4 is (-90 + theta4)
    theta4 = np.arctan2(R_3_5[1, 2], R_3_5[0, 2]) + np.pi/2
    
    # Convert to degrees
    theta1_deg = np.degrees(theta1)
    theta2_deg = np.degrees(theta2)
    theta3_deg = np.degrees(theta3)
    theta4_deg = np.degrees(theta4)
    
    return theta1_deg, theta2_deg, theta3_deg, theta4_deg


def rotation_matrix_from_euler(roll, pitch, yaw):
    """
    Create rotation matrix from Euler angles (ZYX convention)
    """
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    return Rz @ Ry @ Rx


# Example usage
x, y, z = 0, 0, -2
result = inverse_kinematics(x, y, z)

if result is not None:
    print(f"Joint angles (degrees):")
    print(f"Theta1: {result[0]:.2f}°")
    print(f"Theta2: {result[1]:.2f}°")
    print(f"Theta3: {result[2]:.2f}°")
    print(f"Theta4: {result[3]:.2f}°")
else:
    print("Target position unreachable")
