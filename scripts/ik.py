import numpy as np

def inverse_kinematics(x, y, z, l1=1.0, l2=1.0, l3=1.0, l4=0.5):
    # Wrist position
    p_w = np.array([x - l4, y, z])
    
    # Theta 1
    theta1 = np.arctan2(p_w[1], p_w[0])
    
    # Project to plane
    r = np.sqrt(p_w[0]**2 + p_w[1]**2) - l1
    
    # Theta 3 using cosine law
    D = (r**2 + p_w[2]**2 - l2**2 - l3**2) / (2 * l2 * l3)
    theta3 = np.arccos(np.clip(D, -1.0, 1.0))
    
    # Theta 2
    theta2 = np.arctan2(p_w[2], r) - np.arctan2(l3 * np.sin(theta3), l2 + l3 * np.cos(theta3))
    
    # Theta 4 (simplified, assuming R_x alignment)
    theta4 = 0.0  # Adjust based on orientation if provided
    
    # Convert to degrees
    theta1_deg = np.degrees(theta1)
    theta2_deg = np.degrees(theta2)
    theta3_deg = np.degrees(theta3)
    theta4_deg = np.degrees(theta4)
    
    return theta1_deg, theta2_deg, theta3_deg, theta4_deg
    

# Example usage
x, y, z = 10, 10, 10
result = inverse_kinematics(x, y, z)
