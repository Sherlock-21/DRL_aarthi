import numpy as np

def ik(x_e, y_e, z_e, l1=10.0, l2=12.0, l3=10.0, l4=4.0, l5=10.0):
    r_proj = np.sqrt(x_e**2 + y_e**2)
    x_wrist = x_e - (l4 * x_e / r_proj if r_proj != 0 else 0)
    y_wrist = y_e - (l4 * y_e / r_proj if r_proj != 0 else 0)
    z_wrist = z_e
    
    theta_base = np.arctan2(y_wrist, x_wrist)  # radians
    
    r = np.sqrt(x_wrist**2 + y_wrist**2)
    z_arm = z_wrist - l1
    
    D = (r**2 + z_arm**2 - l2**2 - l3**2) / (2 * l2 * l3)
    D = np.clip(D, -1.0, 1.0)
    theta_elbow = np.arccos(D)
    
    phi = np.arctan2(z_arm, r)
    psi = np.arctan2(l3*np.sin(theta_elbow), l2 + l3*np.cos(theta_elbow))
    theta_shoulder = phi - psi
    
    theta_wrist = -(theta_shoulder + theta_elbow)
    

    theta1_deg = np.degrees(theta_base)
    theta2_deg = np.degrees(theta_shoulder)
    theta3_deg = np.degrees(theta_elbow)
    theta4_deg = np.degrees(theta_wrist)

    # if theta1_deg > 180:
    #     theta1_deg =180
    # elif theta1_deg < 0:
    #     theta1_deg =0

    if theta2_deg > 180:
        theta2_deg =180
    elif theta2_deg < 0:
        theta2_deg =0
    

    if theta3_deg > 180:
        theta3_deg =180
    else:
        theta3_deg +=90


    if theta4_deg > 180:
        theta4_deg =180
    else:
        theta4_deg += 180

    return theta1_deg, theta2_deg, theta3_deg, theta4_deg
    
   

# # Example call for testing (replace with your link lengths)
angles = ik(x_e=2, y_e=2, z_e=46, l1=10, l2=12, l3=10, l4=4)
print("Servo commands to send:", angles)
