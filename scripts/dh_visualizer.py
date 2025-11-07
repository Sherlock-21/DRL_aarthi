import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1. Define the D-H Transformation Matrix Function
# --------------------------------------------------
def dh_transform_matrix(alpha, a, d, theta):
    """
    Computes the transformation matrix from D-H parameters.
    Alpha and theta must be in radians.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),              d],
        [0,              0,                           0,                          1]
    ])

# 2. Define Your Manipulator's Kinematics
# --------------------------------------------------
def forward_kinematics(dh_params, angles):
    """
    Calculates the position of each joint in 3D space.
    Returns a list of 3D coordinates for each joint.
    """
    # Base transform from world to the first joint's frame
    T_0_1 = dh_transform_matrix(0, 0, dh_params[0][2], angles[0])
    
    # Subsequent transforms
    T_1_2 = dh_transform_matrix(np.deg2rad(dh_params[1][0]), dh_params[1][1], dh_params[1][2], angles[1])
    T_2_3 = dh_transform_matrix(np.deg2rad(dh_params[2][0]), dh_params[2][1], dh_params[2][2], angles[2])
    T_3_4 = dh_transform_matrix(np.deg2rad(dh_params[3][0]), dh_params[3][1], dh_params[3][2], angles[3])
    T_4_5 = dh_transform_matrix(np.deg2rad(dh_params[4][0]), dh_params[4][1], dh_params[4][2], 0) # Final link has no rotation variable

    # Cumulative transformations
    T_0_2 = T_0_1 @ T_1_2
    T_0_3 = T_0_2 @ T_2_3
    T_0_4 = T_0_3 @ T_3_4
    T_0_5 = T_0_4 @ T_4_5

    # Extract joint positions from the transformation matrices
    p0 = np.array([0, 0, 0])
    p1 = T_0_1[:3, 3]
    p2 = T_0_2[:3, 3]
    p3 = T_0_3[:3, 3]
    p4 = T_0_4[:3, 3]
    p5 = T_0_5[:3, 3]
    
    return [p0, p1, p2, p3, p4, p5]

# 3. Define Parameters and Plot
# --------------------------------------------------
if __name__ == '__main__':
    # Your D-H Table: [alpha (deg), a, d, theta (placeholder)]
    # The first row [0, 0, l0, 0] is handled slightly differently as it's a fixed base.
    # We'll treat theta1 as part of the first transform for simplicity here.
    l0, l1, l2, l3, l4 = 0.1, 0.05, 0.2, 0.15, 0.1
    dh_parameters = [
        [0,   0,  l0, 0],       # Base offset, handled by d in the first transform
        [90,  0,  l1, 0],       # Link 1
        [0,   l2, 0,  0],       # Link 2
        [0,   l3, 0,  0],       # Link 3
        [0,   l4, 0,  0]        # Link 4 (End-effector)
    ]
    
    # Define a joint configuration 'q' in radians.
    # [theta1, theta2, theta3, theta4]
    joint_angles = np.array([np.deg2rad(45), np.deg2rad(45), np.deg2rad(45), np.deg2rad(-45)])
    
    # Calculate the 3D coordinates of each joint
    joint_positions = forward_kinematics(dh_parameters, joint_angles)
    
    # Extract X, Y, Z coordinates for plotting
    x_coords = [p[0] for p in joint_positions]
    y_coords = [p[1] for p in joint_positions]
    z_coords = [p[2] for p in joint_positions]
    
    # Create the 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the links as a line
    ax.plot(x_coords, y_coords, z_coords, 'o-', markersize=8, label='Manipulator Links')
    
    # Plot the joints as points
    ax.scatter(x_coords, y_coords, z_coords, c='red', s=100, label='Joints')

    # Setting labels and title
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('4-DOF Manipulator Visualization')
    
    # Set axis limits to be equal for a proper aspect ratio
    max_range = np.array([x_coords, y_coords, z_coords]).max() - np.array([x_coords, y_coords, z_coords]).min()
    mid_x = (max(x_coords) + min(x_coords)) * 0.5
    mid_y = (max(y_coords) + min(y_coords)) * 0.5
    mid_z = (max(z_coords) + min(z_coords)) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    ax.legend()
    ax.grid(True)
    plt.show()

