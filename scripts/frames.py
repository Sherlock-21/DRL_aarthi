import matplotlib.pyplot as plt
import numpy as np

def draw_frame(ax, origin, R, name, length=0.7):
    """
    Draws a coordinate frame at a given origin with orientation R
    ax: matplotlib axes
    origin: [x, y] position
    R: 2x2 rotation matrix
    name: string label
    length: axis length
    """
    x_axis = origin + length * R[:,0]
    y_axis = origin + length * R[:,1]
    # X axis (red)
    ax.arrow(origin[0], origin[1], x_axis[0]-origin[0], x_axis[1]-origin[1],
             head_width=0.1, color='r', length_includes_head=True)
    # Y axis (green)
    ax.arrow(origin[0], origin[1], y_axis[0]-origin[0], y_axis[1]-origin[1],
             head_width=0.1, color='g', length_includes_head=True)
    ax.text(origin[0]+0.1, origin[1]+0.1, name, fontsize=12)

# --- Robot geometry setup ---
link_lengths = [2.0, 1.5, 1.0] # Replace with your lengths
joint_angles = [np.deg2rad(45), np.deg2rad(30), np.deg2rad(-20)] # Frame angles (in radians)
origins = [np.array([0,0])] # Will accumulate joint positions here
rotations = [np.eye(2)] # Will accumulate joint rotations here

# Compute successive origins and rotations using DH convention (in 2D for visualization)
for i in range(3):
    # Rotation matrix for this joint
    R = np.array([
        [np.cos(joint_angles[i]), -np.sin(joint_angles[i])],
        [np.sin(joint_angles[i]),  np.cos(joint_angles[i])]
    ])
    R_total = rotations[-1] @ R
    # Next origin (tip of this link)
    new_origin = origins[-1] + R_total @ np.array([link_lengths[i], 0])
    origins.append(new_origin)
    rotations.append(R_total)

fig, ax = plt.subplots(figsize=(7,7))

# Draw links
for i in range(3):
    ax.plot([origins[i][0], origins[i+1][0]], [origins[i][1], origins[i+1][1]], 'b-o', lw=3)

# Draw frames at base and each joint
frame_labels = ['Base', 'Joint1', 'Joint2', 'EE']
for i in range(4):
    draw_frame(ax, origins[i], rotations[i], frame_labels[i])

ax.set_aspect('equal')
ax.set_xlim(-1, 5)
ax.set_ylim(-2, 5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('DH Frames and Axes Assignment')
plt.grid(True)
plt.show()
