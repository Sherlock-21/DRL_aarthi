import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ---------- DH Utilities ----------
def dh_transform(a, alpha, d, theta):
    """Return standard DH transformation matrix."""
    alpha, theta = np.deg2rad(alpha), np.deg2rad(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

# ---------- Robot Parameters ----------
d1 = 10    # base height
L2 = 12.5   # first link length
L3 = 10    # second link length
L4 = 10    # tool length

# Initial joint angles
theta1_init = 90
theta2_init = 90
theta3_init = 0
theta4_init = 0

# ---------- Plot Setup ----------
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Adjust plot to make room for sliders
plt.subplots_adjust(left=0.1, bottom=0.3)

# Initial plot
def forward_kinematics(theta1, theta2, theta3, theta4):
    """Calculate forward kinematics with given joint angles."""
    dh_params = [
        # [0,    0,   d1,   theta1],      # Joint 1
        [90,   0,   d1,   theta1],      # Joint 2
        [0,    L2,  0,    theta2],      # Joint 3
        [90,   L2,  0,    theta3],       # Joint 4
        [0,   d1,   0,   theta4]  
    ]
    
    T = np.eye(4)
    points = [T[:3, 3]]  # store origin
    for alpha, a, d, theta in dh_params:
        T = T @ dh_transform(a, alpha, d, theta)
        points.append(T[:3, 3])
    
    return np.array(points)

# Initial robot configuration
points = forward_kinematics(theta1_init, theta2_init, theta3_init, theta4_init)
line, = ax.plot(points[:, 0], points[:, 1], points[:, 2], '-o', lw=3, c='b', markersize=8)

# Label frames
texts = []
for i, p in enumerate(points):
    text = ax.text(p[0], p[1], p[2], f'{i}', fontsize=10, color='red')
    texts.append(text)

# Set axis limits
max_range = 200
ax.set_xlim(-max_range, max_range)
ax.set_ylim(-max_range, max_range)
ax.set_zlim(0, max_range*1.5)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('4-DOF Arm Visualization (Interactive DH-based)')

# ---------- Create Sliders ----------
# Slider axes
ax_theta1 = plt.axes([0.15, 0.20, 0.7, 0.03])
ax_theta2 = plt.axes([0.15, 0.15, 0.7, 0.03])
ax_theta3 = plt.axes([0.15, 0.10, 0.7, 0.03])
ax_theta4 = plt.axes([0.15, 0.05, 0.7, 0.03])

# Create slider widgets
slider_theta1 = Slider(ax_theta1, 'θ1', -180, 180, valinit=theta1_init, valstep=1)
slider_theta2 = Slider(ax_theta2, 'θ2', -180, 180, valinit=theta2_init, valstep=1)
slider_theta3 = Slider(ax_theta3, 'θ3', -180, 180, valinit=theta3_init, valstep=1)
slider_theta4 = Slider(ax_theta4, 'θ4', -180, 180, valinit=theta4_init, valstep=1)

# ---------- Update Function ----------
def update(val):
    """Update robot configuration when slider changes."""
    theta1 = slider_theta1.val
    theta2 = slider_theta2.val
    theta3 = slider_theta3.val
    theta4 = slider_theta4.val
    
    # Recalculate forward kinematics
    points = forward_kinematics(theta1, theta2, theta3, theta4)
    
    # Update line
    line.set_data_3d(points[:, 0], points[:, 1], points[:, 2])
    
    # Update text labels
    for i, (text, p) in enumerate(zip(texts, points)):
        text.set_position((p[0], p[1]))
        text.set_3d_properties(p[2])
    
    fig.canvas.draw_idle()

# Register update function with sliders
slider_theta1.on_changed(update)
slider_theta2.on_changed(update)
slider_theta3.on_changed(update)
slider_theta4.on_changed(update)

plt.show()
