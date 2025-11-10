import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ---------- DH Utilities ----------
def dh_transform(a, alpha, d, theta):
    """Return standard DH transformation matrix."""
    alpha_rad = np.deg2rad(alpha)
    theta_rad = np.deg2rad(theta)
    ca, sa = np.cos(alpha_rad), np.sin(alpha_rad)
    ct, st = np.cos(theta_rad), np.sin(theta_rad)
    
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])


# ---------- Forward Kinematics ----------
def forward_kinematics_3r(theta1, theta2, theta3, d1, a2, a3):
    """
    Forward kinematics for 3R manipulator.
    
    Parameters:
    -----------
    theta1, theta2, theta3 : float (degrees)
        Joint angles
    d1 : float
        Base height
    a2, a3 : float
        Link lengths for planar arm
    
    Returns:
    --------
    T : 4x4 numpy array
        End-effector transformation matrix
    """
    dh_params = [
        [0,   90,  d1,  theta1],  # Joint 1 (spatial rotation)
        [a2,  0,   0,   theta2],  # Joint 2 (planar)
        [a3,  0,   0,   theta3],   # Joint 3 (planar)
        [a3,  0,   0,   0]   
    ]
    
    T = np.eye(4)
    for a, alpha, d, theta in dh_params:
        T = T @ dh_transform(a, alpha, d, theta)
    
    return T


# ---------- Inverse Kinematics ----------
def inverse_kinematics_3r(px, py, pz, d1, a2, a3):
    """
    Analytical inverse kinematics for 3R manipulator.
    First joint: spatial rotation (base)
    Second and third joints: planar 2R arm
    
    Parameters:
    -----------
    px, py, pz : float
        Desired end-effector position
    d1 : float
        Base height offset
    a2, a3 : float
        Link lengths
    
    Returns:
    --------
    solutions : list of dicts
        All possible IK solutions with keys ['theta1', 'theta2', 'theta3']
        Returns empty list if no solution exists
    """
    
    solutions = []
    
    # Step 1: Solve for theta1 (base rotation about Z-axis)
    # The base joint determines the plane in which the 2R arm operates
    theta1 = np.arctan2(py, px)
    
    # Two possible base orientations
    theta1_candidates = [theta1, theta1 + np.pi]
    
    for theta1 in theta1_candidates:
        # Normalize to [-pi, pi]
        theta1 = np.arctan2(np.sin(theta1), np.cos(theta1))
        
        # Step 2: Project problem into 2D plane perpendicular to joint 1 axis
        # Calculate distance from Z-axis to target point
        r = np.sqrt(px**2 + py**2)
        
        # Height from joint 2 to target (subtract base height)
        h = pz - d1
        
        # Distance from joint 2 origin to target in the arm plane
        D = np.sqrt(r**2 + h**2)
        
        # Step 3: Check reachability
        if D > (a2 + a3) or D < abs(a2 - a3):
            continue
        
        # Step 4: Solve 2R planar inverse kinematics using law of cosines
        # Calculate theta3 (elbow angle)
        cos_theta3 = (D**2 - a2**2 - a3**2) / (2 * a2 * a3)
        
        # Check if solution exists
        if abs(cos_theta3) > 1:
            continue
        
        # Two elbow configurations: elbow up and elbow down
        theta3_candidates = [np.arccos(cos_theta3), -np.arccos(cos_theta3)]
        
        for theta3 in theta3_candidates:
            # Step 5: Solve for theta2 (shoulder angle)
            # Angle to target point in the arm plane
            alpha = np.arctan2(h, r)
            
            # Angle contributed by the elbow
            beta = np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))
            
            # theta2 is the difference
            theta2 = alpha - beta
            
            # Normalize angles to [-pi, pi]
            theta1_norm = np.arctan2(np.sin(theta1), np.cos(theta1))
            theta2_norm = np.arctan2(np.sin(theta2), np.cos(theta2))
            theta3_norm = np.arctan2(np.sin(theta3), np.cos(theta3))
            
            solution = {
                'theta1': np.rad2deg(theta1_norm),
                'theta2': np.rad2deg(theta2_norm),
                'theta3': np.rad2deg(theta3_norm)
            }
            
            solutions.append(solution)
    
    return solutions


def verify_solution(sol, target_pos, d1, a2, a3):
    """Verify IK solution by computing forward kinematics."""
    T = forward_kinematics_3r(
        sol['theta1'], sol['theta2'], sol['theta3'], d1, a2, a3
    )
    
    actual_pos = T[:3, 3]
    error = np.linalg.norm(actual_pos - target_pos)
    
    return T, actual_pos, error


# ---------- Interactive Visualization ----------
def visualize_3r_interactive(d1, a2, a3):
    """Create interactive 3D visualization with sliders."""
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(left=0.1, bottom=0.25)
    
    # Initial joint angles
    theta1_init, theta2_init, theta3_init = 0, 45, -45
    
    def calculate_points(theta1, theta2, theta3):
        """Calculate joint positions for visualization."""
        T = np.eye(4)
        points = [T[:3, 3]]
        
        dh_params = [
            [0,   90,  d1,  theta1],
            [a2,  0,   0,   theta2],
            [a3,  0,   0,   theta3]
        ]
        
        for a, alpha, d, theta in dh_params:
            T = T @ dh_transform(a, alpha, d, theta)
            points.append(T[:3, 3])
        
        return np.array(points)
    
    # Initial plot
    points = calculate_points(theta1_init, theta2_init, theta3_init)
    line, = ax.plot(points[:, 0], points[:, 1], points[:, 2], 
                    '-o', lw=3, c='b', markersize=8)
    
    # Labels
    texts = []
    for i, p in enumerate(points):
        text = ax.text(p[0], p[1], p[2], f'J{i}', fontsize=10, color='red')
        texts.append(text)
    
    # Axis limits
    max_range = a2 + a3 + d1
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(0, max_range*1.5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3R Manipulator (Spatial + Planar)')
    
    # Create sliders
    ax_theta1 = plt.axes([0.15, 0.15, 0.7, 0.03])
    ax_theta2 = plt.axes([0.15, 0.10, 0.7, 0.03])
    ax_theta3 = plt.axes([0.15, 0.05, 0.7, 0.03])
    
    slider_theta1 = Slider(ax_theta1, 'θ1 (base)', -180, 180, 
                          valinit=theta1_init, valstep=1)
    slider_theta2 = Slider(ax_theta2, 'θ2 (shoulder)', -180, 180, 
                          valinit=theta2_init, valstep=1)
    slider_theta3 = Slider(ax_theta3, 'θ3 (elbow)', -180, 180, 
                          valinit=theta3_init, valstep=1)
    
    def update(val):
        theta1 = slider_theta1.val
        theta2 = slider_theta2.val
        theta3 = slider_theta3.val
        
        points = calculate_points(theta1, theta2, theta3)
        line.set_data_3d(points[:, 0], points[:, 1], points[:, 2])
        
        for text, p in zip(texts, points):
            text.set_position((p[0], p[1]))
            text.set_3d_properties(p[2])
        
        fig.canvas.draw_idle()
    
    slider_theta1.on_changed(update)
    slider_theta2.on_changed(update)
    slider_theta3.on_changed(update)
    
    plt.show()


# ---------- Example Usage ----------
if __name__ == "__main__":
    # Robot parameters
    d1 = 10  # Base height
    a2 = 12.5   # Link 2 length
    a3 = 10   # Link 3 length
    
    print("=" * 60)
    print("3R MANIPULATOR - Analytical Inverse Kinematics")
    print("=" * 60)
    print(f"Robot parameters: d1={d1}, a2={a2}, a3={a3}\n")
    
    # Test inverse kinematics
    px, py, pz = 15,0,25
    
    print(f"Target position: ({px}, {py}, {pz})\n")
    
    solutions = inverse_kinematics_3r(px, py, pz, d1, a2, a3)
    
    if solutions:
        print(f"Found {len(solutions)} IK solution(s):\n")
        
        for i, sol in enumerate(solutions):
            print(f"Solution {i+1}:")
            print(f"  θ1 = {sol['theta1']:7.2f}° (base rotation)")
            print(f"  θ2 = {sol['theta2']:7.2f}° (shoulder)")
            print(f"  θ3 = {sol['theta3']:7.2f}° (elbow)")
            
            # Verify
            T, actual_pos, error = verify_solution(
                sol, np.array([px, py, pz]), d1, a2, a3
            )
            
            print(f"  Verification:")
            print(f"    Actual: ({actual_pos[0]:.2f}, {actual_pos[1]:.2f}, {actual_pos[2]:.2f})")
            print(f"    Error: {error:.6f} mm\n")
    else:
        print("No solution found - target out of reach!\n")
    
    # Launch interactive visualization
    print("Launching interactive visualization...")
    visualize_3r_interactive(d1, a2, a3)
