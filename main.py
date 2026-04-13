import numpy as np
from delta_robot import DeltaRobot
from kinematics_analysis import workspace_with_condition, generate_union_workspace
from visualization import plot_workspace, plot_workspace_condition, create_animation, plot_union_workspace

def main():
    # 1. Initialize Robot with parameters from the paper
    # La = 0.2, Lb = 0.4, rm = 0.05, Rf = 0.05, Rr = 0.1118, L_EXT = 0.5
    robot = DeltaRobot(
        La=0.25, Lb=0.4, rm=0.05, Rf=0.05, Rr=0.1, L_EXT=0.5, thetas_deg=[60.0, 195.0, 330.0]
    )
    
    # Workspace scan region
    X_RANGE = (-0.60, 0.60)
    Y_RANGE = (-0.60, 0.60)
    Z_RANGE = (-1.20, -0.10)   # expanded for L_EXT
    # Grid resolution (originally 45, 45, 45)
    NX, NY, NZ = 45, 45, 45
    
    # Optional actuator joint limits
    alpha_limits = None  # example: (-85.0, 85.0)

    print(f"Scanning workspace (Grid: {NX}x{NY}x{NZ})...")
    points, kvals = workspace_with_condition(
        robot, X_RANGE, Y_RANGE, Z_RANGE, NX, NY, NZ, alpha_limits_deg=alpha_limits
    )
    print(f"Workspace scan complete. Found {len(points)} valid points.")
    
    # 2. Visualization
    # Plot 1: Workspace valid points (binary)
    print("Generating workspace plot 'delta_workspace.html'...")
    fig_workspace = plot_workspace(points, title="Delta Robot Workspace (Valid Points)")
    fig_workspace.write_html("delta_workspace.html")

    # Plot 2: Workspace inverse condition number
    print("Generating workspace condition plot 'delta_condition.html'...")
    fig_condition = plot_workspace_condition(points, kvals, title="Delta Robot Workspace (Inverse Condition Number)")
    fig_condition.write_html("delta_condition.html")
    
    # 3. Animation Example
    # Define a simple circular trajectory in alpha-space for demonstration
    t = np.linspace(0, 2*np.pi, 60)
    trajectory_alphas = []
    for ti in t:
        a1 = 40 * np.sin(ti)
        a2 = 40 * np.sin(ti + 2*np.pi/3)
        a3 = 40 * np.sin(ti + 4*np.pi/3)
        trajectory_alphas.append([a1, a2, a3])
    
    print("Generating animation 'delta_animation.html'...")
    create_animation(robot, trajectory_alphas, filename="delta_animation.html")


if __name__ == "__main__":
    main()
