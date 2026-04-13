import streamlit as st
import numpy as np
from delta_robot import DeltaRobot
from kinematics_analysis import workspace_with_condition, jacobian_and_condition
import plotly.graph_objects as go

# Set page configuration
st.set_page_config(page_title="Delta Robot Interactive GUI", layout="wide")

st.title("Delta Robot Mechanism - Interactive Simulator")

# --- Initialize Trajectory History and Last State in Session State ---
if 'trajectory_p' not in st.session_state:
    st.session_state.trajectory_p = []
if 'trajectory_e' not in st.session_state:
    st.session_state.trajectory_e = []
if 'last_angles' not in st.session_state:
    st.session_state.last_angles = None
if 'last_geometry' not in st.session_state:
    st.session_state.last_geometry = None
if 'workspace_data' not in st.session_state:
    st.session_state.workspace_data = None
if 'workspace_target' not in st.session_state:
    st.session_state.workspace_target = 'E'

def reset_trajectory():
    st.session_state.trajectory_p = []
    st.session_state.trajectory_e = []

def reset_workspace():
    st.session_state.workspace_data = None

# --- Sidebar: Configuration Parameters ---
st.sidebar.header("1. Robot Geometry")
la = st.sidebar.slider("La (Upper Arm) [m]", 0.01, 1.0, 0.25, step=0.01)
lb = st.sidebar.slider("Lb (Forearm) [m]", 0.01, 1.0, 0.4, step=0.01)
rm = st.sidebar.slider("rm (Platform Radius) [m]", 0.0, 0.2, 0.05, step=0.005)
rf = st.sidebar.slider("Rf (Base Radius) [m]", 0.0, 0.2, 0.05, step=0.005)
rr = st.sidebar.slider("Rr (Base Offset) [m]", 0.0, 0.2, 0.1, step=0.005)
lext = st.sidebar.slider("L_EXT (Extension) [m]", 0.01, 1.0, 0.5, step=0.01)

st.sidebar.header("2. Limb Orientations (θ)")
th1 = st.sidebar.number_input("Theta 1 [deg]", value=60.0)
th2 = st.sidebar.number_input("Theta 2 [deg]", value=195.0)
th3 = st.sidebar.number_input("Theta 3 [deg]", value=330.0)

st.sidebar.header("3. Joint Angles (α)")
a1 = st.sidebar.slider("Alpha 1 [deg]", -90.0, 90.0, 0.0, step=0.5)
a2 = st.sidebar.slider("Alpha 2 [deg]", -90.0, 90.0, 0.0, step=0.5)
a3 = st.sidebar.slider("Alpha 3 [deg]", -90.0, 90.0, 0.0, step=0.5)

st.sidebar.header("4. Joint Limits (α_min, α_max)")
alpha_min = st.sidebar.number_input("Min Alpha [deg]", value=-90.0)
alpha_max = st.sidebar.number_input("Max Alpha [deg]", value=90.0)
alpha_limits = (alpha_min, alpha_max)

st.sidebar.header("5. Visualization Toggles")
show_e = st.sidebar.checkbox("Show End Effector (E)", value=True)
show_traj_p = st.sidebar.checkbox("Show P Trajectory", value=True)
show_traj_e = st.sidebar.checkbox("Show E Trajectory", value=True)

st.sidebar.header("6. Workspace Analysis")
nx = st.sidebar.number_input("Grid X Resolution", min_value=5, max_value=100, value=30)
ny = st.sidebar.number_input("Grid Y Resolution", min_value=5, max_value=100, value=30)
nz = st.sidebar.number_input("Grid Z Resolution", min_value=5, max_value=100, value=30)
ws_target = st.sidebar.radio("Workspace Reference Point", options=['P', 'E'], index=1)
include_cond_workspace = st.sidebar.checkbox("Include Condition Number", value=True)
highlight_singularities = st.sidebar.checkbox("Highlight Singularities", value=False)
singularity_threshold = st.sidebar.number_input("Singularity Threshold (k_inv <)", value=0.01, step=0.001, format="%.4f")

if st.sidebar.button("Calculate Workspace"):
    robot_for_ws = DeltaRobot(
        La=la, Lb=lb, rm=rm, Rf=rf, Rr=rr, L_EXT=lext,
        thetas_deg=[th1, th2, th3]
    )
    with st.spinner(f"Calculating workspace points for {ws_target}..."):
        # Define ranges for grid scanning (covering most expected delta workspace)
        xr = (-0.5, 0.5)
        yr = (-0.5, 0.5)
        zr = (-1.2, 0.2)
        
        pts, kvals = workspace_with_condition(
            robot_for_ws, xr, yr, zr, nx, ny, nz, target=ws_target,
            alpha_limits_deg=alpha_limits
        )
        st.session_state.workspace_data = (pts, kvals)
        st.session_state.workspace_target = ws_target

if st.sidebar.button("Clear Workspace"):
    reset_workspace()

if st.sidebar.button("Reset Trajectory"):
    reset_trajectory()

# --- Robot Logic ---
robot = DeltaRobot(
    La=la, Lb=lb, rm=rm, Rf=rf, Rr=rr, L_EXT=lext,
    thetas_deg=[th1, th2, th3]
)

current_geometry = (la, lb, rm, rf, rr, lext, th1, th2, th3, alpha_min, alpha_max)
if st.session_state.last_geometry != current_geometry:
    reset_trajectory()
    reset_workspace()
    st.session_state.last_geometry = current_geometry

current_angles = (a1, a2, a3)

# Check if current angles exceed limits
angles_in_limits = all(alpha_min <= a <= alpha_max for a in current_angles)
state = robot.get_robot_state([a1, a2, a3]) if angles_in_limits else None

# Calculate Current Condition Numbers
current_k_inv_p = None
current_k_inv_e = None
if state:
    _, _, current_k_inv_p = jacobian_and_condition(robot, state['P'], target='P', alpha_limits_deg=alpha_limits)
    _, _, current_k_inv_e = jacobian_and_condition(robot, state['P'], target='E', alpha_limits_deg=alpha_limits)

# --- Main Window: Display and Controls ---
if state:
    # Append to trajectory only if angles changed
    if st.session_state.last_angles != current_angles:
        st.session_state.trajectory_p.append(state['P'])
        st.session_state.trajectory_e.append(state['E'])
        st.session_state.last_angles = current_angles
    
    # Visualization using Plotly
    fig = go.Figure()
    
    # 1. Draw Limbs
    for i, limb in enumerate(state['limbs']):
        pts = np.vstack([limb['F'], limb['A'], limb['B'], limb['C'], state['P']])
        fig.add_trace(go.Scatter3d(
            x=pts[:,0], y=pts[:,1], z=pts[:,2],
            mode='lines+markers',
            marker=dict(size=4),
            line=dict(width=5),
            name=f'Limb {i+1}'
        ))

    # 2. Draw Extension Bar (on Limb 2)
    if show_e:
        limb2 = state['limbs'][1]
        ext_pts = np.vstack([limb2['B'], limb2['C'], state['E']])
        fig.add_trace(go.Scatter3d(
            x=ext_pts[:,0], y=ext_pts[:,1], z=ext_pts[:,2],
            mode='lines+markers',
            marker=dict(size=5, color='orange'),
            line=dict(width=6, color='orange'),
            name='Extension Bar'
        ))

    # 3. Draw Trajectories
    if show_traj_p and len(st.session_state.trajectory_p) > 1:
        traj_p = np.array(st.session_state.trajectory_p)
        fig.add_trace(go.Scatter3d(
            x=traj_p[:,0], y=traj_p[:,1], z=traj_p[:,2],
            mode='lines',
            line=dict(color='blue', width=2, dash='dash'),
            name='P Trajectory'
        ))

    if show_traj_e and show_e and len(st.session_state.trajectory_e) > 1:
        traj_e = np.array(st.session_state.trajectory_e)
        fig.add_trace(go.Scatter3d(
            x=traj_e[:,0], y=traj_e[:,1], z=traj_e[:,2],
            mode='lines',
            line=dict(color='red', width=3),
            name='E Trajectory'
        ))

    # 4. Draw Workspace
    if st.session_state.workspace_data is not None:
        pts, kvals = st.session_state.workspace_data
        if len(pts) > 0:
            if highlight_singularities:
                is_singularity = kvals < singularity_threshold
                non_singularity = ~is_singularity
                
                # Plot non-singularity points
                if np.any(non_singularity):
                    if include_cond_workspace:
                        fig.add_trace(go.Scatter3d(
                            x=pts[non_singularity,0], y=pts[non_singularity,1], z=pts[non_singularity,2],
                            mode='markers',
                            marker=dict(
                                size=2,
                                color=kvals[non_singularity],
                                colorscale='Viridis',
                                colorbar=dict(title="k_inv", x=1.1),
                                opacity=0.4
                            ),
                            name=f'Workspace ({st.session_state.workspace_target}) - Condition'
                        ))
                    else:
                        fig.add_trace(go.Scatter3d(
                            x=pts[non_singularity,0], y=pts[non_singularity,1], z=pts[non_singularity,2],
                            mode='markers',
                            marker=dict(size=2, color='gray', opacity=0.3),
                            name=f'Workspace ({st.session_state.workspace_target})'
                        ))
                
                # Plot singularity points in RED
                if np.any(is_singularity):
                    fig.add_trace(go.Scatter3d(
                        x=pts[is_singularity,0], y=pts[is_singularity,1], z=pts[is_singularity,2],
                        mode='markers',
                        marker=dict(size=4, color='red', opacity=0.8),
                        name=f'Singularity Point (k_inv < {singularity_threshold})'
                    ))
            else:
                if include_cond_workspace:
                    fig.add_trace(go.Scatter3d(
                        x=pts[:,0], y=pts[:,1], z=pts[:,2],
                        mode='markers',
                        marker=dict(
                            size=2,
                            color=kvals,
                            colorscale='Viridis',
                            colorbar=dict(title="k_inv", x=1.1),
                            opacity=0.4
                        ),
                        name=f'Workspace ({st.session_state.workspace_target}) - Condition'
                    ))
                else:
                    fig.add_trace(go.Scatter3d(
                        x=pts[:,0], y=pts[:,1], z=pts[:,2],
                        mode='markers',
                        marker=dict(size=2, color='gray', opacity=0.3),
                        name=f'Workspace ({st.session_state.workspace_target})'
                    ))

    # Update plot layout
    fig.update_layout(
        scene=dict(
            xaxis=dict(range=[-0.8, 0.8], title='X [m]'),
            yaxis=dict(range=[-0.8, 0.8], title='Y [m]'),
            zaxis=dict(range=[-1.4, 0.4], title='Z [m]'),
            aspectmode='manual',
            aspectratio=dict(x=1, y=1, z=1),
            uirevision='robot_plot_state'  # Directly relevant for 3D camera
        ),
        margin=dict(r=0, l=0, b=0, t=30),
        height=700,
        title="Interactive Delta Robot (FK Solution)",
        uirevision='robot_plot_state'  # Preserve camera viewpoint across updates
    )
    
    st.plotly_chart(fig, width='stretch', key='robot_plot')
    
    # Show Coordinates and Condition Numbers
    st.write("---")
    col1, col2 = st.columns(2)
    with col1:
        st.write(f"**Platform Position (P):**")
        st.code(f"[{state['P'][0]:.4f}, {state['P'][1]:.4f}, {state['P'][2]:.4f}]")
        st.write(f"**Inv. Condition (P):** {current_k_inv_p:.4f}" if current_k_inv_p is not None else "**Inv. Condition (P):** N/A")
    with col2:
        st.write(f"**End Effector (E):**")
        st.code(f"[{state['E'][0]:.4f}, {state['E'][1]:.4f}, {state['E'][2]:.4f}]")
        st.write(f"**Inv. Condition (E):** {current_k_inv_e:.4f}" if current_k_inv_e is not None else "**Inv. Condition (E):** N/A")

else:
    if not angles_in_limits:
        st.error(f"Invalid Configuration: Joint angles out of limits [{alpha_min}, {alpha_max}].")
    else:
        st.error("Invalid Configuration: No Kinematic Solution found for these joint angles.")
    if st.button("Clear Trajectory to try again"):
        reset_trajectory()
        st.rerun()

st.markdown("""
---
**Instructions:**
1. Use the sidebar to change geometric parameters and limb orientations.
2. Use the **Alpha** sliders to move the robot live.
3. The trajectories of `P` (platform) and `E` (extension point) are traced as you move.
4. Use **Reset Trajectory** to start over.
""")
