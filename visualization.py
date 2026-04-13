import numpy as np
import plotly.graph_objects as go

def plot_workspace(points, title="Workspace", marker_size=3, color=None):
    if len(points) == 0:
        print(f"No points to plot for {title}")
        return go.Figure()

    if color is None:
        color = 'blue'

    fig = go.Figure(data=[go.Scatter3d(
        x=points[:, 0],
        y=points[:, 1],
        z=points[:, 2],
        mode='markers',
        marker=dict(
            size=marker_size,
            color=color,
            opacity=0.6
        )
    )])

    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title='X [m]',
            yaxis_title='Y [m]',
            zaxis_title='Z [m]',
            aspectmode='data'
        )
    )
    return fig

def plot_union_workspace(points, kvals=None, title="Union Workspace (Asymmetric Reconfiguration)", color='green', marker_size=3):
    """
    Specifically for plotting the union workspace.
    If kvals is provided, plots condition number.
    """
    if kvals is not None:
        return plot_workspace_condition(points, kvals, title=title)
    return plot_workspace(points, title=title, color=color, marker_size=marker_size)

def plot_workspace_condition(points, kvals, title="Inverse Condition Number"):
    if len(points) == 0:
        print("No points for condition plot.")
        return go.Figure()

    fig = go.Figure(data=[go.Scatter3d(
        x=points[:, 0],
        y=points[:, 1],
        z=points[:, 2],
        mode='markers',
        marker=dict(
            size=3,
            color=kvals,
            colorscale='Viridis',
            colorbar=dict(title="k_inv"),
            opacity=0.8
        )
    )])

    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title='X [m]',
            yaxis_title='Y [m]',
            zaxis_title='Z [m]',
            aspectmode='data'
        )
    )
    return fig

def create_animation(robot, trajectory_alphas, filename=None):
    all_states = []
    for alphas in trajectory_alphas:
        state = robot.get_robot_state(alphas)
        if state:
            all_states.append(state)
            
    if not all_states:
        print("No valid states found for trajectory.")
        return None

    path_x = [s['P'][0] for s in all_states]
    path_y = [s['P'][1] for s in all_states]
    path_z = [s['P'][2] for s in all_states]

    path_e_x = [s['E'][0] for s in all_states]
    path_e_y = [s['E'][1] for s in all_states]
    path_e_z = [s['E'][2] for s in all_states]

    fig = go.Figure()
    state0 = all_states[0]
    
    # 0: End-effector path (P)
    fig.add_trace(go.Scatter3d(
        x=path_x, y=path_y, z=path_z,
        mode='lines', line=dict(color='blue', width=1, dash='dash'),
        name='P Path'
    ))

    # 1: New End-effector path (E)
    fig.add_trace(go.Scatter3d(
        x=path_e_x, y=path_e_y, z=path_e_z,
        mode='lines', line=dict(color='red', width=2),
        name='E Path'
    ))
    
    # 2,3,4: Limbs
    for i, limb in enumerate(state0['limbs']):
        pts = np.vstack([limb['F'], limb['A'], limb['B'], limb['C'], state0['P']])
        fig.add_trace(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='lines+markers',
            marker=dict(size=4),
            name=f'Limb {i+1}'
        ))

    # 5: Extension bar (Limb 2)
    limb2 = state0['limbs'][1]
    ext_pts = np.vstack([limb2['B'], limb2['C'], state0['E']])
    fig.add_trace(go.Scatter3d(
        x=ext_pts[:, 0], y=ext_pts[:, 1], z=ext_pts[:, 2],
        mode='lines+markers',
        marker=dict(size=5, color='orange'),
        line=dict(width=4),
        name='Ext Bar'
    ))

    # Frames for animation
    frames = []
    for state in all_states:
        frame_data = []
        # Trace 0,1 remain static or update if you want the "growing" path effect.
        # Let's keep them static for simplicity or update them to show moving point.
        frame_data.append(go.Scatter3d(x=path_x, y=path_y, z=path_z)) # 0
        frame_data.append(go.Scatter3d(x=path_e_x, y=path_e_y, z=path_e_z)) # 1
        
        # 2,3,4: Updated Limbs
        for i, limb in enumerate(state['limbs']):
            pts = np.vstack([limb['F'], limb['A'], limb['B'], limb['C'], state['P']])
            frame_data.append(go.Scatter3d(x=pts[:, 0], y=pts[:, 1], z=pts[:, 2]))
            
        # 5: Extension bar
        limb2 = state['limbs'][1]
        ext_pts = np.vstack([limb2['B'], limb2['C'], state['E']])
        frame_data.append(go.Scatter3d(x=ext_pts[:, 0], y=ext_pts[:, 1], z=ext_pts[:, 2]))
        
        frames.append(go.Frame(data=frame_data))

    fig.frames = frames

    fig.update_layout(
        scene=dict(
            xaxis=dict(range=[-0.7, 0.7]),
            yaxis=dict(range=[-0.7, 0.7]),
            zaxis=dict(range=[-1.3, 0.3]),
            aspectmode='manual',
            aspectratio=dict(x=1, y=1, z=1)
        ),
        updatemenus=[dict(
            type="buttons",
            buttons=[dict(label="Play", method="animate", args=[None, {"frame": {"duration": 50, "redraw": True}}])]
        )]
    )

    if filename:
        fig.write_html(filename)
        print(f"Animation saved to {filename}")
    
    return fig
