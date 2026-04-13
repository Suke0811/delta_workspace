import numpy as np

def numerical_jacobian(robot, P, h=1e-5, alpha_limits_deg=None, target='P'):
    """
    Compute J = d(target)/d(alpha), shape (3,3)
    using central finite differences.
    target can be 'P' (platform) or 'E' (end effector).
    Returns J such that target_dot = J * alpha_dot (with alpha in radians).
    """
    P = np.array(P, dtype=float)
    alpha0 = robot.get_alpha_vector(P, alpha_limits_deg=alpha_limits_deg)

    if alpha0 is None:
        return None, None

    J = np.zeros((3, 3), dtype=float)
    for j in range(3):
        # Perturb alpha_j
        alpha_plus = alpha0.copy()
        alpha_plus[j] += h
        
        alpha_minus = alpha0.copy()
        alpha_minus[j] -= h

        state_plus = robot.get_robot_state(alpha_plus)
        state_minus = robot.get_robot_state(alpha_minus)

        if state_plus is None or state_minus is None:
            return None, alpha0

        val_plus = state_plus[target]
        val_minus = state_minus[target]

        diff = val_plus - val_minus
        # d(target) / d(alpha_deg)
        J[:, j] = diff / (2.0 * h)

    # Convert to d(target) / d(alpha_rad)
    # d(target)/d(alpha_rad) = d(target)/d(alpha_deg) * (180/pi)
    J = J * (180.0 / np.pi)

    return J, alpha0

def inverse_condition_number(J):
    """
    Compute inverse condition number: k_inv = sigma_min / sigma_max
    """
    if J is None:
        return 0.0

    s = np.linalg.svd(J, compute_uv=False)
    smax = np.max(s)
    smin = np.min(s)

    if smax < 1e-12:
        return 0.0
    return float(smin / smax)

def jacobian_and_condition(robot, P, h=1e-5, alpha_limits_deg=None, target='P'):
    """
    Returns J, alpha0, and k_inv for the specified target ('P' or 'E').
    """
    J, alpha0 = numerical_jacobian(
        robot, P, h=h, alpha_limits_deg=alpha_limits_deg, target=target
    )
    if J is None:
        return None, alpha0, 0.0
    
    k_inv = inverse_condition_number(J)
    return J, alpha0, k_inv

def generate_workspace_fixed_thetas(robot, x_range, y_range, z_range, nx, ny, nz, alpha_limits_deg=None):
    """
    Scan a 3D grid and return points that have valid IK solutions.
    """
    xs = np.linspace(*x_range, nx)
    ys = np.linspace(*y_range, ny)
    zs = np.linspace(*z_range, nz)
    
    valid_points = []
    for x in xs:
        for y in ys:
            for z in zs:
                P = [x, y, z]
                alphas = robot.get_alpha_vector(P, alpha_limits_deg=alpha_limits_deg)
                if alphas is not None:
                    valid_points.append(P)
                    
    return np.array(valid_points)

def workspace_with_condition(robot, x_range, y_range, z_range, nx, ny, nz, alpha_limits_deg=None, target='P'):
    """
    Scan 3D grid and return valid points and their inverse condition numbers.
    Points are defined by P, but result points are either P or E based on target.
    """
    xs = np.linspace(*x_range, nx)
    ys = np.linspace(*y_range, ny)
    zs = np.linspace(*z_range, nz)
    
    points = []
    k_invs = []
    
    for x in xs:
        for y in ys:
            for z in zs:
                P = [x, y, z]
                # We need to check if P is reachable
                alphas = robot.get_alpha_vector(P, alpha_limits_deg=alpha_limits_deg)
                if alphas is None:
                    continue
                
                J, _, k_inv = jacobian_and_condition(robot, P, alpha_limits_deg=alpha_limits_deg, target=target)
                if J is not None:
                    if target == 'P':
                        points.append(P)
                    else:
                        state = robot.get_robot_state(alphas)
                        points.append(state['E'])
                    k_invs.append(k_inv)
                    
    return np.array(points), np.array(k_invs)

def generate_union_workspace(robot, x_range, y_range, z_range, nx, ny, nz, theta_vals_list, alpha_limits_deg=None, include_condition=False):
    """
    Computes the union of all feasible workspace points across multiple sampled arm reconfigurations.
    
    - robot: DeltaRobot instance
    - x_range, y_range, z_range: tuples of (min, max)
    - nx, ny, nz: grid resolutions
    - theta_vals_list: list of lists [theta1_vals, theta2_vals, theta3_vals]
    - include_condition: If True, also returns the best (maximum) inverse condition number for each point.
    """
    import itertools
    combinations = list(itertools.product(*theta_vals_list))
    
    union_points = {}  # Map tuple(round(p,4)) -> best_k_inv
    
    for th_vec in combinations:
        robot.thetas_deg = list(th_vec)
        if include_condition:
            pts, kvals = workspace_with_condition(robot, x_range, y_range, z_range, nx, ny, nz, alpha_limits_deg)
            for p, k in zip(pts, kvals):
                p_key = tuple(np.round(p, 4))
                if p_key not in union_points or k > union_points[p_key]:
                    union_points[p_key] = k
        else:
            pts = generate_workspace_fixed_thetas(robot, x_range, y_range, z_range, nx, ny, nz, alpha_limits_deg)
            for p in pts:
                p_key = tuple(np.round(p, 4))
                if p_key not in union_points:
                    union_points[p_key] = 1.0 # Placeholder
    
    points = np.array(list(union_points.keys()))
    if include_condition:
        kvals = np.array([union_points[tuple(p)] for p in points])
        return points, kvals
    
    return points
