# Delta-Type Parallel Robot with Variable Arm Orientations

## Source paper

This summary is based on:

**Balmaceda-Santamaría, A. L., and Chávez-Toruño, A. E.**
*Asymmetric Reconfiguration of a 3-DOF Parallel Manipulator*.
Nexo Revista Científica, Vol. 33, No. 1, 2020.

Key points from the paper used here:

- The robot is a **3-DOF translational parallel manipulator** with a structure related to a Delta robot.
- Instead of fixing the three arm orientations at 120 degrees apart, the paper allows each arm orientation to be described by its own variable angle \(\theta_i\).
- The paper defines the geometry of the main limb points using these angles:
  - \(F_i = [R_f \cos\theta_i,\ 0,\ R_f \sin\theta_i]\)
  - \(A_i = [R_r \cos\theta_i + F_{x_i},\ 0,\ R_r \sin\theta_i + F_{z_i}]\)
  - \(C_i = [P_x + r_m \cos\theta_i,\ P_y,\ P_z + r_m \sin\theta_i]\)
- Workspace generation is done by **sweeping 3D Cartesian space** and discarding points that do not admit a real inverse-kinematics solution.
- The paper notes that when the three \(\theta_i\) are separated by 120 degrees, the standard symmetric Delta-like configuration is recovered.

## Parameters used in the code

The Python code used the same geometric parameters reported in the paper:

- `La = 0.2` m  
  Upper arm length

- `Lb = 0.4` m  
  Forearm length

- `rm = 0.05` m  
  Offset from end-effector center to each moving-platform attachment point

- `Rf = 0.05` m  
  Distance from base center to point \(F_i\)

- `Rr = 0.1118` m  
  Distance from \(F_i\) to \(A_i\)

- `L_EXT = 0.5` m
  Length of the extension bar attached to the platform at \(C_2\). This bar extends the second forearm linkage.

These values were taken from the workspace section of the paper.

## Angle ranges used

The paper gives example reconfiguration ranges:

- \(0^\circ \le \theta_1 \le 115^\circ\)
- \(120^\circ \le \theta_2 \le 235^\circ\)
- \(240^\circ \le \theta_3 \le 355^\circ\)

The code supports two modes:

1. **Fixed configuration workspace**
   - Example symmetric case: `(0, 120, 240)`
   - Example asymmetric case: `(20, 160, 300)`

2. **Union workspace over multiple sampled angle combinations**
   - The code samples values within the paper's angle ranges and unions all feasible workspace points.

## Workspace scan region used in the code

The code scanned a 3D box:

- `X_RANGE = (-0.40, 0.40)`
- `Y_RANGE = (-0.70, -0.10)`
- `Z_RANGE = (-0.80, 0.40)`

The `Y` upper limit of `-0.10` m was chosen to match the paper's note that the workspace was limited in the Y direction to avoid collisions.

## Grid resolution used

For the default script:

- Fixed-theta workspace:
  - `NX = 45`
  - `NY = 45`
  - `NZ = 45`

- Union workspace:
  - `nx = ny = nz = 35`

These are moderate values chosen to keep runtime reasonable while still showing the workspace shape clearly.

## What the code does

The script has four main parts.

### 1. Geometry definition

For each limb orientation angle \(\theta_i\), it computes:

- The base point \(F_i\)
- The shifted base joint point \(A_i\)
- The moving-platform attachment point \(C_i\)

using the equations from the paper.

### 2. Inverse-kinematics feasibility test

For a candidate end-effector position \(P = (x, y, z)\), the code checks whether each limb can reach its corresponding moving-platform point with the link lengths `La` and `Lb`.

This is done using a geometric existence test for the actuator angle \(\alpha_i\). A point is accepted only if all three limbs admit a real solution.

### 3. Workspace generation

The code sweeps a 3D grid of Cartesian points and keeps only the feasible ones.

There are two variants:

- `generate_workspace_fixed_thetas(...)`
  - Computes the workspace for one chosen tuple `(theta1, theta2, theta3)`

- `generate_union_workspace(...)`
  - Computes the union of feasible points over many angle combinations

### 4. Plotly visualization

The feasible points are displayed with `plotly.graph_objects.Scatter3d`.

The point cloud is colored by the `y` coordinate and shown as an interactive 3D scatter plot.

## Functions in the code

### `unit_vectors(theta_deg)`

Builds the local basis for each limb:

- radial direction in the x-z plane
- tangential direction in the x-z plane
- vertical y direction

### `get_F_A_C(P, theta_deg)`

Computes the three key geometric points for a limb:

- `F`
- `A`
- `C`

### `limb_has_solution(P, theta_deg, La, Lb, alpha_limits_deg=None)`

Checks whether one limb has a real inverse-kinematics solution.

This uses a compact geometric relation derived from the limb closure condition. It does not only test distance, but also computes candidate motor angles.

### `point_in_workspace(P, thetas_deg, La, Lb, alpha_limits_deg=None)`

Checks whether a Cartesian point is feasible for all three limbs.

### `generate_workspace_fixed_thetas(...)`

Builds the workspace for one fixed configuration.

### `generate_union_workspace(...)`

Builds the union workspace over many sampled angle combinations.

### `plot_workspace(points, title=...)`

Plots the workspace using Plotly.

## Assumptions and limitations

A few implementation choices were made in the code.

- The paper gives the parameterization and inverse-kinematics structure clearly, but not every step is presented in a copy-paste computational form.
- The code therefore uses a standard geometric feasibility test consistent with the paper's definitions of \(F_i\), \(A_i\), and \(C_i\).
- This is suitable for workspace generation and visualization.
- It is not yet a full branch-tracking inverse-kinematics solver with singularity analysis, collision geometry, or exact workspace volume computation as done in specialized CAD tools.

## Example configurations included in the code

### Symmetric Delta-like case

```python
symmetric_thetas = (0.0, 120.0, 240.0)
```

This approximates the standard Delta arrangement.

### Asymmetric case

```python
asymmetric_thetas = (20.0, 160.0, 300.0)
```

This shows how the workspace changes when the arm orientations are no longer equally spaced.

### Reconfigurable union case

```python
theta1_vals = np.linspace(0.0, 115.0, 5)
theta2_vals = np.linspace(120.0, 235.0, 5)
theta3_vals = np.linspace(240.0, 355.0, 5)
```

This samples the angle ranges given in the paper.

## Why this is useful

This code lets you study:

- how the workspace changes when the arms are not fixed at 120 degrees,
- how much reach can be gained through asymmetric reconfiguration,
- how to compare symmetric and asymmetric Delta-like layouts,
- how to build intuition before doing a more exact symbolic or CAD-based derivation.

## Possible next improvements

Natural extensions of the script would be:

- returning the actuator angles \(\alpha_i\) for each feasible point,
- separating elbow-up and elbow-down branches,
- computing workspace volume numerically,
- adding collision constraints between limbs,
- plotting symmetric and asymmetric workspaces together in one figure,
- adding Jacobian and condition-number evaluation as discussed in the paper.
