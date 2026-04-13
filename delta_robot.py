import numpy as np
from utils import unit_vectors, select_alpha_solution

class DeltaRobot:
    def __init__(self, La=0.2, Lb=0.4, rm=0.05, Rf=0.05, Rr=0.1118, L_EXT=0.5, thetas_deg=None):
        """
        Delta robot configuration parameters.
        - La: upper arm length
        - Lb: forearm length
        - rm: moving platform offset
        - Rf: base radius to Fi
        - Rr: offset Fi -> Ai along radial direction
        - L_EXT: extension bar length for limb index 1 (C_2)
        - thetas_deg: list of orientation angles for each limb
        """
        self.La = La
        self.Lb = Lb
        self.rm = rm
        self.Rf = Rf
        self.Rr = Rr
        self.L_EXT = L_EXT
        if thetas_deg is None:
            self.thetas_deg = [0.0, 120.0, 240.0]
        else:
            self.thetas_deg = thetas_deg

    def get_F_A_C(self, P, theta_deg):
        """
        Fi = [Rf cos(theta_i), Rf sin(theta_i), 0]
        Ai = Fi + [Rr cos(theta_i), Rr sin(theta_i), 0]
        Ci = P + [rm cos(theta_i), rm sin(theta_i), 0]
        """
        e_r, _, _ = unit_vectors(theta_deg)
        F = self.Rf * e_r
        A = F + self.Rr * e_r
        C = np.array(P) + self.rm * e_r
        return F, A, C

    def limb_has_solution(self, P, theta_deg, alpha_limits_deg=None):
        """
        Check if an IK solution exists for a single limb at position P.
        Returns: (success_bool, alpha_candidates_deg)
        """
        _, A, C = self.get_F_A_C(P, theta_deg)
        e_r, _, e_z = unit_vectors(theta_deg)

        d = C - A
        dr = np.dot(d, e_r)
        dz = np.dot(d, e_z)
        R = np.hypot(dr, dz)

        if R < 1e-12:
            return False, []

        K = (np.dot(d, d) + self.La**2 - self.Lb**2) / (2.0 * self.La)

        if abs(K) > R + 1e-12:
            return False, []

        phi = np.arctan2(dz, dr)
        gamma = np.arccos(np.clip(K / R, -1.0, 1.0))

        alpha_candidates = [phi + gamma, phi - gamma]
        alpha_candidates_deg = [np.rad2deg(a) for a in alpha_candidates]

        if alpha_limits_deg is None:
            return True, alpha_candidates_deg

        amin, amax = alpha_limits_deg
        valid = [a for a in alpha_candidates_deg if amin <= a <= amax]
        return len(valid) > 0, valid

    def get_alpha_vector(self, P, alpha_limits_deg=None, previous_alpha_deg=None):
        """
        Compute full alpha vector [alpha1, alpha2, alpha3] for position P.
        If previous_alpha_deg is provided, it helps in branch selection.
        """
        alphas = []
        for i, th in enumerate(self.thetas_deg):
            has_sol, alpha_sol = self.limb_has_solution(P, th, alpha_limits_deg=alpha_limits_deg)
            if not has_sol:
                return None
            
            prefer = previous_alpha_deg[i] if previous_alpha_deg is not None else None
            a = select_alpha_solution(alpha_sol, prefer_deg=prefer)
            if a is None:
                return None
            alphas.append(a)

        return np.array(alphas, dtype=float)

    def get_B(self, theta_deg, alpha_deg):
        """
        Given limb orientation theta and actuator angle alpha, find the elbow position B.
        """
        e_r, _, e_z = unit_vectors(theta_deg)
        A = (self.Rf + self.Rr) * e_r
        alpha = np.radians(alpha_deg)
        B = A + self.La * (np.cos(alpha) * e_r + np.sin(alpha) * e_z)
        return B

    def solve_fk(self, alphas_deg):
        """
        Solve for end-effector position P given actuator angles alphas_deg.
        """
        Qs = []
        for th, al in zip(self.thetas_deg, alphas_deg):
            e_r, _, _ = unit_vectors(th)
            B = self.get_B(th, al)
            Q = B - self.rm * e_r
            Qs.append(Q)
        
        Q1, Q2, Q3 = Qs
        A_mat = 2.0 * np.vstack([Q2 - Q1, Q3 - Q1])
        b_vec = np.array([
            np.dot(Q2, Q2) - np.dot(Q1, Q1),
            np.dot(Q3, Q3) - np.dot(Q1, Q1)
        ])
        
        v = np.cross(A_mat[0], A_mat[1])
        if np.linalg.norm(v) < 1e-9:
            return None
        v = v / np.linalg.norm(v)
        
        P0 = np.linalg.pinv(A_mat) @ b_vec
        w = P0 - Q1
        a = 1.0
        b = 2.0 * np.dot(v, w)
        c = np.dot(w, w) - self.Lb**2
        
        delta = b**2 - 4*a*c
        if delta < 0:
            return None
        
        t1 = (-b + np.sqrt(delta)) / (2*a)
        t2 = (-b - np.sqrt(delta)) / (2*a)
        
        P1 = P0 + t1 * v
        P2 = P0 + t2 * v
        
        # Pick lower solution (smaller z in this coordinate system)
        return P1 if P1[2] < P2[2] else P2

    def get_robot_state(self, alphas_deg):
        """
        Return the full state of the robot (P, E, and limb joint positions).
        """
        P = self.solve_fk(alphas_deg)
        if P is None:
            return None
        
        limb_data = []
        for i, (th, al) in enumerate(zip(self.thetas_deg, alphas_deg)):
            F, A, C_local = self.get_F_A_C(P, th)
            B = self.get_B(th, al)
            limb_data.append({
                'F': F, 'A': A, 'B': B, 'C': C_local
            })
        
        # Extension bar on limb 2 (index 1)
        C2 = limb_data[1]['C']
        B2 = limb_data[1]['B']
        forearm_dir = C2 - B2
        forearm_len = np.linalg.norm(forearm_dir)
        E = C2 + self.L_EXT * (forearm_dir / forearm_len) if forearm_len > 1e-12 else C2
        
        return {'P': P, 'E': E, 'limbs': limb_data}
