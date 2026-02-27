#!/usr/bin/env python3
"""
Compensator Design for Grace Self-Balancing Robot
==================================================
Computes:
  1. LQR controller gain K  (full-state feedback)       u = -K x
  2. LQR-dual observer gain L  (state estimator)         via lqr(A^T, C^T, ...)

Both K and L are continuous-time. The controller node Euler-integrates the
observer at each timestep:  x̂ += dt * (A·x̂ + B·u + L·(y − C·x̂))

State:  x = [theta, theta_dot, p, p_dot]^T
Input:  u = tau_total [N·m]  (total torque from both motors)
Output: y = [theta, p]       (pitch from IMU, position from wheel encoders)

Observer via duality:
  Solve  lqr(A^T, C^T, Q_obs, R_obs)  →  K_obs
  Then   L = K_obs^T

Usage:
    python3 CompensatorDesign.py
"""

import numpy as np
import control as ct

# ═══════════════════════════════════════════════════════════════════════════════
# 1) Robot parameters (SI units)
# ═══════════════════════════════════════════════════════════════════════════════
r = 0.08255        # wheel radius [m]
b_tr = 0.45468     # track width [m]

Mb = 23.033614     # body mass excluding wheels [kg]
Mw = 3.0851        # wheel mass (one wheel) [kg]
L = 0.049851       # axle-to-body COM distance [m]

Jb = 0.16457       # body pitch inertia about COM [kg·m²]
Jw = 0.0090468     # wheel inertia about spin axis [kg·m²]

g = 9.81           # [m/s²]

Kt = 0.001162      # motor torque constant [N·m per firmware unit per motor]

# ═══════════════════════════════════════════════════════════════════════════════
# 2) Continuous-time state-space model
# ═══════════════════════════════════════════════════════════════════════════════
M_p = Mb + 2*Mw + 2*Jw / (r**2)
D = Jb + Mb * (L**2)
Delta = M_p * D - (Mb * L)**2

alpha1 = M_p * Mb * g * L / Delta
alpha2 = -(Mb**2) * g * L**2 / Delta
beta1  = -(Mb * L / r + M_p) / Delta
beta2  = (D + Mb * L * r) / (r * Delta)

A = np.array([
    [0.0,     1.0,  0.0,  0.0],
    [alpha1,  0.0,  0.0,  0.0],
    [0.0,     0.0,  0.0,  1.0],
    [alpha2,  0.0,  0.0,  0.0]
])

B = np.array([[0.0], [beta1], [0.0], [beta2]])

C = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0]
])

# ═══════════════════════════════════════════════════════════════════════════════
# 3) Controllability & Observability
# ═══════════════════════════════════════════════════════════════════════════════
assert np.linalg.matrix_rank(ct.ctrb(A, B)) == 4, "System not fully controllable!"
assert np.linalg.matrix_rank(ct.obsv(A, C)) == 4, "System not fully observable!"

# ═══════════════════════════════════════════════════════════════════════════════
# 4) LQR controller:  u = -K x
# ═══════════════════════════════════════════════════════════════════════════════
GAIN_SCALE = 100.0

# Reduce emphasis on theta_dot, increase emphasis on position (p) and p_dot
# to make the controller hold position while keeping overall control effort
# similar by keeping the same GAIN_SCALE.
Q_ctrl = np.diag([1000000.0, 10000000.0, 200.0, 2000.0]) / GAIN_SCALE
R_ctrl = np.array([[1.0]]) * GAIN_SCALE

K, S_ctrl, E_ctrl = ct.lqr(A, B, Q_ctrl, R_ctrl)

# ═══════════════════════════════════════════════════════════════════════════════
# 5) LQR-dual observer:  L = lqr(A^T, C^T, Q_obs, R_obs)^T
#
#    The observer error dynamics are:  e_dot = (A - LC) e
#    By duality, solving the LQR problem for the dual system (A^T, C^T)
#    gives an optimal L that minimises estimation error.
#
#    Q_obs penalises estimation error states  (process noise covariance analog)
#    R_obs penalises sensor trust             (measurement noise covariance analog)
#
#    Higher Q_obs/R_obs ratio → faster observer → more sensitive to noise
#
#    NOTE: This observer is run passively — the control law always uses the
#    directly measured states.  The observer runs alongside for monitoring,
#    logging, and state-estimation comparison only.
# ═══════════════════════════════════════════════════════════════════════════════
Q_obs = np.diag([10000.0, 50000.0, 10.0, 100.0])   # state estimation error penalty
R_obs = np.diag([1.0, 1.0])                          # sensor noise penalty (2 outputs)

K_obs, S_obs, E_obs = ct.lqr(A.T, C.T, Q_obs, R_obs)
L = K_obs.T  # Observer gain (4×2)

# ═══════════════════════════════════════════════════════════════════════════════
# 6) Print results
# ═══════════════════════════════════════════════════════════════════════════════
np.set_printoptions(precision=15, linewidth=120)

print("=" * 70)
print("  COMPENSATOR DESIGN RESULTS")
print("=" * 70)

K_flat = K.flatten()

print(f"\nOpen-loop poles:       {np.linalg.eigvals(A)}")
print(f"Closed-loop poles (K): {E_ctrl}")
print(f"Observer poles (L):    {E_obs}")

# Verify continuous observer stability: eigenvalues of (A - LC) must have Re < 0
eig_obs = np.linalg.eigvals(A - L @ C)
print(f"Observer eigs (A - LC): {eig_obs}")
print(f"All Re < 0: {all(np.real(e) < 0 for e in eig_obs)}")

print(f"\nK (N·m) = {K_flat}")
print(f"K_firmware = {K_flat / (2*Kt)}")
print(f"\nA:\n{A}")
print(f"\nB:\n{B}")
print(f"\nC:\n{C}")
print(f"\nL (4x2):\n{L}")

# ═══════════════════════════════════════════════════════════════════════════════
# 7) Copy-paste block for lqr_balancing_controller.py
# ═══════════════════════════════════════════════════════════════════════════════
print(f"\n{'=' * 70}")
print(f"  COPY-PASTE BLOCK")
print(f"{'=' * 70}")

print(f"""
K_THETA      = {K_flat[0]}
K_THETA_DOT  = {K_flat[1]}
K_P          = {K_flat[2]}
K_P_DOT      = {K_flat[3]}

A_OBS = np.{repr(A)}

B_OBS = np.{repr(B)}

C_OBS = np.array([[1.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0, 0.0]])

L_OBS = np.{repr(L)}
""")
