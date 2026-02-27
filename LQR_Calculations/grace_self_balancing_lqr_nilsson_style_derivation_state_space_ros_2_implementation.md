# GRACE Self-Balancing — Torque-Input LQR Derivation (Nilsson-style)

> **UPDATED**: Firmware switched from **speed control** to **torque control**.
> The control input is now motor torque (τ), not commanded velocity (v\_cmd).
> This changes the state-space model fundamentally — the first-order velocity lag
> is removed and replaced with full rigid-body dynamics including wheel inertia.

---

## Parameters (from URDF + 6.5″ wheels)

```python
# Geometry
r      = 0.08255        # wheel radius [m]
b_tr   = 0.45468        # track width (wheel center-to-center) [m]

# Masses
Mb     = 23.033614      # body mass excluding wheels [kg]
Mw     = 3.0851         # wheel mass (one wheel) [kg]

# Axle-to-body COM height
L      = 0.049851       # [m]

# Inertias
Jb     = 0.16457        # body pitch inertia about COM [kg*m^2]
Jw     = 0.0090468      # wheel inertia about spin axis (per wheel) [kg*m^2]

g      = 9.81           # gravity [m/s^2]

# Actuation
# Wheels are TORQUE-commanded via hoverboard FOC driver.
# Input: firmware torque units [-1000, 1000] (maps to FOC q-axis current ref)
# Physical torque: τ [N·m] = Kt * firmware_cmd
# Kt must be calibrated (see esp32_firmware_parameters.txt)
Kt     = 0.004          # [N·m per firmware unit per motor] — NEEDS CALIBRATION
```

---

## What changed from velocity control to torque control

| Aspect | Old (Speed Control) | New (Torque Control) |
|--------|-------------------|---------------------|
| ESP32 command format | `lXXX rXXX\n` (RPM) | `tLXXX tRXXX\n` (torque) |
| Control input u | v\_cmd [m/s] | τ [N·m] (total wheel torque) |
| Actuation model | First-order lag: v̇ = (v\_cmd − v)/τ\_lag | Direct torque: F=ma, τ=Iα |
| Velocity lag (τ\_lag) | ~25 ms (measured) | **Not applicable** (no velocity loop) |
| Wheel mass Mw | Not used (hidden by velocity loop) | **Required** (appears in dynamics) |
| Wheel inertia Jw | Not used (hidden by velocity loop) | **Required** (appears in dynamics) |
| State vector | [θ, θ̇, p, v] (v had its own dynamics) | [θ, θ̇, p, ṗ] (ṗ from rigid body EOM) |
| New feedback available | Speed only | + iq, id, DC current, motor angle |

Key consequence: The model with torque input is **more physically complete** (includes
wheel inertia and mass) and **more responsive** (no velocity loop lag), but also
**less inherently damped** (the velocity loop provided natural damping).

---

## Modeling setup (Nilsson-style)

We model the robot as an inverted pendulum (body) on a two-wheel base.

Variables (planar x–z):

- p: forward displacement of wheel axle (x direction)
- θ: body pitch angle (θ = 0 upright, positive = leaning forward)

Encoder kinematics:

- p = r \* (φ\_L + φ\_R)/2
- ṗ = r \* (φ̇\_L + φ̇\_R)/2

IMU (Madgwick) gives:

- θ from orientation quaternion
- θ̇ from gyro (pitch axis)

Input:

- τ = total forward torque from both motors [N·m]
- (For both wheels commanding the same torque: τ = 2 × τ\_one\_motor)

---

## Full derivation (Lagrangian mechanics → torque input)

### Body COM position and velocity

- x\_c = p + L sin(θ)
- z\_c = L cos(θ)
- ẋ\_c = ṗ + Lθ̇ cos(θ)
- ż\_c = −Lθ̇ sin(θ)

### Kinetic energy

Body:

- T\_body = ½ Mb (ẋ\_c² + ż\_c²) + ½ Jb θ̇²
- T\_body = ½ Mb (ṗ² + 2Lṗθ̇ cos θ + L²θ̇²) + ½ Jb θ̇²

Wheels (both, rolling without slip, φ̇ = ṗ/r):

- T\_wheels = ½ (2Mw) ṗ² + ½ (2Jw) (ṗ/r)²

Total:

- T = ½ Mb ṗ² + Mb L ṗ θ̇ cos θ + ½ (Mb L² + Jb) θ̇² + (Mw + Jw/r²) ṗ²

### Potential energy

- V = Mb g L cos(θ)

### Euler–Lagrange equations

Generalized coordinates: q = [p, θ]

Generalized forces from motor torque τ (applied between body and wheels):

- Q\_p = τ / r    (tangential force at wheel contact → horizontal acceleration)
- Q\_θ = −τ       (reaction torque on body — Newton's third law)

The sign convention: positive τ spins wheels forward, reaction tilts body backward.

### Linearization about θ ≈ 0

sin θ ≈ θ,  cos θ ≈ 1,  θ̇² ≈ 0

**Equation for p (horizontal translation):**

    M\_p · p̈ + Mb·L · θ̈ = τ/r                    ... (1)

where  M\_p = Mb + 2Mw + 2Jw/r²   (effective translational mass)

**Equation for θ (pitch rotation):**

    Mb·L · p̈ + D · θ̈ = Mb·g·L · θ − τ           ... (2)

where  D = Jb + Mb·L²   (pitch inertia about axle)

### Solving for p̈ and θ̈

Write in matrix form:

    ┌ M\_p    Mb·L ┐ ┌ p̈ ┐   ┌  0       0    ┐ ┌ p ┐   ┌  1/r ┐
    │              │ │    │ = │                 │ │   │ + │      │ τ
    └ Mb·L   D    ┘ └ θ̈ ┘   └  0    Mb·g·L  ┘ └ θ ┘   └  -1  ┘

Mass matrix determinant:

    Δ = M\_p · D − (Mb·L)²

Inverse mass matrix:

    M⁻¹ = (1/Δ) ┌  D      -Mb·L ┐
                  └ -Mb·L   M\_p  ┘

Solving:

**θ̈ = (M\_p · Mb·g·L / Δ) · θ  −  ((Mb·L/r + M\_p) / Δ) · τ**

**p̈ = −(Mb²·g·L² / Δ) · θ  +  ((D + Mb·L·r) / (r·Δ)) · τ**

#### Physical interpretation of signs

1. A₂₁ = M\_p·Mb·g·L/Δ > 0: Lean forward (θ>0) → pitch accelerates further (unstable)
2. B₂ = −(Mb·L/r + M\_p)/Δ < 0: Forward torque (τ>0) → body tilts backward (reaction)
3. A₄₁ = −Mb²·g·L²/Δ < 0: Lean forward → axle pushed backward (body falling pulls base)
4. B₄ = (D + Mb·L·r)/(r·Δ) > 0: Forward torque → axle accelerates forward ✓

---

## State-space model (linearized, torque input)

### State and input

State vector:

    x = [θ, θ̇, p, ṗ]ᵀ

- θ:  body pitch angle [rad]
- θ̇:  body pitch rate [rad/s]
- p:  forward position of wheel axle [m]
- ṗ:  forward velocity of wheel axle [m/s]

Input:

    u = τ   [N·m]  (total forward torque from both motors)

### Intermediate constants

    M\_p = Mb + 2·Mw + 2·Jw/r²     [kg]        effective translational mass
    D   = Jb + Mb·L²                [kg·m²]     pitch inertia about axle
    Δ   = M\_p·D − (Mb·L)²          [kg²·m²]    mass matrix determinant

With the robot parameters:

    M\_p = 23.033614 + 2(3.0851) + 2(0.0090468)/0.08255²
        = 23.033614 + 6.1702 + 2.6553
        = 31.8592  [kg]

    D   = 0.16457 + 23.033614 × 0.049851²
        = 0.16457 + 0.05724
        = 0.22181  [kg·m²]

    Δ   = 31.8592 × 0.22181 − (23.033614 × 0.049851)²
        = 7.0691 − 1.3189
        = 5.7503  [kg²·m²]

### A and B matrices

    ẋ = A·x + B·u

```
A = ┌ 0     1     0     0   ┐
    │ α₁    0     0     0   │
    │ 0     0     0     1   │
    └ α₂    0     0     0   ┘

B = ┌   0  ┐
    │  β₁  │
    │   0  │
    └  β₂  ┘
```

Where:

    α₁ = M\_p · Mb · g · L / Δ        (pitch-gravity coupling)
    α₂ = −Mb² · g · L² / Δ            (position-gravity coupling)
    β₁ = −(Mb·L/r + M\_p) / Δ          (torque → pitch acceleration)
    β₂ = (D + Mb·L·r) / (r · Δ)       (torque → position acceleration)

Numerically:

    α₁ =  31.8592 × 23.033614 × 9.81 × 0.049851 / 5.7503  =  62.47   [1/s²]
    α₂ = −(23.033614)² × 9.81 × 0.049851² / 5.7503         = −2.25    [1/s²]
    β₁ = −(23.033614 × 0.049851 / 0.08255 + 31.8592) / 5.7503 = −7.96 [1/(kg·m)]
    β₂ =  (0.22181 + 23.033614 × 0.049851 × 0.08255) / (0.08255 × 5.7503) = 0.67 [1/(kg·m)]

So numerically:

```
A = ┌  0       1       0       0    ┐
    │  62.47   0       0       0    │
    │  0       0       0       1    │
    └  -2.25   0       0       0    ┘

B = ┌   0     ┐
    │  -7.96  │
    │   0     │
    └   0.67  ┘
```

### Comparison with old velocity-command model

Old (speed control, with velocity lag τ\_lag):

```
A_old = ┌ 0     1     0         0       ┐     B_old = ┌     0      ┐
        │ a     0     0     b/τ_lag     │             │ -b/τ_lag   │
        │ 0     0     0         1       │             │     0      │
        └ 0     0     0    -1/τ_lag     ┘             └  1/τ_lag   ┘
```

Key differences:
1. **No velocity lag** in A\[3,3\] — torque acts instantaneously (no −1/τ term)
2. **Position-pitch coupling** in A\[3,0\] — gravity affects p̈ through body coupling
3. **B matrix** has torque coefficients instead of velocity-lag coefficients
4. **All four physical parameters** (Mb, Mw, L, Jb, Jw, r) appear — more accurate model

---

## Converting LQR output to firmware units

The LQR computes:

    u = τ\_desired = −K · x     [N·m]

To send to ESP32:

    firmware\_cmd\_per\_wheel = τ\_desired / (2 · Kt)

where Kt \[N·m per firmware unit per motor\] is calibrated experimentally
(see esp32\_firmware\_parameters.txt).

The factor of 2 is because τ is the TOTAL torque from both wheels, and each
wheel gets half.

For Gazebo simulation, u is in N·m and can be applied directly as joint effort.

---

## LQR tuning notes (torque control)

- **R penalty**: Should be higher than velocity model because torque can cause
  mechanical stress and current spikes. Start with R = diag(\[10\]) or higher.
- **Q\[0,0\] (theta)**: Most important — keeps robot upright. Start ~1000.
- **Q\[1,1\] (theta\_dot)**: Damps oscillation. Start ~10.
- **Q\[2,2\] (p)**: Position regulation. Start ~1 (low priority for balance-only).
- **Q\[3,3\] (p\_dot)**: Velocity regulation. Start ~10.
- **No tau parameter** to tune — one less thing to measure!
- The system is **less damped** than the velocity-loop version — expect the LQR
  to naturally provide more aggressive corrections.

