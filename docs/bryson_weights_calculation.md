\# LQR Weighting Matrix Derivation (Bryson's Rule)



This document details the derivation of the $Q$ and $R$ weighting matrices used in `matlab/design\_lqr.m` to compute the optimal gain matrix $K$.



\## 1. Methodology

Bryson's Rule normalizes state and control values by setting each diagonal weight inversely proportional to the square of the maximum allowable limit:



$$Q\_{ii} = \\frac{1}{(x\_{i,\\text{max}})^2}, \\quad R = \\frac{1}{(u\_{\\text{max}})^2}$$



\---



\## 2. Parameter Derivations \& Limits



| Variable | State / Input | Description | Constraint Source | Max Limit Value | Computed Weight |

| :--- | :--- | :--- | :--- | :--- | :--- |

| $x\_1$ | $z\_s - z\_u$ | Suspension Travel | Requirement R2 | $0.08 \\text{ m}$ | $q\_1 = 1 / 0.08^2 = 156.25$ |

| $x\_2$ | $\\dot{z}\_s$ | Body Velocity | Transient response target | $1.0 \\text{ m/s}$ | $q\_2 = 1 / 1.0^2 = 1.0$ |

| $x\_3$ | $z\_u - z\_r$ | Tire Deflection | Requirement R3 (No wheel lift-off) | $z\_{\\text{static}} \\approx 0.0176 \\text{ m}$ | $q\_3 = 1 / 0.0176^2 \\approx 3225$ |

| $x\_4$ | $\\dot{z}\_u$ | Wheel Velocity | Transient response target | $5.0 \\text{ m/s}$ | $q\_4 = 1 / 5.0^2 = 0.04$ |

| $u$ | $F\_a$ | Actuator Force | Requirement R5 (Saturation) | $1500 \\text{ N}$ | $r = 1 / 1500^2 \\approx 4.44 \\times 10^{-7}$ |



\---



\## 3. MATLAB Implementation

In `design\_lqr.m`, the matrix weights are defined directly using these limits:



```matlab

Q = diag(\[156.25, 1.0, 3225, 0.04]);

R = 4.44e-7;

