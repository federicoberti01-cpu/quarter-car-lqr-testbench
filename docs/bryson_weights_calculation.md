## 1. LQR Weighting Matrix Derivation (Bryson's Rule)

This document details the derivation of the $Q$ and $R$ weighting matrices used in `matlab/design_lqr.m` to compute the optimal gain matrix $K$.

\## 1. Methodology

Bryson's Rule normalizes state and control values by setting each diagonal weight inversely proportional to the square of the maximum allowable limit:

$$Q\_{ii} = \frac{1}{(x_{i,\text{max}})^2}, \quad R = \frac{1}{(u_{\text{max}})^2}$$

\---

## 2. Parameter Derivations \& Limits

| Variable | State / Input | Description | Constraint Source | Max Limit Value | Computed Weight |

| :--- | :--- | :--- | :--- | :--- | :--- |

| $x\_1$ | $z\_s - z\_u$ | Suspension Travel | Requirement R2 | $0.08 \text{ m}$ | $q\_1 = 1 / 0.03^2 = 1.11 \times 10^{3}$ |

| $x\_2$ | $\dot{z}_s$ | Body Velocity | Transient response target | $1.0 \text{ m/s}$ | $q\_2 = 1 / 1.0^2 = 1.0$ |

| $x\_3$ | $z\_u - z\_r$ | Tire Deflection | Requirement R3 (No wheel lift-off) | $z_{\text{static}} \approx 0.0176 \text{ m}$ | $q_3 = 1 / 0.0176^2 \approx 3225$ |

| $u$ | $F_a$ | Actuator Force | Requirement R5 (Saturation) | $2000 \text{ N}$ | $r = 1 / 2000^2 \approx 2.5 \times 10^{-7}$ |

\---

## 3. Weight fixing and tradeoffs

The procedure for fixing the weights starts from the values suggested from Bryson' rule. From there, manual tuning is needed to satisfy the requirements. The adjustment is done as follows:

- Starting from the given values, r is increased to check if the requirement $R4$ can be satisfied. If so, it is fixed at a value around 400-500N below the threshold

- From here, q3 is increased accordingly to match the R3 requirement. 

- q1 and q3 are constantly tuned to satisfy R1 and R3. r is either increased or decreased in order to allow for q1 and q3 variations.

- Finally, values are adjusted to match R2 and R4.



The final values are:

$q_1 = 90$

$q_2 = 1$

$q_3 = 5 \times 10^{6}$

$r = 1\times 10^{-6}$


