# quarter-car-lqr-testbench

# Active Suspension Quarter-Car LQR Testbench

[](https://github.com/<your-username>/quarter-car-lqr-testbench/actions)
[![Python Version](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)

## Project Overview

This repository implements and validates a **Linear Quadratic Regulator (LQR)** active suspension controller for a quarter-car vehicle model. The project translates classical automotive control design into a modern, automated Python CI/CD pipeline (`pytest` + GitHub Actions) to rigorously verify ride comfort, packaging constraints, and road holding against strict quantitative requirements.

---

## Quantitative Requirements & Verification

All designs are benchmarked against specs defined in [`requirements.md`](requirements.md):

* **R1 - Ride Comfort:** Body acceleration RMS $\leq 70\%$ of passive suspension.
* **R2 - Suspension Travel:** Max absolute deflection $\le 0.08 \, \text{m}$ (8 cm) during step/bump disturbances[cite: 2].
* **R3 - Road Holding:** Tire load ratio $\vert{}F_{\text{dynamic}}\vert{} / ((m_s + m_u)g) < 1$ (no wheel lift-off)[cite: 2].
* **R4 - Transient Response:** Settling time $t_s \le 1.5 \, \text{s}$ within an $8 \, \text{mm}$ band[cite: 2].
* **R5 - Control Effort:** Peak actuator force $\vert{}u\vert{} \le 1500 \, \text{N}$[cite: 2].

*Note: LQR weighting matrices ($Q$ and $R$) are systematically derived via **Bryson's Rule** (see [`docs/lqr_tuning_bryson.md`](docs/lqr_tuning_bryson.md)).*

---

## Road Scenarios

The test suite evaluates controllers across four standardized road profiles:

1. **Ramp:** Smooth linear incline ($3 \, \text{cm}$ over $0.3 \, \text{m}$ at $v = 5 \, \text{m/s}$).

2. **Bump:** Raised cosine profile ($5 \, \text{cm}$ height, $3.0 \, \text{m}$ length at $v = 5 \, \text{m/s}$).

3. **Rough Road:** ISO 8608 Class D stochastic profile generated via harmonic superposition ($N_{\text{freq}} = 500$, seed 42).

---

## 🗂️ Repository Structure

```text
quarter-car-lqr-testbench/
├── .github/
│   └── workflows/ci.yml       # GitHub Actions CI/CD automation
├── docs/
│   └── lqr_tuning_bryson.md   # Mathematical derivation of LQR weights
├── src/                       # Python core engineering library
│   ├── params.py              # Quarter-car physical parameters
│   ├── plant.py               # State-space representation (A, B, C, D)
│   ├── controller.py          # LQR gain calculation (Bryson's Rule)
│   └── scenarios.py           # Step, ramp, bump, and ISO rough road generators
├── tests/                     # Automated pytest validation suite
│   ├── test_plant.py          # Matrix & controllability checks
|   ├── sanity_checks.m        # Sample tests of functionality on Matlab
│   └── test_requirements.py   # R1–R5 requirement assertion tests
├── matlab/                    # Legacy MATLAB baseline prototype scripts
|   ├── suspension_params.m    # Parameters definition
|   ├── build_plant.m          # System construction
|   ├── lqr_design.m           # Design of LQR Controller with gains
|   ├── road_inputs.m          # Design of test scenarios
|   ├── compute_metrics.m      # Verification of requirements based on tests
|   └── run_prototype.m        # Validation and verification script
├── requirements.md            # Quantitative performance specifications
└── pyproject.toml             # Python package dependencies
```
