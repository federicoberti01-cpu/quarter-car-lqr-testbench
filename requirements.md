# Requirements Specification: Active Suspension Quarter-Car LQR Testbench

This document defines the quantitative performance and safety requirements for the active suspension control system using a quarter-car model. All requirements must be verified via automated simulation tests.

## Scenario Definition:

### 1. Ramp

- Linear increase in road height simulating a slope

- Length: 0.3m, height: 0.03m, start distance: 1m, car velocity: 10 m/s



### 2. Bump

- Raised cosine profile simulating a road bump

- Length: 3m, heigth: 0.05m, start distance: 1m, car velocity: 10 m/s



### 3. Rough Road

- Random mixture of sinusoidals simulating a rough road according to ISO 8096 standard (D class roughness, G0 = 512e-6)

- car velocity: 10 m/s, min/max cycles/m: 0.025 - 1, number of harmonics: 500

---

## 1. Ride Comfort (Body Acceleration)

* **ID:** R1
* **Class**: Performance
* **Description:** The vehicle body acceleration must remain strictly bounded to ensure passenger comfort.
* **Metric:** Root Mean Square of body acceleration over simulation window.
* **Requirement:** RMS $\leq70\%$ of passive suspension

---

## 2. Suspension Travel Constraints (Packaging)

* **ID:** R2
* **Class**: Safety
* **Description:** The relative displacement between the sprung mass (body) and unsprung mass (wheel) must not exceed physical mechanical limits.
* **Metric:** Maximum absolute suspension deflection.
* **Requirement:** Maximum suspension travel $\le 0.08 \, \text{m}$ (8cm) during scenarios.

---

## 3. Road Holding (Tire Contact)

* **ID:** R3
* **Class**: Safety
* **Description:** The tire must maintain continuous contact with the road surface. Complete loss of contact (wheel lift-off) is hazardous and unacceptable.
* **Metric:** Dynamic tire load/static tire load  $\vert{}F_{tire,dynamic}\vert{}/((m_s + m_u)\cdot g) < 1$
* **Requirement:** Ratio is $<1$ 

---

## 4. Transient Response & Settling Time

* **ID:** R4
* **Class**: Performance
* **Description:** After encountering a ramp input or discrete bump, the suspension system must stabilize quickly without excessive oscillation. Rough scenario is excluded from this metric
* **Metric:** Settling time ($t_s$) of the suspension deflection to within a $\pm 8\ \text{mm}$ band of steady state.
* **Requirement:** Settling time $t_s \le 1.5 \, \text{seconds}$.

---

## 5. Control Effort & Actuator Limits

* **ID:** R5
* **Class**: Safety
* **Description:** The active actuator force supplied by the controller must respect realistic physical saturation limits of actuators.
* **Metric:** Maximum control force ($\vert{}u_{max}\vert{}$).
* **Requirement:** Peak control force $\vert{}u\vert{} \le 1500 \, \text{N}$.



## Additional notes

Requirements have been divided into safety and performance conditions. Safety conditions must be verified at all times with suitable margins to ensure that the car and the passenger is not put at risk. Performance requirements are to be kept under a pre-defined threshold and lowered as possible.