# Requirements Specification: Active Suspension Quarter-Car LQR Testbench

This document defines the quantitative performance requirements for the active suspension control system using a quarter-car model. All requirements must be verified via automated simulation tests.

## Scenario Definition: (to be written)

* Bump: height,length,speed
* Step: height
* Simulation Duration and i.c.
* Rough road: (ISO)

---

## 1. Ride Comfort (Body Acceleration)

* **ID:** R1
* **Description:** The vehicle body acceleration must remain strictly bounded to ensure passenger comfort when traversing a standardized bump disturbance.
* **Metric:** Root Mean Square of body acceleration (over what time window?)
* **Requirement:** RMS $\leq80\%$ of passive suspension (no control, fixed damper)

---

## 2. Suspension Travel Constraints (Packaging)

* **ID:** R2
* **Description:** The relative displacement between the sprung mass (body) and unsprung mass (wheel) must not exceed physical mechanical limits.
* **Metric:** Maximum absolute suspension deflection.
* **Requirement:** Maximum suspension travel $\le 0.10 \, \text{m}$ (10cm) during standard step and bump disturbances.

---

## 3. Road Holding (Tire Contact)

* **ID:** R3
* **Description:** The tire must maintain continuous contact with the road surface. Complete loss of contact (wheel lift-off) is hazardous and unacceptable.
* **Metric:** Dynamic tire load/static tire load  $\vert{}F_{tire,dynamic}\vert{}/((m_s + m_u)\cdot g) < 1
* **Requirement:** Ratio is $<1$ (never loses contact)

---

## 4. Transient Response & Settling Time

* **ID:** R4
* **Description:** After encountering a step input or discrete bump, the suspension system must stabilize quickly without excessive oscillation.
* **Metric:** Settling time ($t_s$) of the suspension deflection to within a $\pm 8\ \text{mm}$ band of steady state.
* **Requirement:** Settling time $t_s \le 2.5 \, \text{seconds}$.

---

## 5. Control Effort & Actuator Limits

* **ID:** R5
* **Description:** The active actuator force supplied by the controller must respect realistic physical saturation limits of actuators.
* **Metric:** Maximum control force ($\vert{}u_{max}\vert{}$).
* **Requirement:** Peak control force $\vert{}u\vert{} \le 1500 \, \text{N}$.