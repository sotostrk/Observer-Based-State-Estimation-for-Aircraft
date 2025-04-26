# Observer-Based-State-Estimation-for-Aircraft

##  Project Overview
This project models a simplified 4-state aircraft system and designs a state observer to estimate hidden states based on partial measurements. The observer was built and simulated in Python. Only the pitch angle (θ) and altitude (h) are directly measured; the pitch rate (q) and vertical speed (v) are estimated.

---

##  System Description
The modeled aircraft has the following states:

- θ (Pitch angle) [rad]
- q (Pitch rate) [rad/s]
- v (Vertical speed) [m/s]
- h (Altitude) [m]

The system dynamics are simplified under small angle assumptions:

- theta_dot = q 
- q_dot = -a q + b u 
- v_dot = g sin(theta) - D v 
- h_dot = v 

where:
- g = gravitational acceleration
- a = aerodynamic damping coefficient
- b = elevator control effectiveness
- D = vertical drag coefficient
- u = elevator input (control)

---

##  Observer Design
An observer (state estimator) was implemented to reconstruct the full state vector x_hat using:

x_dot_hat = A x_hat + B u + L (y - y_hat)

where:
- y = measured outputs ( theta, h )
- y_hat = C x_hat  = estimated outputs
- L = observer gain matrix

Observer gain L was manually tuned to achieve fast, stable estimation due to observability limitations.

---

##  Observability Limitations
- Observability matrix was computed.
- Was found to be: Rank 3 (out of 4 states) 
- Meaning: The system is only partially observable.
- Resulting in: Vertical speed (v) cannot be fully reconstructed based only on measurements of theta and h.
- Solution: Observer gains were manually tuned to stabilize estimation of observable states.

---

##  Simulation Results

### Open-Loop Aircraft Dynamics (No Estimation)
![Open-Loop Dynamics](https://raw.githubusercontent.com/sotostrk/Observer-Based-State-Estimation-for-Aircraft/main/Observer-Based%20State%20Estimation%20for%20Aircraft%20Pitch%20Dynamics%20in%20Python/fig/Fig_1.png)

### True vs Estimated States (Observer Performance)
![True vs Estimated States](https://raw.githubusercontent.com/sotostrk/Observer-Based-State-Estimation-for-Aircraft/main/Observer-Based%20State%20Estimation%20for%20Aircraft%20Pitch%20Dynamics%20in%20Python/fig/Fig_2.png)

### Estimation Errors Over Time
![Estimation Errors](https://raw.githubusercontent.com/sotostrk/Observer-Based-State-Estimation-for-Aircraft/main/Observer-Based%20State%20Estimation%20for%20Aircraft%20Pitch%20Dynamics%20in%20Python/fig/Fig_3.png)

##  How to Run

1. Clone or download the repository.
2. Ensure you have Python 3.8+ with the following packages:
   - numpy
   - scipy
   - matplotlib
3. Run `main_simulation.py`.
4. Figures will be generated showing observer performance.

---

##  Future Improvements
- Add direct pitch rate measurement (e.g., simulate gyroscope sensor)
- Implement reduced-order observer for observable modes only
- Expand to full 6-DOF aircraft model with more realistic aerodynamics


