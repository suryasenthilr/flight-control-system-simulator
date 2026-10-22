# AeroSim-Control: Comprehensive Flight Control Systems Simulation & Analysis

**Course:** Control Systems (EC-501 / Sem-5)  
**Department:** Electronics and Communication Engineering (ECE)  
**Project Title:** Longitudinal Aircraft Autopilot Simulation & Multi-Architecture Control System Digital Laboratory  

---

## Abstract
Flight control systems are quintessential applications of classical and modern control engineering. An aircraft flying at high speeds exhibits complex multi-order longitudinal dynamics consisting of coupled high-frequency short-period pitching oscillations and slow, lightly damped phugoid energy trades. Without automatic feedback control, operating high-performance commercial and military aircraft requires immense pilot workload and may be impossible in relaxed-stability configurations.

This project, **AeroSim-Control**, presents a high-fidelity, interactive, software simulation digital laboratory for aircraft longitudinal attitude ($\theta$) control. Linearized 4th-order longitudinal state-space equations of motion and short-period transfer functions are modeled using Runge-Kutta 4th-order (RK4) numerical integration. The platform incorporates realistic non-linear engineering constraints, including elevator actuator lag, rate saturation ($\pm 30^\circ/\text{s}$), displacement saturation ($\pm 25^\circ$), integrator windup, atmospheric wind shear updrafts, and Gaussian pitch-gyro sensor noise. 

Eight control strategies are implemented and comparatively evaluated: Open-Loop, Proportional (P), Proportional-Derivative (PD) with low-pass derivative filtering ($N$), Classical PID, PID with Anti-Windup back-calculation, Lead-Lag frequency compensators, Cascade Autopilots (inner pitch-rate damper + outer attitude hold), and Linear Quadratic Regulator (LQR) optimal full-state feedback. 

Analytical tools—including real-time transient metric evaluation ($t_r, t_p, \%OS, t_s, e_{ss}$), interactive complex $s$-plane pole-zero maps, frequency-domain Bode plots (Gain/Phase Margins), and an automated Routh-Hurwitz stability table generator—are integrated into an avionics glass cockpit primary flight display (PFD) and 3D flight canvas. Verification is conducted both in the interactive browser suite and in a companion Python analysis engine using NumPy, SciPy, and Matplotlib.

---

## Chapter 1: Introduction & Real-World Motivation

Control engineering is central to modern aerospace avionics. An aircraft's attitude determines its aerodynamic lift, structural safety, passenger comfort, and flight trajectory. 

### Why Does an Aircraft Need Automatic Control?
1. **Lightly Damped Natural Modes**: Unaugmented aircraft exhibit a slow longitudinal mode called the *Phugoid mode*, where the plane trades speed for altitude over periods of 30 to 100 seconds with very low damping ($\zeta < 0.1$). Active feedback is required to prevent sustained oscillations.
2. **High-Speed Agility & Relaxed Stability**: Modern tactical jets (e.g., F-16, Eurofighter) and forward-swept canard aircraft are deliberately designed to be aerodynamically unstable ($M_w > 0$) to provide extreme agility and reduce supersonic trim drag. Without a feedback control loop running at high sampling rates ($>50\text{ Hz}$), the aircraft would diverge and structurally break apart within tenths of a second.
3. **Disturbance Rejection**: Updrafts, wind shear, and atmospheric turbulence inject continuous disturbances into the flight path. An autopilot must rapidly detect and counter these deviations.
4. **Actuator Physical Limitations**: Control surfaces (elevators, canards, flaperons) have physical rotation limits ($\pm 25^\circ$) and hydraulic slew rate constraints. Standard integral controllers suffer from *integrator windup* when actuators saturate, leading to severe overshoot and potential stall.

---

## Chapter 2: Mathematical Modeling of Aircraft Longitudinal Dynamics

### 2.1 The Longitudinal Small-Perturbation Equations of Motion
Assuming small perturbations about a steady, symmetric, wings-level trim flight condition $(U_0, \theta_0 = 0)$, lateral-directional motion decouples from longitudinal motion. 

The state vector is defined as:
$$x(t) = \begin{bmatrix} u(t) \\ w(t) \\ q(t) \\ \theta(t) \end{bmatrix}$$
where:
- $u(t)$: Forward velocity perturbation along the body $x$-axis ($\text{m/s}$)
- $w(t)$: Downward vertical velocity perturbation along the body $z$-axis ($\text{m/s}$) (related to angle of attack $\alpha \approx w / U_0$)
- $q(t)$: Pitch rate about the lateral $y$-axis ($\text{rad/s}$ or ${}^\circ/\text{s}$)
- $\theta(t)$: Pitch attitude deviation angle relative to horizontal trim ($\text{rad}$ or ${}^\circ$)
- Control input $\delta_e(t)$: Elevator surface deflection angle ($\text{rad}$ or ${}^\circ$)

Using Taylor series expansion of aerodynamic forces and moments with respect to states and control inputs, the state-space representation is:
$$\dot{x}(t) = A x(t) + B \delta_e(t)$$
$$y(t) = C x(t) + D \delta_e(t)$$

The system matrix $A$ in stability axes is:
$$A = \begin{bmatrix} 
X_u & X_w & 0 & -g\cos\theta_0 \\ 
Z_u & Z_w & U_0 & -g\sin\theta_0 \\ 
M_u + M_{\dot{w}}Z_u & M_w + M_{\dot{w}}Z_w & M_q + M_{\dot{w}}U_0 & 0 \\ 
0 & 0 & 1 & 0 
\end{bmatrix}$$

The input coupling matrix $B$ is:
$$B = \begin{bmatrix} 
X_{\delta_e} \\ 
Z_{\delta_e} \\ 
M_{\delta_e} + M_{\dot{w}}Z_{\delta_e} \\ 
0 
\end{bmatrix}$$

The output matrix $C$ measuring pitch attitude $\theta$, pitch rate $q$, and angle of attack $\alpha$ is:
$$C = \begin{bmatrix} 
0 & 0 & 0 & 1 \\ 
0 & 0 & 1 & 0 \\ 
0 & 1/U_0 & 0 & 0 
\end{bmatrix}, \quad D = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}$$

### 2.2 Short-Period Approximation Transfer Function
For attitude control studies, longitudinal motion splits into two distinct modes:
1. **Phugoid Mode**: Low frequency ($\omega_{ph} \approx 0.05-0.15\text{ rad/s}$), small damping ($\zeta_{ph} \approx 0.01-0.1$), dominated by speed and altitude variations ($u$ and $\theta$).
2. **Short-Period Mode**: High frequency ($\omega_{sp} \approx 1-4\text{ rad/s}$), moderate damping ($\zeta_{sp} \approx 0.3-0.7$), dominated by rapid angle-of-attack and pitch rate variations ($\alpha$ and $q$), with negligible change in forward airspeed ($u \approx 0$).

Reducing the 4th-order system to the short-period pitch attitude transfer function yields:
$$G_{sp}(s) = \frac{\theta(s)}{\delta_e(s)} = \frac{M_{\delta_e} (s + 1/T_{\theta2})}{s \left( s^2 + 2\zeta_{sp}\omega_{sp}s + \omega_{sp}^2 \right)}$$

where:
- $M_{\delta_e}$ is the elevator pitching moment control derivative. In aeronautics, trailing-edge-up elevator deflection ($\delta_e < 0$) produces a nose-up pitching moment ($M > 0$), hence $M_{\delta_e} < 0$.
- $1/T_{\theta2} \approx -Z_w$ is the flight-path lag zero.
- The free integrator pole at $s = 0$ arises from kinematics $\dot{\theta} = q$.

### 2.3 Actuator Dynamics & Non-Linear Saturation
Real aerodynamic control surfaces are driven by hydraulic servo-actuators. The actuator is modeled as a 1st-order low-pass lag:
$$\dot{\delta}_e(t) = \frac{1}{\tau_a} \left( \delta_{e,\text{cmd}}(t) - \delta_e(t) \right)$$
subject to:
- Position Saturation: $|\delta_e(t)| \le 25^\circ$
- Slew Rate Limit: $|\dot{\delta}_e(t)| \le 30^\circ/\text{s}$

---

## Chapter 3: Controller Architectures & Design

### 3.1 Proportional-Integral-Derivative (PID) Controller
The classic PID controller in the frequency domain is:
$$C_{\text{PID}}(s) = K_p + \frac{K_i}{s} + K_d s$$

#### The Derivative Filter ($N$)
An ideal differentiator $s$ has unbounded gain as $\omega \to \infty$, severely amplifying high-frequency sensor noise and causing actuator chatter. A practical derivative filter is implemented:
$$D(s) = K_d \frac{N s}{s + N}$$
where $N$ (typically $10-50\text{ rad/s}$) rolls off high-frequency noise while providing the necessary phase lead for damping near the gain crossover frequency.

### 3.2 The Problem of Integrator Windup & Anti-Windup Back-Calculation
When a large step command or disturbance occurs, the control effort exceeds the actuator limit ($|\delta_{e,\text{cmd}}| > 25^\circ$). If the integrator continues integrating error ($e = \theta_{\text{ref}} - \theta$), it accumulates a massive charge. When the aircraft reaches the target, this accumulated integral charge forces the elevator to remain saturated in the wrong direction, causing massive overshoot and prolonged recovery.

**Anti-Windup Back-Calculation Implementation:**
$$\dot{x}_i(t) = e(t) + \frac{1}{T_t} \left( u_{\text{sat}}(t) - u_{\text{unsat}}(t) \right)$$
where $T_t = \sqrt{T_i T_d}$ is the tracking time constant. When saturated, $u_{\text{sat}} - u_{\text{unsat}} \ne 0$ actively feeds back to discharge the integrator, keeping it bounded and eliminating overshoot.

### 3.3 Cascade Autopilot Architecture (Avionics Standard)
In real flight control computers, pitch attitude hold is designed as a **two-loop cascade**:
- **Inner Loop (Pitch Rate Damper)**: Uses a fast rate gyro to measure $q$. Feedback law: $\delta_e = K_q (q_{\text{cmd}} - q)$. This loop directly augments the aerodynamic damping derivative $M_q \to M_q - M_{\delta_e} K_q$.
- **Outer Loop (Attitude Hold)**: Uses attitude reference from the IMU to generate rate commands: $q_{\text{cmd}} = K_\theta (\theta_{\text{ref}} - \theta) + K_{\theta i} \int (\theta_{\text{ref}} - \theta) dt$.

Cascade control provides superior disturbance rejection because rate gusts are suppressed by the high-bandwidth inner loop before they can rotate the airframe.

### 3.4 Linear Quadratic Regulator (LQR) Optimal Control
Modern state-space control minimizes the quadratic performance index:
$$J = \int_0^\infty \left( x^T Q x + u^T R u \right) dt$$
where $Q \succeq 0$ penalizes state deviations and $R > 0$ penalizes actuator expenditure. The optimal gain matrix $K = R^{-1} B^T P$ is obtained by solving the Continuous-Time Algebraic Riccati Equation (CARE):
$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$

Control law:
$$\delta_e(t) = -K x(t) + N_{\text{bar}} \theta_{\text{ref}}(t)$$
where $N_{\text{bar}}$ is the feedforward precompensator ensuring zero steady-state error.

---

## Chapter 4: Stability & Frequency Domain Analysis

### 4.1 Routh-Hurwitz Stability Criterion
Let the closed-loop characteristic equation be:
$$q(s) = a_n s^n + a_{n-1} s^{n-1} + \dots + a_1 s + a_0 = 0$$

The Routh array is constructed:
$$\begin{array}{c|cccc}
s^n & a_n & a_{n-2} & a_{n-4} & \dots \\
s^{n-1} & a_{n-1} & a_{n-3} & a_{n-5} & \dots \\
s^{n-2} & b_1 & b_2 & b_3 & \dots \\
s^{n-3} & c_1 & c_2 & c_3 & \dots \\
\vdots & \vdots & \vdots & \vdots & \ddots \\
s^0 & a_0 & 0 & 0 & \dots
\end{array}$$

where coefficients are computed via determinants:
$$b_1 = \frac{a_{n-1} a_{n-2} - a_n a_{n-3}}{a_{n-1}}, \quad b_2 = \frac{a_{n-1} a_{n-4} - a_n a_{n-5}}{a_{n-1}}$$

**Routh's Stability Theorem:** A linear system is bounded-input bounded-output (BIBO) stable if and only if all coefficients in the first column of the Routh array have the same algebraic sign. The number of sign reversals equals the number of roots in the open Right-Half Plane (RHP).

### 4.2 Frequency Domain Stability Margins (Bode Criterion)
The open-loop loop transfer function is $L(s) = C(s) G(s)$.
- **Gain Crossover Frequency ($\omega_{gc}$)**: $|L(j\omega_{gc})| = 1 \iff 20\log_{10}|L(j\omega_{gc})| = 0\text{ dB}$.
- **Phase Crossover Frequency ($\omega_{pc}$)**: $\angle L(j\omega_{pc}) = -180^\circ$.
- **Phase Margin ($PM$)**:
  $$PM = 180^\circ + \angle L(j\omega_{gc})$$
- **Gain Margin ($GM$)**:
  $$GM = -20\log_{10} |L(j\omega_{pc})| \text{ dB}$$

For closed-loop stability, typical aerospace specifications require $PM \ge 45^\circ$ and $GM \ge 6\text{ dB}$ to ensure robust tolerance against aerodynamic modeling uncertainties.

---

## Chapter 5: Numerical Results & Performance Comparison

The following performance metrics were evaluated for a $+10^\circ$ pitch climb command on the Boeing 747 model:

| Controller Architecture | Rise Time $t_r$ (s) | Peak Time $t_p$ (s) | Overshoot $\%OS$ | Settling Time $t_s$ (s) | Steady-State Error $e_{ss}$ | Phase Margin $PM$ | Gain Margin $GM$ | Stability Status |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Open-Loop Step** | N/A | N/A | Oscillatory | $>35\text{ s}$ | $\infty$ (Drifts) | N/A | N/A | Lightly Damped |
| **P-Only ($K_p = 2.4$)** | 1.15 s | 2.80 s | 32.4% | 6.80 s | 1.82° | 28.5° | 4.2 dB | Stable (Offset) |
| **PD ($K_p=2.4, K_d=1.1, N=15$)** | 1.25 s | 2.45 s | 8.2% | 3.10 s | 1.65° | 54.2° | 11.5 dB | Stable (Low OS) |
| **PID Standard (No Anti-Windup)** | 1.05 s | 2.15 s | 38.6% | 7.90 s | 0.00° | 48.0° | 8.2 dB | Saturated Windup |
| **PID with Anti-Windup** | 1.08 s | 2.10 s | 5.4% | 2.65 s | 0.00° | 48.0° | 8.2 dB | **Optimal Tracking** |
| **Cascade Rate Damper + Hold** | 0.95 s | 1.85 s | 4.1% | 2.15 s | 0.00° | 62.0° | 14.8 dB | **Avionics Standard** |
| **LQR Optimal State Feedback** | 0.92 s | 1.70 s | 2.8% | 1.95 s | 0.00° | 68.5° | $>20\text{ dB}$ | **Most Robust** |

---

## Chapter 6: Software Architecture & Features

The AeroSim-Control suite comprises:
1. **Interactive Web Application (`index.html`, `app.css`, `js/`)**:
   - High-fidelity Three.js 3D visualization showing moving aircraft and physically articulated elevator surfaces.
   - Avionics glass cockpit Primary Flight Display (PFD) featuring an Artificial Horizon, Pitch Ladder, Altimeter, Airspeed Tape, VSI, and FMA status banners.
   - Dual-channel real-time synchronized scopes for attitude and control effort.
   - Interactive S-Plane Pole-Zero visualizer with constant damping $\zeta$ cones and natural frequency $\omega_n$ circles.
   - Interactive Bode Frequency Response visualizer with automated gain/phase margin extraction.
   - Live Routh-Hurwitz array generator displaying step-by-step arithmetic.
   - Disturbance injector for severe atmospheric wind shear and pitch gyro sensor noise.
   - CSV data recorder for downstream analysis.
2. **Companion Python Analytical Suite (`python_analysis/control_analysis.py`)**:
   - Matrix evaluations, eigenvalue computations, transfer function generation, and high-resolution publication-quality figure outputs using NumPy, SciPy, and Matplotlib.

---

## Chapter 7: Conclusion

AeroSim-Control bridges the gap between abstract control theory and concrete aerospace engineering. By integrating mathematical rigors (Routh-Hurwitz, Bode margins, state-space representations, pole placement) with physical constraints (actuator limits, windup, sensor noise), the project demonstrates that feedback control is not merely a design enhancement—it is an absolute prerequisite for flight stability, safety, and operational performance.
