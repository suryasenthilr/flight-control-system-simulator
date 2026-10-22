# AeroSim-Control: Comprehensive Viva Voce & Oral Exam Preparation Guide

This guide contains the most critical oral exam and viva voce questions asked by university professors in undergraduate Control Systems courses (ECE Semester 5), along with exact mathematical formulas and textbook answers.

---

## Part 1: Aircraft Dynamics & Physical Modeling

### Q1: Why did you choose an airplane for your control systems project?
**Answer:** An airplane is the classic benchmark in control engineering because it has multi-order coupled dynamics that are inherently lightly damped or even aerodynamically unstable. Flight control demonstrates the real necessity of feedback: without control augmentation, an aircraft suffers from Phugoid oscillations, diverges under wind shear, and high-performance jets cannot remain airborne.

### Q2: What are the two longitudinal modes of an aircraft?
**Answer:**
1. **Short-Period Mode**: A fast, well-damped pitch oscillation ($\omega_{sp} \approx 1-3\text{ rad/s}, \zeta_{sp} \approx 0.3-0.7$) involving rapid changes in angle of attack $\alpha$ and pitch rate $q$, while forward speed $u$ remains almost constant.
2. **Phugoid (Long-Period) Mode**: A slow, very lightly damped oscillation ($\omega_{ph} \approx 0.05-0.1\text{ rad/s}, \zeta_{ph} \approx 0.01-0.1$) where the aircraft trades kinetic energy (airspeed) for potential energy (altitude) with nearly constant angle of attack.

### Q3: Why is there an integrator pole at $s = 0$ in the pitch transfer function $\theta(s)/\delta_e(s)$?
**Answer:** Because pitch attitude angle $\theta(t)$ is the mathematical integral of pitch rate $q(t)$:
$$\dot{\theta}(t) = q(t) \implies \theta(s) = \frac{1}{s} q(s)$$
This kinematic relationship introduces a pole at the origin $s = 0$, making the open-loop transfer function a Type 1 system for pitch attitude.

### Q4: Why does a trailing-edge-up elevator deflection correspond to positive pitch up in aerodynamics?
**Answer:** When the elevator surface deflects trailing-edge up ($\delta_e < 0$), it decreases the camber of the horizontal tailplane, creating a downward aerodynamic lift force on the tail. Because the tail is behind the aircraft's Center of Gravity (C.G.), this downward force generates a positive (nose-up) pitching moment:
$$M = -l_t \cdot \Delta L_{\text{tail}} > 0$$
Thus, the pitching moment derivative with respect to elevator deflection $M_{\delta_e}$ is negative.

---

## Part 2: Time-Domain Performance Specifications

### Q5: Define the 5 key time-domain specifications for a second-order step response.
**Answer:**
1. **Rise Time ($t_r$)**: Time required for the response to rise from 10% to 90% (or 0% to 100%) of the final steady-state value:
   $$t_r \approx \frac{1.8}{\omega_n} \quad \text{or} \quad t_r = \frac{\pi - \beta}{\omega_d}, \quad \beta = \tan^{-1}\left(\frac{\sqrt{1-\zeta^2}}{\zeta}\right)$$
2. **Peak Time ($t_p$)**: Time required to reach the first peak of the overshoot:
   $$t_p = \frac{\pi}{\omega_d} = \frac{\pi}{\omega_n \sqrt{1 - \zeta^2}}$$
3. **Maximum Percentage Overshoot ($\%OS$ or $M_p$)**:
   $$\%OS = e^{-\frac{\zeta \pi}{\sqrt{1 - \zeta^2}}} \times 100\%$$
4. **Settling Time ($t_s$)**: Time required for the response curve to reach and remain within a specified tolerance band:
   - 2% Criterion: $t_s \approx \frac{4}{\zeta \omega_n}$
   - 5% Criterion: $t_s \approx \frac{3}{\zeta \omega_n}$
5. **Steady-State Error ($e_{ss}$)**: Difference between the commanded reference and actual output as $t \to \infty$:
   $$e_{ss} = \lim_{t \to \infty} e(t) = \lim_{s \to 0} s E(s) = \lim_{s \to 0} \frac{s R(s)}{1 + G(s)H(s)}$$

### Q6: If a teacher asks: "How can I reduce overshoot without making the response slower?"
**Answer:** Increase the damping ratio $\zeta$ using derivative action ($K_d$) or a lead compensator. Derivative control introduces an open-loop zero in the Left-Half Plane, which pulls the root locus branches further into the stable left-half plane, adding damping without sacrificing system bandwidth.

---

## Part 3: Stability & The Routh-Hurwitz Criterion

### Q7: State the Routh-Hurwitz Stability Criterion.
**Answer:** For a linear time-invariant system to be bounded-input bounded-output (BIBO) stable:
1. All coefficients of the characteristic polynomial $q(s) = a_n s^n + a_{n-1} s^{n-1} + \dots + a_0 = 0$ must be strictly positive (necessary condition).
2. All elements in the first column of the Routh array must have the same algebraic sign (sufficient condition).
3. The number of sign changes in the first column equals the number of roots located in the open Right-Half Plane (RHP).

### Q8: What happens if a zero appears in the first column of the Routh array?
**Answer:** There are two distinct cases:
1. **Single zero in column 1, but remaining row elements are non-zero**: Replace the zero with a small positive number $\epsilon > 0$, complete the array calculations, and take the limit as $\epsilon \to 0^+$.
2. **An entire row of zeros**: This indicates symmetric pairs of roots (pairs on the imaginary axis $\pm j\omega$, or real pairs $\pm \sigma$, or complex quadruplets). We form an auxiliary polynomial $A(s)$ from the coefficients of the preceding row, differentiate it $\frac{dA(s)}{ds}$, and replace the zero row with the derivative coefficients.

### Q9: How does AeroSim-Control compute the Routh table dynamically?
**Answer:** AeroSim-Control extracts the characteristic polynomial coefficients of the closed-loop system $1 + C(s)G(s) = 0$ in real time whenever the user adjusts slider gains ($K_p, K_i, K_d$). It populates the Routh array, checks for pivot zeros (replacing them with $\epsilon = 10^{-6}$), counts sign changes, and renders the exact formatted arithmetic in the UI.

---

## Part 4: Root Locus & The Complex S-Plane

### Q10: What is the Root Locus?
**Answer:** The Root Locus is the trajectory traced by the closed-loop poles in the complex $s$-plane as a system parameter (typically open-loop gain $K$) varies from $0$ to $+\infty$. It is governed by two fundamental conditions derived from $1 + K G(s) H(s) = 0$:
1. **Magnitude Condition**: $|K G(s) H(s)| = 1$
2. **Angle Condition**: $\angle [G(s) H(s)] = \pm (2k + 1) 180^\circ$ ($k = 0, 1, 2, \dots$)

### Q11: What do the radial lines and concentric circles represent on the S-plane map?
**Answer:**
- **Radial Lines from Origin**: Represent lines of constant damping ratio $\zeta = \cos\phi$, where $\phi$ is the angle between the negative real axis and the radial line. Poles lying along the same radial line have the same percentage overshoot.
- **Concentric Circles Centered at Origin**: Represent curves of constant undamped natural frequency $\omega_n$, since $|s| = \sqrt{\sigma^2 + \omega_d^2} = \omega_n$.

### Q12: Why is the imaginary axis ($j\omega$) called the stability boundary?
**Answer:** A pole with $\text{Re}(s) = \sigma$ corresponds to a time response factor $e^{\sigma t}$:
- If $\sigma < 0$ (Left-Half Plane): $e^{\sigma t} \to 0$ as $t \to \infty$ (Asymptotically Stable).
- If $\sigma > 0$ (Right-Half Plane): $e^{\sigma t} \to \infty$ as $t \to \infty$ (Unstable / Exponential Blowup).
- If $\sigma = 0$ (On the $j\omega$ axis): $e^{0 \cdot t} = 1$ (Marginally Stable / Sustained Oscillation).

---

## Part 5: Frequency Response & Bode Stability Margins

### Q13: What are Gain Margin (GM) and Phase Margin (PM)?
**Answer:**
- **Gain Crossover Frequency ($\omega_{gc}$)**: Frequency where the open-loop magnitude is unity ($0\text{ dB}$): $|L(j\omega_{gc})| = 1$.
- **Phase Crossover Frequency ($\omega_{pc}$)**: Frequency where open-loop phase equals $-180^\circ$: $\angle L(j\omega_{pc}) = -180^\circ$.
- **Phase Margin (PM)**: The additional phase lag at $\omega_{gc}$ required to bring the system to the verge of instability:
  $$PM = 180^\circ + \angle L(j\omega_{gc})$$
- **Gain Margin (GM)**: The factor by which open-loop gain can be increased before the system becomes unstable:
  $$GM = -20\log_{10}|L(j\omega_{pc})| \text{ dB}$$

### Q14: How does Phase Margin relate to damping ratio $\zeta$?
**Answer:** For a prototype second-order system with low damping ($\zeta \le 0.6$):
$$\zeta \approx \frac{PM}{100}$$
For example, a Phase Margin of $50^\circ$ corresponds to approximately $\zeta \approx 0.5$, which yields about $16\%$ overshoot.

---

## Part 6: Controller Design & Real-World Constraints

### Q15: Why is pure Proportional (P) control alone insufficient for aircraft pitch tracking?
**Answer:**
1. A P-controller has finite gain, which leaves a persistent **steady-state error** ($e_{ss} \ne 0$) for step inputs.
2. Increasing $K_p$ reduces $e_{ss}$, but simultaneously decreases the phase margin, causing excessive oscillatory ringing and eventually driving the closed-loop poles into the Right-Half Plane (unstable).

### Q16: What is Integrator Windup, and how did you solve it?
**Answer:**
- **Problem**: When a large step pitch command is requested, the elevator actuator hits its physical deflection limit ($\pm 25^\circ$). While the actuator is pinned at its physical stop, the error $e(t)$ remains non-zero. A standard integral controller continues integrating this error, growing to a huge value ("winding up"). When the aircraft finally reaches the setpoint, the massive accumulated charge keeps the elevator saturated in the wrong direction, causing massive overshoot and delayed recovery.
- **Solution (Anti-Windup Back-Calculation)**: We monitor the difference between the saturated command and the unconstrained command:
  $$\dot{x}_i = e(t) + \frac{1}{T_t} (u_{\text{sat}} - u_{\text{unsat}})$$
  When saturated, $(u_{\text{sat}} - u_{\text{unsat}}) \ne 0$ feeds back negatively into the integrator, immediately halting windup and ensuring instant, damped recovery.

### Q17: Why did you include a derivative filter coefficient $N$?
**Answer:** An ideal derivative term $D(s) = K_d s$ has infinite gain at high frequencies ($\lim_{\omega \to \infty} |j\omega| = \infty$). Real aircraft pitch gyros have high-frequency measurement noise (vibration, electrical sensor noise). An ideal differentiator would amplify this noise into rapid elevator chatter, wearing out actuators. The 1st-order filter:
$$D(s) = K_d \frac{N s}{s + N}$$
acts as a band-limited differentiator, rolling off noise above $N\text{ rad/s}$ while retaining derivative damping at control frequencies.

### Q18: What is the Cascade Autopilot architecture?
**Answer:** In commercial and military aviation, pitch attitude control is structured as a two-loop cascade:
- **Fast Inner Loop**: Feeds back pitch rate $q$ measured by a high-bandwidth rate gyro. It augments pitch damping ($M_q$) and rapidly rejects wind gusts.
- **Outer Loop**: Feeds back pitch attitude $\theta$ or altitude $h$, commanding the inner loop's pitch rate setpoint $q_{\text{cmd}}$.
This decouples fast structural damping from slower navigational guidance.

---

## Part 7: Modern Control & State-Space

### Q19: What is the difference between Transfer Functions and State-Space representation?
**Answer:**
- **Transfer Functions (Classical)**: SISO (Single-Input Single-Output), applicable only to linear time-invariant systems with zero initial conditions, formulated in the frequency domain ($s$-domain).
- **State Space (Modern)**: MIMO (Multi-Input Multi-Output), handles multiple states, non-zero initial conditions, time-varying and non-linear systems, formulated directly in the time domain:
  $$\dot{x} = A x + B u, \quad y = C x + D u$$

### Q20: What is LQR (Linear Quadratic Regulator)?
**Answer:** LQR is an optimal state-feedback control technique that calculates constant gain matrix $K$ in $u = -K x$ to minimize a quadratic cost function:
$$J = \int_0^\infty \left( x^T Q x + u^T R u \right) dt$$
where $Q$ penalizes deviations of the aircraft states (speed, angle of attack, pitch rate, pitch angle) and $R$ penalizes elevator actuator effort. It guarantees infinite gain margin and at least $60^\circ$ of phase margin for symmetric SISO systems.
