/**
 * AeroSim-Control: Comprehensive Controller Suite
 * 
 * Implementations of:
 *  1. Open Loop (Manual / Uncontrolled)
 *  2. Proportional (P) Controller
 *  3. Proportional-Derivative (PD) with 1st-Order Low-Pass Derivative Filter (N)
 *  4. Full PID Controller
 *  5. PID with Anti-Windup (Conditional Integration & Back-Calculation)
 *  6. Lead-Lag Frequency Domain Compensator
 *  7. Cascade Autopilot (Inner Rate Damper Loop + Outer Attitude Tracking Loop)
 *  8. LQR (Linear Quadratic Regulator) Optimal State Feedback
 */

class AutopilotController {
    constructor() {
        this.mode = 'pid_antiwindup'; // default mode
        
        // Classical PID Gains (Tuned for aircraft pitch attitude control)
        this.Kp = 2.4;
        this.Ki = 0.85;
        this.Kd = 1.10;
        this.N = 15.0; // Derivative filter coefficient (rad/s)
        
        // Lead-Lag Parameters
        // Gc(s) = Kc * [(s + z_lead)/(s + p_lead)] * [(s + z_lag)/(s + p_lag)]
        this.Kc = 3.2;
        this.z_lead = 1.2;
        this.p_lead = 8.5; // p_lead > z_lead gives phase lead (boosts phase margin)
        this.z_lag = 0.5;
        this.p_lag = 0.05; // p_lag < z_lag gives high DC gain (reduces steady-state error)
        
        // Cascade Controller Gains
        this.Kq_inner = 0.75;      // Pitch rate damping gain
        this.Ktheta_outer = 2.2;   // Outer attitude error gain
        this.Ktheta_i = 0.6;       // Outer attitude integral gain
        
        // Actuator limits for Anti-Windup (rad)
        this.maxOutput = (25.0 * Math.PI) / 180.0; // ±25 deg
        this.minOutput = -this.maxOutput;
        this.antiWindupEnabled = true;
        this.antiWindupTrackingGain = 2.0; // 1 / Tt back-calculation gain
        
        // Internal state variables for controllers
        this.reset();
    }

    reset() {
        // PID states
        this.integral = 0.0;
        this.prevError = 0.0;
        this.derivativeState = 0.0;
        this.prevTheta = 0.0;
        
        // Lead-Lag filter internal states (Bilinear / Tustin filter states)
        this.lead_x = 0.0;
        this.lag_x = 0.0;
        
        // Cascade states
        this.cascadeOuterIntegral = 0.0;
        
        // Diagnostic metrics
        this.lastP_term = 0.0;
        this.lastI_term = 0.0;
        this.lastD_term = 0.0;
        this.lastRawCommand = 0.0;
        this.lastSaturatedCommand = 0.0;
        this.isWindupActive = false;
    }

    setMode(mode) {
        this.mode = mode;
        this.reset();
    }

    /**
     * Compute elevator command delta_e (radians)
     * 
     * @param {number} thetaRef Target pitch angle (rad)
     * @param {object} sensorData { thetaMeasured, qMeasured, altitudeMeasured }
     * @param {number} dt Time step (seconds)
     * @param {object} flightState Full aircraft state for LQR [u, w, q, theta]
     * @param {number} manualInput Manual elevator deflection for open-loop (rad)
     * @returns {number} Commanded elevator deflection delta_e (rad)
     */
    compute(thetaRef, sensorData, dt, flightState = null, manualInput = 0.0) {
        const theta = sensorData.thetaMeasured;
        const q = sensorData.qMeasured;
        const error = thetaRef - theta;

        let delta_e_cmd = 0.0;
        this.isWindupActive = false;

        switch (this.mode) {
            case 'open_loop':
                // Note: Elevator sign convention in aeronautics: negative delta_e produces positive pitch up!
                // We use standard intuitive sign: positive command pitches up.
                delta_e_cmd = -manualInput;
                this.lastP_term = 0;
                this.lastI_term = 0;
                this.lastD_term = 0;
                break;

            case 'p_only': {
                const P = this.Kp * error;
                // Elevator sign: pitching up requires trailing edge UP (-delta_e)
                delta_e_cmd = -P;
                this.lastP_term = P;
                this.lastI_term = 0;
                this.lastD_term = 0;
                break;
            }

            case 'pd': {
                // PD with derivative filter N
                // D(s) = Kd * s / (1 + s/N) * E(s)
                // In discrete form: d_state += dt * N * (error - d_state)
                // deriv = Kd * N * (error - d_state)
                const P = this.Kp * error;
                
                // Using derivative on measurement (derivative kick prevention):
                const dTheta = (theta - this.prevTheta) / Math.max(dt, 1e-5);
                this.prevTheta = theta;
                // Filtered derivative
                this.derivativeState += (dTheta - this.derivativeState) * (this.N * dt) / (1.0 + this.N * dt);
                const D = -this.Kd * this.derivativeState;

                delta_e_cmd = -(P + D);
                this.lastP_term = P;
                this.lastI_term = 0;
                this.lastD_term = D;
                break;
            }

            case 'pid_standard': {
                // Classical PID WITHOUT anti-windup (shows windup phenomenon!)
                const P = this.Kp * error;

                // Standard unconstrained integration
                this.integral += error * dt;
                const I = this.Ki * this.integral;

                // Filtered derivative on error
                const dTheta = (theta - this.prevTheta) / Math.max(dt, 1e-5);
                this.prevTheta = theta;
                this.derivativeState += (dTheta - this.derivativeState) * (this.N * dt) / (1.0 + this.N * dt);
                const D = -this.Kd * this.derivativeState;

                const rawCmd = -(P + I + D);
                delta_e_cmd = rawCmd;

                this.lastP_term = P;
                this.lastI_term = I;
                this.lastD_term = D;
                break;
            }

            case 'pid_antiwindup': {
                // Classical PID WITH Anti-Windup Clamping + Back-Calculation
                const P = this.Kp * error;

                // Filtered derivative
                const dTheta = (theta - this.prevTheta) / Math.max(dt, 1e-5);
                this.prevTheta = theta;
                this.derivativeState += (dTheta - this.derivativeState) * (this.N * dt) / (1.0 + this.N * dt);
                const D = -this.Kd * this.derivativeState;

                // Current unconstrained output candidate
                const unsatCmd = -(P + this.Ki * this.integral + D);
                
                // Check if saturated
                let satCmd = unsatCmd;
                if (unsatCmd > this.maxOutput) {
                    satCmd = this.maxOutput;
                } else if (unsatCmd < this.minOutput) {
                    satCmd = this.minOutput;
                }

                const saturationDifference = satCmd - unsatCmd; // (u_sat - u)
                if (Math.abs(saturationDifference) > 1e-4) {
                    this.isWindupActive = true;
                }

                // Anti-windup back-calculation:
                // d(integral)/dt = error + (1/Tt) * (satCmd - unsatCmd)
                if (this.antiWindupEnabled) {
                    // Back-calculation with tracking gain
                    const trackingCorrection = this.antiWindupTrackingGain * (-saturationDifference);
                    this.integral += (error - trackingCorrection) * dt;
                } else {
                    this.integral += error * dt;
                }

                // Prevent numeric drift
                this.integral = Math.max(Math.min(this.integral, 5.0), -5.0);

                const I = this.Ki * this.integral;
                delta_e_cmd = satCmd;

                this.lastP_term = P;
                this.lastI_term = I;
                this.lastD_term = D;
                break;
            }

            case 'lead_lag': {
                // Lead-Lag Compensator implemented via direct continuous-to-discrete states
                // Lead: (s + z1)/(s + p1) -> boosts phase margin around crossover frequency
                // Lag:  (s + z2)/(s + p2) -> boosts low frequency gain for zero ess
                
                // 1st Stage: Lead filter
                // dx_lead/dt = -p_lead * x_lead + error
                // y_lead = (z_lead - p_lead) * x_lead + error
                const dx_lead = -this.p_lead * this.lead_x + error;
                this.lead_x += dx_lead * dt;
                const lead_out = (this.z_lead - this.p_lead) * this.lead_x + error;

                // 2nd Stage: Lag filter
                // dx_lag/dt = -p_lag * x_lag + lead_out
                // y_lag = (z_lag - p_lag) * x_lag + lead_out
                const dx_lag = -this.p_lag * this.lag_x + lead_out;
                this.lag_x += dx_lag * dt;
                const lag_out = (this.z_lag - this.p_lag) * this.lag_x + lead_out;

                const u = this.Kc * lag_out;
                delta_e_cmd = -u;

                this.lastP_term = u;
                this.lastI_term = 0;
                this.lastD_term = 0;
                break;
            }

            case 'cascade': {
                // Cascade Autopilot Architecture:
                // Outer Loop: Pitch attitude error generates commanded pitch rate q_cmd
                // Inner Loop: High-bandwidth Pitch Rate Damper tracks q_cmd using pitch gyro q
                
                // Outer Loop (Attitude Hold)
                this.cascadeOuterIntegral += error * dt;
                this.cascadeOuterIntegral = Math.max(Math.min(this.cascadeOuterIntegral, 2.0), -2.0);
                
                const q_cmd = this.Ktheta_outer * error + this.Ktheta_i * this.cascadeOuterIntegral;
                // Limit commanded pitch rate to ±15 deg/s for passenger comfort / structural limit
                const max_q = (15.0 * Math.PI) / 180.0;
                const q_cmd_clamped = Math.max(Math.min(q_cmd, max_q), -max_q);

                // Inner Loop (Pitch Rate Damper)
                const q_error = q_cmd_clamped - q;
                const inner_cmd = this.Kq_inner * q_error;

                delta_e_cmd = -inner_cmd;

                this.lastP_term = this.Ktheta_outer * error;
                this.lastI_term = this.Ktheta_i * this.cascadeOuterIntegral;
                this.lastD_term = this.Kq_inner * q_error;
                break;
            }

            case 'lqr': {
                // Linear Quadratic Regulator (Optimal Full-State Feedback)
                // u = -K*x + Nbar*r
                // States: x = [u, w, q, theta]^T
                let K = [0.005, 0.08, 0.95, 2.8]; // Optimal gains for nominal airliner
                let Nbar = 2.8;

                if (flightState) {
                    const u_dev = flightState.u;
                    const w_dev = flightState.w;
                    const q_val = flightState.q;
                    const theta_val = flightState.theta;

                    const stateEffort = K[0]*u_dev + K[1]*w_dev + K[2]*q_val + K[3]*theta_val;
                    const feedForward = Nbar * thetaRef;
                    
                    const u_lqr = -stateEffort + feedForward;
                    delta_e_cmd = -u_lqr;

                    this.lastP_term = K[3] * (thetaRef - theta_val);
                    this.lastI_term = feedForward;
                    this.lastD_term = K[2] * q_val;
                }
                break;
            }

            default:
                delta_e_cmd = 0.0;
        }

        this.lastRawCommand = delta_e_cmd;
        
        // Clamp to physical elevator saturation
        if (delta_e_cmd > this.maxOutput) delta_e_cmd = this.maxOutput;
        if (delta_e_cmd < this.minOutput) delta_e_cmd = this.minOutput;
        this.lastSaturatedCommand = delta_e_cmd;

        return delta_e_cmd;
    }

    /**
     * Compute open-loop and closed-loop transfer functions for theoretical analysis
     */
    getClosedLoopTransferFunction(plantTF) {
        // In aeronautics, trailing-edge up (-delta_e) produces positive pitch up (+theta).
        // For closed-loop negative feedback stability, the loop gain magnitude is positive:
        const G_num = plantTF.num.map(c => Math.abs(c));
        const G_den = plantTF.den; // [1, 2*zeta*wn, wn^2, 0] = s * D2(s)
        const D2 = [G_den[0], G_den[1], G_den[2]];

        if (this.mode === 'open_loop') {
            // Open-loop characteristic polynomial is simply the plant denominator
            return {
                num: G_num,
                den: G_den
            };
        }

        if (this.mode === 'lead_lag') {
            // C(s) = Kc * (s + z1)(s + z2) / [(s + p1)(s + p2)]
            const Nc = [this.Kc, this.Kc * (this.z_lead + this.z_lag), this.Kc * this.z_lead * this.z_lag];
            const Dc = [1.0, (this.p_lead + this.p_lag), this.p_lead * this.p_lag];
            const Gden_Dc = this._polyMul(G_den, Dc);
            const Gnum_Nc = this._polyMul(G_num, Nc);
            const charPoly = this._polyAdd(Gden_Dc, Gnum_Nc);
            return {
                num: Gnum_Nc,
                den: charPoly
            };
        }

        if (this.mode === 'cascade') {
            // Cascade architecture: inner pitch rate damper Kq + outer attitude tracking Ktheta, Ki
            const C_num = [this.Kq_inner, this.Ktheta_outer, this.Ktheta_i];
            const s2_D2 = [...D2, 0.0, 0.0];
            const CG_num = this._polyMul(C_num, G_num);
            const charPoly = this._polyAdd(s2_D2, CG_num);
            return {
                num: CG_num,
                den: charPoly
            };
        }

        if (this.mode === 'lqr') {
            // LQR full-state feedback characteristic polynomial
            const charPoly = [1.0, 4.2, 8.5, 9.8, 3.2];
            return {
                num: G_num,
                den: charPoly
            };
        }

        // PID / PD / P-Only
        const Kp = Math.max(this.Kp, 0.0);
        const Ki = (this.mode === 'pd' || this.mode === 'p_only') ? 0.0 : Math.max(this.Ki, 0.0);
        const Kd = (this.mode === 'p_only') ? 0.0 : Math.max(this.Kd, 0.0);

        if (Ki > 0.001) {
            // PID: s^2 * D2(s) + (Kd*s^2 + Kp*s + Ki) * N(s)
            const s2_D2 = [...D2, 0.0, 0.0];
            const C_num = [Kd, Kp, Ki];
            const CG_num = this._polyMul(C_num, G_num);
            const charPoly = this._polyAdd(s2_D2, CG_num);
            return {
                num: CG_num,
                den: charPoly
            };
        } else {
            // PD or P-Only: s * D2(s) + (Kd*s + Kp) * N(s)
            const s_D2 = [...D2, 0.0];
            const C_num = [Kd, Kp];
            const CG_num = this._polyMul(C_num, G_num);
            const charPoly = this._polyAdd(s_D2, CG_num);
            return {
                num: CG_num,
                den: charPoly
            };
        }
    }

    _polyMul(p1, p2) {
        const result = new Array(p1.length + p2.length - 1).fill(0.0);
        for (let i = 0; i < p1.length; i++) {
            for (let j = 0; j < p2.length; j++) {
                result[i + j] += p1[i] * p2[j];
            }
        }
        return result;
    }

    _polyAdd(p1, p2) {
        const len = Math.max(p1.length, p2.length);
        const pad1 = new Array(len - p1.length).fill(0.0).concat(p1);
        const pad2 = new Array(len - p2.length).fill(0.0).concat(p2);
        const result = new Array(len);
        for (let i = 0; i < len; i++) {
            result[i] = pad1[i] + pad2[i];
        }
        return result;
    }
}

if (typeof window !== 'undefined') {
    window.AutopilotController = AutopilotController;
}
