/**
 * AeroSim-Control: Flight Dynamics & Mathematical Modeling Engine
 * 
 * Implements 4th-order linearized longitudinal flight dynamics:
 * State vector: x = [u, w, q, theta]^T
 *   u: forward velocity perturbation (m/s)
 *   w: vertical velocity perturbation (m/s) (w ≈ U0 * alpha)
 *   q: pitch rate (rad/s)
 *   theta: pitch attitude angle (rad)
 * 
 * Actuator model:
 *   tau_a * d(delta_e)/dt + delta_e = delta_e_cmd
 *   Subject to position saturation [-25°, +25°] and rate limit [-30°/s, +30°/s]
 */

class FlightDynamics {
    constructor() {
        this.aircraftPresets = {
            'b747': {
                name: 'Boeing 747 Commercial Airliner',
                description: 'Heavy transport aircraft at cruise altitude (Mach 0.8, 40,000 ft). High inertia, distinct Phugoid and Short-Period modes.',
                U0: 236.0,          // Cruise speed (m/s)
                altitude0: 10000.0,  // Initial altitude (m)
                g: 9.81,
                // Stability derivatives in body-fixed stability axes (1/s, 1/(m*s), etc.)
                Xu: -0.0033,  Xw:  0.0380,  Zdelta_e: -8.50,
                Zu: -0.0900,  Zw: -0.3690,  Mdelta_e: -1.15,
                Mu:  0.0001,  Mw: -0.0021,  Mdot_w: -0.00015,
                Mq: -0.4920,
                // Actuator parameters
                actuatorTau: 0.10,       // Actuator time constant (s)
                maxDeflectionDeg: 25.0,   // Max elevator deflection (deg)
                maxRateDegPerSec: 30.0,   // Max elevator rate (deg/s)
                // Pitch transfer function parameters (approximate short period: theta(s)/delta_e(s))
                sp_wn: 0.95,
                sp_zeta: 0.45,
                T_theta2: 1.45,
                K_theta: -1.15
            },
            'f16': {
                name: 'F-16 Falcon (Agile Fighter)',
                description: 'High-agility fighter aircraft with relaxed static stability. Fast short-period mode requiring high bandwidth control.',
                U0: 180.0,
                altitude0: 5000.0,
                g: 9.81,
                Xu: -0.0150,  Xw:  0.0850,  Zdelta_e: -24.0,
                Zu: -0.1800,  Zw: -0.6500,  Mdelta_e: -11.5,
                Mu:  0.0005,  Mw: -0.0150,  Mdot_w: -0.0008,
                Mq: -1.2500,
                actuatorTau: 0.05,
                maxDeflectionDeg: 25.0,
                maxRateDegPerSec: 60.0,
                sp_wn: 3.20,
                sp_zeta: 0.60,
                T_theta2: 0.85,
                K_theta: -11.5
            },
            'c172': {
                name: 'Cessna 172 Skyhawk',
                description: 'Light general aviation aircraft at cruise (110 kts). Highly stable open-loop dynamics with well-damped short-period mode.',
                U0: 55.0,
                altitude0: 1500.0,
                g: 9.81,
                Xu: -0.0450,  Xw:  0.0950,  Zdelta_e: -12.0,
                Zu: -0.3200,  Zw: -1.4500,  Mdelta_e: -4.20,
                Mu:  0.0002,  Mw: -0.0450,  Mdot_w: -0.0020,
                Mq: -2.0500,
                actuatorTau: 0.08,
                maxDeflectionDeg: 28.0,
                maxRateDegPerSec: 40.0,
                sp_wn: 2.45,
                sp_zeta: 0.52,
                T_theta2: 1.10,
                K_theta: -4.20
            },
            'unstable_canard': {
                name: 'High-Agility Canard Fighter Jet',
                description: 'Forward-swept canard design with relaxed static stability. High pitch authority and fast short-period response.',
                U0: 180.0,
                altitude0: 4000.0,
                g: 9.81,
                Xu: -0.0150,  Xw:  0.0400,  Zdelta_e: -20.0,
                Zu: -0.1500,  Zw: -0.8500,  Mdelta_e: -15.0,
                Mu:  0.0002,  Mw:  0.0015,  Mdot_w: -0.0004,
                Mq: -1.8500,
                actuatorTau: 0.04,
                maxDeflectionDeg: 25.0,
                maxRateDegPerSec: 70.0,
                sp_wn: 2.80,
                sp_zeta: 0.65,
                T_theta2: 0.95,
                K_theta: -15.0
            }
        };

        this.currentPresetKey = 'b747';
        this.preset = this.aircraftPresets[this.currentPresetKey];
        this.reset();
    }

    setPreset(presetKey) {
        if (this.aircraftPresets[presetKey]) {
            this.currentPresetKey = presetKey;
            this.preset = this.aircraftPresets[presetKey];
            this.reset();
            return true;
        }
        return false;
    }

    reset() {
        // State variables:
        this.u = 0.0;       // m/s perturbation
        this.w = 0.0;       // m/s vertical perturbation
        this.q = 0.0;       // rad/s pitch rate
        this.theta = 0.0;   // rad pitch attitude
        this.altitude = this.preset.altitude0; // meters
        this.x_pos = 0.0;   // meters ground distance

        // Elevator actuator actual state
        this.delta_e = 0.0;      // rad
        this.delta_e_dot = 0.0;  // rad/s
        this.isSaturated = false;
        this.isRateLimited = false;

        // Disturbance inputs
        this.w_gust = 0.0;       // m/s updraft / downdraft gust
        this.sensorNoiseStd = 0.002; // rad (~0.11 deg)
    }

    /**
     * Build the 4x4 longitudinal state-space matrix A and 4x1 input matrix B
     */
    getStateSpaceMatrices() {
        const p = this.preset;
        const g = p.g;
        const U0 = p.U0;

        // Linearized Longitudinal Equations in Stability Axes:
        // u_dot = Xu*u + Xw*w - g*cos(theta0)*theta + Xde*de
        // w_dot = Zu*u + Zw*w + U0*q - g*sin(theta0)*theta + Zde*de
        // q_dot = (Mu + Mdot_w*Zu)*u + (Mw + Mdot_w*Zw)*w + (Mq + Mdot_w*U0)*q + (Mde + Mdot_w*Zde)*de
        // theta_dot = q

        const theta0 = 0.0; // level flight trim condition
        const A11 = p.Xu;
        const A12 = p.Xw;
        const A13 = 0.0;
        const A14 = -g * Math.cos(theta0);

        const A21 = p.Zu;
        const A22 = p.Zw;
        const A23 = U0;
        const A24 = -g * Math.sin(theta0);

        const A31 = p.Mu + p.Mdot_w * p.Zu;
        const A32 = p.Mw + p.Mdot_w * p.Zw;
        const A33 = p.Mq + p.Mdot_w * U0;
        const A34 = 0.0;

        const A41 = 0.0;
        const A42 = 0.0;
        const A43 = 1.0;
        const A44 = 0.0;

        const A = [
            [A11, A12, A13, A14],
            [A21, A22, A23, A24],
            [A31, A32, A33, A34],
            [A41, A42, A43, A44]
        ];

        const B = [
            0.0,
            p.Zdelta_e,
            p.Mdelta_e + p.Mdot_w * p.Zdelta_e,
            0.0
        ];

        return { A, B, U0 };
    }

    /**
     * Compute Short-Period Approximation Transfer Function:
     * G_sp(s) = theta(s) / delta_e(s) = K_theta * (s + 1/T_theta2) / [s * (s^2 + 2*zeta*wn*s + wn^2)]
     */
    getShortPeriodTransferFunction() {
        const p = this.preset;
        const K = p.K_theta;
        const zero1 = -1.0 / p.T_theta2;
        const wn = p.sp_wn;
        const zeta = p.sp_zeta;

        // Numerator: K * (s + 1/T_theta2) = K*s + K/T_theta2
        // Denominator: s * (s^2 + 2*zeta*wn*s + wn^2) = s^3 + 2*zeta*wn*s^2 + wn^2*s + 0
        const num = [K, K / p.T_theta2];
        const den = [1.0, 2.0 * zeta * wn, wn * wn, 0.0];

        return {
            num,
            den,
            wn,
            zeta,
            T_theta2: p.T_theta2,
            K
        };
    }

    /**
     * Full 4th-order Characteristic Polynomial:
     * det(sI - A) = s^4 + a3*s^3 + a2*s^2 + a1*s + a0
     */
    getCharacteristicPolynomial() {
        const { A } = this.getStateSpaceMatrices();
        // Compute coefficients using Faddeev-LeVerrier algorithm
        const n = 4;
        let Ak = this._identityMatrix(n);
        const poly = [1.0];

        for (let k = 1; k <= n; k++) {
            // M = A * Ak
            const M = this._matMul(A, Ak);
            const tr = this._trace(M);
            const pk = -tr / k;
            poly.push(pk);
            // Ak = M + pk * I
            Ak = this._matAdd(M, this._scalarMul(this._identityMatrix(n), pk));
        }

        return poly; // [1, a3, a2, a1, a0]
    }

    /**
     * High-Precision Runge-Kutta 4th Order (RK4) Step for Continuous Aircraft Physics
     * dt: time step (seconds, e.g. 0.01s)
     * delta_e_command: elevator command from controller in radians
     * disturbanceGust: updraft vertical wind in m/s
     */
    step(dt, delta_e_command, disturbanceGust = 0.0) {
        const p = this.preset;
        const maxDeflectionRad = (p.maxDeflectionDeg * Math.PI) / 180.0;
        const maxRateRadPerSec = (p.maxRateDegPerSec * Math.PI) / 180.0;

        // 1. Actuator Dynamics with Rate and Saturation limits
        let target_de = delta_e_command;
        this.isSaturated = false;
        if (target_de > maxDeflectionRad) {
            target_de = maxDeflectionRad;
            this.isSaturated = true;
        } else if (target_de < -maxDeflectionRad) {
            target_de = -maxDeflectionRad;
            this.isSaturated = true;
        }

        // First-order actuator lag: d(de)/dt = (target - de) / tau
        let de_rate = (target_de - this.delta_e) / p.actuatorTau;
        this.isRateLimited = false;
        if (de_rate > maxRateRadPerSec) {
            de_rate = maxRateRadPerSec;
            this.isRateLimited = true;
        } else if (de_rate < -maxRateRadPerSec) {
            de_rate = -maxRateRadPerSec;
            this.isRateLimited = true;
        }

        this.delta_e_dot = de_rate;
        this.delta_e += de_rate * dt;

        // Clamp actual deflection to physical stops
        if (this.delta_e > maxDeflectionRad) this.delta_e = maxDeflectionRad;
        if (this.delta_e < -maxDeflectionRad) this.delta_e = -maxDeflectionRad;

        // 2. Flight Dynamics RK4 State Integration
        const { A, B, U0 } = this.getStateSpaceMatrices();
        const de_actual = this.delta_e;
        this.w_gust = disturbanceGust;

        const deriv = (state) => {
            const [u, w, q, theta] = state;
            const w_eff = w + this.w_gust; // gust acts on aerodynamic forces

            const du = A[0][0]*u + A[0][1]*w_eff + A[0][2]*q + A[0][3]*theta + B[0]*de_actual;
            const dw = A[1][0]*u + A[1][1]*w_eff + A[1][2]*q + A[1][3]*theta + B[1]*de_actual;
            const dq = A[2][0]*u + A[2][1]*w_eff + A[2][2]*q + A[2][3]*theta + B[2]*de_actual;
            const dtheta = q;

            return [du, dw, dq, dtheta];
        };

        const s0 = [this.u, this.w, this.q, this.theta];

        // k1
        const k1 = deriv(s0);

        // k2
        const s1 = [
            s0[0] + 0.5 * dt * k1[0],
            s0[1] + 0.5 * dt * k1[1],
            s0[2] + 0.5 * dt * k1[2],
            s0[3] + 0.5 * dt * k1[3]
        ];
        const k2 = deriv(s1);

        // k3
        const s2 = [
            s0[0] + 0.5 * dt * k2[0],
            s0[1] + 0.5 * dt * k2[1],
            s0[2] + 0.5 * dt * k2[2],
            s0[3] + 0.5 * dt * k2[3]
        ];
        const k3 = deriv(s2);

        // k4
        const s3 = [
            s0[0] + dt * k3[0],
            s0[1] + dt * k3[1],
            s0[2] + dt * k3[2],
            s0[3] + dt * k3[3]
        ];
        const k4 = deriv(s3);

        // Update states
        this.u += (dt / 6.0) * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
        this.w += (dt / 6.0) * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);
        this.q += (dt / 6.0) * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]);
        this.theta += (dt / 6.0) * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]);

        // Physical sanity clamping: prevent runaway spinning/tumbling
        const maxPitchRad = (80.0 * Math.PI) / 180.0;
        if (this.theta > maxPitchRad) {
            this.theta = maxPitchRad;
            if (this.q > 0) this.q = 0;
        } else if (this.theta < -maxPitchRad) {
            this.theta = -maxPitchRad;
            if (this.q < 0) this.q = 0;
        }

        // Clamp speed perturbations to realistic flight envelope
        this.u = Math.max(Math.min(this.u, 100.0), -100.0);
        this.w = Math.max(Math.min(this.w, 80.0), -80.0);
        this.q = Math.max(Math.min(this.q, 2.5), -2.5); // max ±143 deg/s pitch rate

        // Angle of attack alpha ≈ w / U0 (rad)
        const alpha = this.w / U0;
        // Flight path angle gamma = theta - alpha
        const gamma = this.theta - alpha;
        // Altitude rate h_dot = U0 * sin(gamma) ≈ U0 * gamma
        const h_dot = (U0 + this.u) * Math.sin(gamma);
        this.altitude += h_dot * dt;
        this.x_pos += (U0 + this.u) * Math.cos(this.theta) * dt;

        return {
            u: this.u,
            w: this.w,
            q: this.q,
            theta: this.theta,
            alpha: alpha,
            gamma: gamma,
            altitude: this.altitude,
            h_dot: h_dot,
            airspeed: U0 + this.u,
            delta_e: this.delta_e,
            delta_e_cmd: delta_e_command,
            isSaturated: this.isSaturated,
            isRateLimited: this.isRateLimited
        };
    }

    /**
     * Get simulated sensor reading with optional noise
     */
    getSensors(noiseEnabled = false) {
        let noiseTheta = 0.0;
        let noiseQ = 0.0;
        if (noiseEnabled) {
            // Box-Muller Gaussian Noise
            const u1 = Math.random() || 0.0001;
            const u2 = Math.random();
            const z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
            const z1 = Math.sqrt(-2.0 * Math.log(u1)) * Math.sin(2.0 * Math.PI * u2);
            noiseTheta = z0 * this.sensorNoiseStd;
            noiseQ = z1 * (this.sensorNoiseStd * 2.5); // gyro rate noise
        }
        return {
            thetaMeasured: this.theta + noiseTheta,
            qMeasured: this.q + noiseQ,
            altitudeMeasured: this.altitude
        };
    }

    // Helper matrix functions
    _identityMatrix(n) {
        const I = [];
        for (let i = 0; i < n; i++) {
            const row = new Array(n).fill(0.0);
            row[i] = 1.0;
            I.push(row);
        }
        return I;
    }

    _matMul(A, B) {
        const n = A.length;
        const C = [];
        for (let i = 0; i < n; i++) {
            C[i] = new Array(n).fill(0.0);
            for (let j = 0; j < n; j++) {
                let sum = 0.0;
                for (let k = 0; k < n; k++) {
                    sum += A[i][k] * B[k][j];
                }
                C[i][j] = sum;
            }
        }
        return C;
    }

    _matAdd(A, B) {
        const n = A.length;
        const C = [];
        for (let i = 0; i < n; i++) {
            C[i] = new Array(n);
            for (let j = 0; j < n; j++) {
                C[i][j] = A[i][j] + B[i][j];
            }
        }
        return C;
    }

    _scalarMul(A, s) {
        const n = A.length;
        const C = [];
        for (let i = 0; i < n; i++) {
            C[i] = new Array(n);
            for (let j = 0; j < n; j++) {
                C[i][j] = A[i][j] * s;
            }
        }
        return C;
    }

    _trace(A) {
        let tr = 0.0;
        for (let i = 0; i < A.length; i++) {
            tr += A[i][i];
        }
        return tr;
    }
}

// Export for browser
if (typeof window !== 'undefined') {
    window.FlightDynamics = FlightDynamics;
}
