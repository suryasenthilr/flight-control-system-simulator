/**
 * AeroSim-Control: Control Engineering Plotting & Analysis Engine
 * 
 * Includes:
 *  1. Real-time Multi-Channel Time Response Scopes
 *  2. Interactive Complex S-Plane Pole-Zero Map with Damping/Frequency Grids
 *  3. Interactive Bode Frequency Response Plot with Gain & Phase Margins
 *  4. Live Transient Metric Extractor (Rise time, Settling time, Overshoot, Ess)
 */

class ControlPlotSuite {
    constructor() {
        this.historyLength = 400; // 400 data points (~8 seconds of history at 50Hz)
        this.timeHistory = [];
        this.thetaHistory = [];
        this.thetaCmdHistory = [];
        this.deltaEHistory = [];
        this.altitudeHistory = [];

        this.initScopes();
        this.initSPlaneCanvas('sPlaneCanvas');
        this.initBodeCanvas('bodeCanvas');
    }

    initScopes() {
        // High-performance canvas-based scope for pitch
        this.pitchCanvas = document.getElementById('pitchScopeCanvas');
        this.pitchCtx = this.pitchCanvas ? this.pitchCanvas.getContext('2d') : null;

        // High-performance canvas-based scope for elevator
        this.elevatorCanvas = document.getElementById('elevatorScopeCanvas');
        this.elevatorCtx = this.elevatorCanvas ? this.elevatorCanvas.getContext('2d') : null;
    }

    pushData(time, thetaDeg, thetaCmdDeg, deltaEDeg, altitudeM) {
        this.timeHistory.push(time);
        this.thetaHistory.push(thetaDeg);
        this.thetaCmdHistory.push(thetaCmdDeg);
        this.deltaEHistory.push(deltaEDeg);
        this.altitudeHistory.push(altitudeM);

        if (this.timeHistory.length > this.historyLength) {
            this.timeHistory.shift();
            this.thetaHistory.shift();
            this.thetaCmdHistory.shift();
            this.deltaEHistory.shift();
            this.altitudeHistory.shift();
        }
    }

    reset() {
        this.timeHistory = [];
        this.thetaHistory = [];
        this.thetaCmdHistory = [];
        this.deltaEHistory = [];
        this.altitudeHistory = [];
    }

    /**
     * Compute step response transient metrics
     */
    computeTransientMetrics() {
        if (this.thetaHistory.length < 30) {
            return { tr: 0, tp: 0, ts: 0, Mp: 0, ess: 0 };
        }

        const target = this.thetaCmdHistory[this.thetaCmdHistory.length - 1];
        if (Math.abs(target) < 0.1) {
            // Trim / zero command condition
            const currentErr = Math.abs(this.thetaHistory[this.thetaHistory.length - 1]);
            return { tr: 0, tp: 0, ts: 0, Mp: 0, ess: currentErr };
        }

        const t0 = this.timeHistory[0];
        let t10 = null;
        let t90 = null;
        let peakVal = -Infinity;
        let peakTime = 0;
        let settlingTime = 0;

        const lowThresh = 0.10 * target;
        const highThresh = 0.90 * target;
        const tol2Percent = 0.02 * Math.abs(target);

        for (let i = 0; i < this.thetaHistory.length; i++) {
            const val = this.thetaHistory[i];
            const t = this.timeHistory[i];

            if (t10 === null && (target > 0 ? val >= lowThresh : val <= lowThresh)) {
                t10 = t;
            }
            if (t90 === null && (target > 0 ? val >= highThresh : val <= highThresh)) {
                t90 = t;
            }

            if (target > 0 ? val > peakVal : val < peakVal) {
                peakVal = val;
                peakTime = t;
            }

            // Settling time: time after which signal remains within ±2% of target
            if (Math.abs(val - target) > tol2Percent) {
                settlingTime = t;
            }
        }

        const tr = (t10 !== null && t90 !== null && t90 >= t10) ? (t90 - t10) : 0;
        const tp = peakTime - t0;
        const Mp = target !== 0 ? Math.max(0, ((peakVal - target) / target) * 100.0) : 0;
        const ts = Math.max(0, settlingTime - t0);
        const lastVal = this.thetaHistory[this.thetaHistory.length - 1];
        const ess = Math.abs(target - lastVal);

        return {
            tr: tr,
            tp: tp,
            ts: ts,
            Mp: Mp,
            ess: ess,
            peakVal: peakVal
        };
    }

    /**
     * Draw real-time pitch response scope
     */
    renderPitchScope() {
        if (!this.pitchCtx || this.timeHistory.length < 2) return;
        const ctx = this.pitchCtx;
        const w = this.pitchCanvas.width = this.pitchCanvas.clientWidth;
        const h = this.pitchCanvas.height = this.pitchCanvas.clientHeight;

        ctx.clearRect(0, 0, w, h);

        // Find min/max for dynamic scaling
        let minVal = -5;
        let maxVal = 15;
        for (let i = 0; i < this.thetaHistory.length; i++) {
            minVal = Math.min(minVal, this.thetaHistory[i], this.thetaCmdHistory[i]);
            maxVal = Math.max(maxVal, this.thetaHistory[i], this.thetaCmdHistory[i]);
        }
        const span = Math.max(maxVal - minVal, 10);
        const yMargin = span * 0.15;
        const yMin = minVal - yMargin;
        const yMax = maxVal + yMargin;

        const toY = (val) => h - ((val - yMin) / (yMax - yMin)) * h;
        const toX = (idx) => (idx / (this.historyLength - 1)) * w;

        // Draw grid
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
        ctx.lineWidth = 1;
        const step = span > 25 ? 10 : 5;
        for (let yVal = Math.ceil(yMin / step) * step; yVal <= yMax; yVal += step) {
            const y = toY(yVal);
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(w, y);
            ctx.stroke();

            ctx.fillStyle = '#64748b';
            ctx.font = '10px "JetBrains Mono", monospace';
            ctx.fillText(`${yVal.toFixed(0)}°`, 6, y - 3);
        }

        // Draw zero line
        const zeroY = toY(0);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
        ctx.beginPath();
        ctx.moveTo(0, zeroY);
        ctx.lineTo(w, zeroY);
        ctx.stroke();

        // Draw Command Reference (dashed magenta line)
        ctx.strokeStyle = '#ff2df4';
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        for (let i = 0; i < this.thetaCmdHistory.length; i++) {
            const x = toX(i);
            const y = toY(this.thetaCmdHistory[i]);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.setLineDash([]);

        // Draw Actual Pitch Response (solid cyan with glowing shadow)
        ctx.save();
        ctx.shadowColor = '#38bdf8';
        ctx.shadowBlur = 6;
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        for (let i = 0; i < this.thetaHistory.length; i++) {
            const x = toX(i);
            const y = toY(this.thetaHistory[i]);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.restore();

        // Scope Legend
        ctx.font = '10px "JetBrains Mono", monospace';
        ctx.fillStyle = '#ff2df4';
        ctx.fillText('--- Pitch Ref \u03B8_ref', w - 180, 16);
        ctx.fillStyle = '#38bdf8';
        ctx.fillText('\u2014 Actual Pitch \u03B8', w - 85, 16);
    }

    /**
     * Draw real-time elevator control effort scope
     */
    renderElevatorScope() {
        if (!this.elevatorCtx || this.deltaEHistory.length < 2) return;
        const ctx = this.elevatorCtx;
        const w = this.elevatorCanvas.width = this.elevatorCanvas.clientWidth;
        const h = this.elevatorCanvas.height = this.elevatorCanvas.clientHeight;

        ctx.clearRect(0, 0, w, h);

        const yMin = -30.0;
        const yMax = 30.0;
        const toY = (val) => h - ((val - yMin) / (yMax - yMin)) * h;
        const toX = (idx) => (idx / (this.historyLength - 1)) * w;

        // Draw grid
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
        ctx.lineWidth = 1;
        [-20, -10, 0, 10, 20].forEach(val => {
            const y = toY(val);
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(w, y);
            ctx.stroke();
            ctx.fillStyle = '#64748b';
            ctx.font = '9px "JetBrains Mono", monospace';
            ctx.fillText(`${val}°`, 6, y - 2);
        });

        // Saturation Limit Lines (Red dashed lines at ±25°)
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);
        const satUpperY = toY(25);
        const satLowerY = toY(-25);

        ctx.beginPath();
        ctx.moveTo(0, satUpperY);
        ctx.lineTo(w, satUpperY);
        ctx.moveTo(0, satLowerY);
        ctx.lineTo(w, satLowerY);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = '#ef4444';
        ctx.font = '9px "JetBrains Mono", monospace';
        ctx.fillText('+25° Limit', w - 65, satUpperY - 3);
        ctx.fillText('-25° Limit', w - 65, satLowerY + 11);

        // Draw Elevator Deflection (Amber curve)
        ctx.save();
        ctx.shadowColor = '#f59e0b';
        ctx.shadowBlur = 5;
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 2;
        ctx.beginPath();
        for (let i = 0; i < this.deltaEHistory.length; i++) {
            const x = toX(i);
            const y = toY(this.deltaEHistory[i]);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.restore();

        ctx.fillStyle = '#f59e0b';
        ctx.font = '10px "JetBrains Mono", monospace';
        ctx.fillText('\u03B4e Elevator (\u00B0)', 10, 16);
    }

    initSPlaneCanvas(canvasId) {
        this.sPlaneCanvas = document.getElementById(canvasId);
        this.sPlaneCtx = this.sPlaneCanvas ? this.sPlaneCanvas.getContext('2d') : null;
    }

    initBodeCanvas(canvasId) {
        this.bodeCanvas = document.getElementById(canvasId);
        this.bodeCtx = this.bodeCanvas ? this.bodeCanvas.getContext('2d') : null;
    }

    /**
     * Render Interactive S-Plane Pole-Zero Map
     * Shows open-loop and closed-loop poles/zeros, damping lines, and stability region
     */
    renderSPlane(plantTF, controller) {
        if (!this.sPlaneCtx) return;
        const ctx = this.sPlaneCtx;
        const w = this.sPlaneCanvas.width = this.sPlaneCanvas.clientWidth || 450;
        const h = this.sPlaneCanvas.height = this.sPlaneCanvas.clientHeight || 350;

        ctx.clearRect(0, 0, w, h);

        const cx = w * 0.65; // origin shifted right to give more room for Left Half Plane
        const cy = h * 0.5;
        const scale = Math.min(w, h) * 0.085; // pixels per unit sigma/omega

        // 1. Fill Left-Half-Plane (LHP: Stable) and Right-Half-Plane (RHP: Unstable)
        ctx.fillStyle = 'rgba(34, 197, 94, 0.06)'; // Soft green LHP
        ctx.fillRect(0, 0, cx, h);

        ctx.fillStyle = 'rgba(239, 68, 68, 0.08)'; // Soft red RHP
        ctx.fillRect(cx, 0, w - cx, h);

        // 2. Draw Constant Damping Ratio Lines (zeta = cos phi)
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);

        [0.707, 0.5, 0.3].forEach(zeta => {
            const angle = Math.acos(zeta);
            const r = 12 * scale;
            // Top damping line
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx - r * Math.cos(angle), cy - r * Math.sin(angle));
            ctx.stroke();
            // Bottom damping line
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx - r * Math.cos(angle), cy + r * Math.sin(angle));
            ctx.stroke();

            ctx.fillStyle = '#64748b';
            ctx.font = '9px "JetBrains Mono", monospace';
            ctx.fillText(`\u03B6=${zeta}`, cx - (r * 0.75) * Math.cos(angle) - 30, cy - (r * 0.75) * Math.sin(angle));
        });

        // 3. Draw Natural Frequency Circles (wn = 1, 2, 4, 6 rad/s)
        [1.0, 2.5, 5.0].forEach(wn => {
            const r = wn * scale;
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, 2 * Math.PI);
            ctx.stroke();
            ctx.fillText(`\u03C9n=${wn}`, cx - r - 2, cy - 4);
        });
        ctx.setLineDash([]);

        // 4. Main Coordinate Axes (Real sigma and Imaginary j*omega)
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
        ctx.lineWidth = 1.5;
        // Real axis (sigma)
        ctx.beginPath();
        ctx.moveTo(0, cy);
        ctx.lineTo(w, cy);
        ctx.stroke();
        // Imaginary axis (j*omega) - The boundary of stability!
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(cx, 0);
        ctx.lineTo(cx, h);
        ctx.stroke();

        // Labels
        ctx.fillStyle = '#e2e8f0';
        ctx.font = 'bold 11px "JetBrains Mono", monospace';
        ctx.fillText('\u03C3 (Real)', w - 60, cy - 8);
        ctx.fillText('j\u03C9 (Imag)', cx + 8, 16);

        ctx.fillStyle = '#22c55e';
        ctx.fillText('STABLE (LHP)', 15, 20);
        ctx.fillStyle = '#ef4444';
        ctx.fillText('UNSTABLE (RHP)', cx + 15, 20);

        // 5. Compute & Draw Poles & Zeros
        // Plant Short-Period Open-Loop Poles:
        // Denominator: s * (s^2 + 2*zeta*wn*s + wn^2) = 0
        // Root 1: s = 0 (integrator)
        // Roots 2,3: -zeta*wn ± j*wn*sqrt(1 - zeta^2)
        const wn = plantTF.wn;
        const zeta = plantTF.zeta;
        const sigma_sp = -zeta * wn;
        const omega_d = wn * Math.sqrt(Math.max(0, 1.0 - zeta * zeta));

        // Open-Loop Zero: s = -1/T_theta2
        const z1 = -1.0 / plantTF.T_theta2;

        // Draw Open-Loop Zero (circle 'O')
        this._drawSPlaneZero(ctx, cx + z1 * scale, cy, '#38bdf8', 'z1');

        // Draw Open-Loop Poles (cross 'X')
        this._drawSPlanePole(ctx, cx, cy, '#94a3b8', 's=0');
        this._drawSPlanePole(ctx, cx + sigma_sp * scale, cy - omega_d * scale, '#94a3b8', 'p_sp');
        this._drawSPlanePole(ctx, cx + sigma_sp * scale, cy + omega_d * scale, '#94a3b8', 'p_sp*');

        // 6. Draw Closed-Loop Poles with Active Controller (Highlighted Squares)
        // Closed loop pole locations shift with Kp, Ki, Kd
        const closedLoopPoles = this._estimateClosedLoopPoles(plantTF, controller);
        closedLoopPoles.forEach((p, idx) => {
            const px = cx + p.real * scale;
            const py = cy - p.imag * scale;
            const isUnstable = p.real > 0.001;

            ctx.save();
            ctx.shadowColor = isUnstable ? '#ef4444' : '#4ade80';
            ctx.shadowBlur = 8;
            ctx.fillStyle = isUnstable ? '#ef4444' : '#4ade80';
            ctx.fillRect(px - 5, py - 5, 10, 10);
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 1.5;
            ctx.strokeRect(px - 5, py - 5, 10, 10);
            ctx.restore();

            ctx.fillStyle = isUnstable ? '#ef4444' : '#4ade80';
            ctx.font = '10px "JetBrains Mono", monospace';
            const poleLabel = p.imag !== 0 
                ? `${p.real.toFixed(2)}${p.imag >= 0 ? '+' : ''}${p.imag.toFixed(2)}j` 
                : `${p.real.toFixed(2)}`;
            ctx.fillText(poleLabel, px + 8, py - 4);
        });

        // S-Plane Legend
        ctx.fillStyle = '#94a3b8';
        ctx.font = '10px "JetBrains Mono", monospace';
        ctx.fillText('\u2715 Open-Loop Pole', 15, h - 30);
        ctx.fillStyle = '#38bdf8';
        ctx.fillText('\u25EF Open-Loop Zero', 15, h - 16);
        ctx.fillStyle = '#4ade80';
        ctx.fillText('\u25A0 Closed-Loop Pole', 140, h - 16);
    }

    _drawSPlanePole(ctx, x, y, color, label) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.5;
        const size = 6;
        ctx.beginPath();
        ctx.moveTo(x - size, y - size);
        ctx.lineTo(x + size, y + size);
        ctx.moveTo(x - size, y + size);
        ctx.lineTo(x + size, y - size);
        ctx.stroke();

        if (label) {
            ctx.fillStyle = color;
            ctx.font = '9px "JetBrains Mono", monospace';
            ctx.fillText(label, x + 8, y + 3);
        }
    }

    _drawSPlaneZero(ctx, x, y, color, label) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.arc(x, y, 6, 0, 2 * Math.PI);
        ctx.stroke();

        if (label) {
            ctx.fillStyle = color;
            ctx.font = '9px "JetBrains Mono", monospace';
            ctx.fillText(label, x + 8, y + 3);
        }
    }

    _estimateClosedLoopPoles(plantTF, controller) {
        // Compute roots of characteristic polynomial: 1 + C(s)*G(s) = 0
        const clTF = controller.getClosedLoopTransferFunction(plantTF);
        const poly = clTF.den;

        return this._findPolynomialRoots(poly);
    }

    /**
     * Numerical root finder (Eigenvalues of Companion Matrix via QR algorithm)
     */
    _findPolynomialRoots(poly) {
        // Strip leading zeros
        const p = [...poly];
        while (p.length > 0 && Math.abs(p[0]) < 1e-12) p.shift();
        const n = p.length - 1;
        if (n <= 0) return [];

        // Normalize leading coefficient
        const an = p[0];
        const a = p.map(c => c / an);

        if (n === 1) {
            return [{ real: -a[1], imag: 0 }];
        }

        if (n === 2) {
            const b = a[1];
            const c = a[2];
            const disc = b * b - 4 * c;
            if (disc >= 0) {
                return [
                    { real: (-b + Math.sqrt(disc)) / 2, imag: 0 },
                    { real: (-b - Math.sqrt(disc)) / 2, imag: 0 }
                ];
            } else {
                return [
                    { real: -b / 2, imag: Math.sqrt(-disc) / 2 },
                    { real: -b / 2, imag: -Math.sqrt(-disc) / 2 }
                ];
            }
        }

        // For n >= 3, build Companion Matrix and compute eigenvalues
        const comp = [];
        for (let i = 0; i < n; i++) {
            comp[i] = new Array(n).fill(0.0);
            if (i > 0) comp[i][i - 1] = 1.0;
            comp[i][n - 1] = -a[n - i];
        }

        return this._computeMatrixEigenvalues(comp);
    }

    _computeMatrixEigenvalues(mat) {
        // Power-iteration / shifted QR iteration for companion matrix
        // For project scope, return accurate roots using Durand-Kerner algorithm
        const n = mat.length;
        // Durand-Kerner (Weierstrass) root finder
        const roots = [];
        const radius = 2.5;
        for (let i = 0; i < n; i++) {
            const angle = (2 * Math.PI * i) / n + 0.35;
            roots.push({
                real: Math.pow(radius, i / n) * Math.cos(angle),
                imag: Math.pow(radius, i / n) * Math.sin(angle)
            });
        }

        // Reconstruct polynomial coeffs from companion matrix
        const a = [1.0];
        for (let i = n - 1; i >= 0; i--) {
            a.push(-mat[i][n - 1]);
        }

        // Iterate Durand-Kerner
        for (let iter = 0; iter < 45; iter++) {
            for (let i = 0; i < n; i++) {
                let f = this._evalComplexPoly(a, roots[i]);
                let denom = { real: 1.0, imag: 0.0 };

                for (let j = 0; j < n; j++) {
                    if (i === j) continue;
                    const diff = {
                        real: roots[i].real - roots[j].real,
                        imag: roots[i].imag - roots[j].imag
                    };
                    denom = this._complexMul(denom, diff);
                }

                const delta = this._complexDiv(f, denom);
                roots[i].real -= delta.real;
                roots[i].imag -= delta.imag;
            }
        }

        return roots;
    }

    _evalComplexPoly(poly, s) {
        let res = { real: poly[0], imag: 0.0 };
        for (let i = 1; i < poly.length; i++) {
            res = this._complexAdd(this._complexMul(res, s), { real: poly[i], imag: 0.0 });
        }
        return res;
    }

    _complexAdd(a, b) {
        return { real: a.real + b.real, imag: a.imag + b.imag };
    }

    _complexMul(a, b) {
        return {
            real: a.real * b.real - a.imag * b.imag,
            imag: a.real * b.imag + a.imag * b.real
        };
    }

    _complexDiv(a, b) {
        const d = b.real * b.real + b.imag * b.imag || 1e-12;
        return {
            real: (a.real * b.real + a.imag * b.imag) / d,
            imag: (a.imag * b.real - a.real * b.imag) / d
        };
    }

    /**
     * Render Interactive Bode Plot (Magnitude in dB and Phase in degrees)
     */
    renderBode(plantTF, controller) {
        if (!this.bodeCtx) return;
        const ctx = this.bodeCtx;
        const w = this.bodeCanvas.width = this.bodeCanvas.clientWidth || 450;
        const h = this.bodeCanvas.height = this.bodeCanvas.clientHeight || 380;

        ctx.clearRect(0, 0, w, h);

        const margin = { left: 55, right: 25, top: 25, bottom: 25, gap: 20 };
        const plotH = (h - margin.top - margin.bottom - margin.gap) / 2;

        const wMin = 0.05; // rad/s
        const wMax = 50.0; // rad/s
        const logMin = Math.log10(wMin);
        const logMax = Math.log10(wMax);

        const freqToX = (freq) => {
            const lf = Math.log10(freq);
            return margin.left + ((lf - logMin) / (logMax - logMin)) * (w - margin.left - margin.right);
        };

        // 1. Compute Open-Loop Frequency Response L(jw) = C(jw)*G(jw)
        const N_pts = 150;
        const freqs = [];
        const magsDb = [];
        const phasesDeg = [];

        let gainCrossoverFreq = null; // where Mag = 0 dB
        let phaseCrossoverFreq = null; // where Phase = -180 deg
        let phaseAtGainCrossover = null;
        let magAtPhaseCrossover = null;

        for (let i = 0; i < N_pts; i++) {
            const omega = Math.pow(10, logMin + (i / (N_pts - 1)) * (logMax - logMin));
            freqs.push(omega);

            // G(jw)
            const s = { real: 0.0, imag: omega };
            const numVal = this._evalComplexPoly(plantTF.num, s);
            const denVal = this._evalComplexPoly(plantTF.den, s);
            const G_raw = this._complexDiv(numVal, denVal);
            // In closed loop, negative feedback inverts the negative actuator sign:
            const G_jw = (controller.mode === 'open_loop') 
                ? G_raw 
                : { real: -G_raw.real, imag: -G_raw.imag };

            // Compute C(jw) dynamically based on active controller mode
            let C_jw = { real: 1.0, imag: 0.0 };
            const mode = controller.mode;
            if (mode === 'open_loop') {
                C_jw = { real: 1.0, imag: 0.0 };
            } else if (mode === 'p_only') {
                C_jw = { real: Math.max(controller.Kp, 0.01), imag: 0.0 };
            } else if (mode === 'pd') {
                const N = controller.N || 15.0;
                const d_den = 1.0 + (omega / N) * (omega / N);
                C_jw = {
                    real: controller.Kp + (controller.Kd * omega * (omega / N)) / d_den,
                    imag: (controller.Kd * omega) / d_den
                };
            } else if (mode === 'lead_lag') {
                const num = {
                    real: controller.Kc * (controller.z_lead * controller.z_lag - omega * omega),
                    imag: controller.Kc * omega * (controller.z_lead + controller.z_lag)
                };
                const den = {
                    real: (controller.p_lead * controller.p_lag - omega * omega),
                    imag: omega * (controller.p_lead + controller.p_lag)
                };
                C_jw = this._complexDiv(num, den);
            } else if (mode === 'cascade') {
                C_jw = {
                    real: controller.Ktheta_outer,
                    imag: -controller.Ktheta_i / omega + controller.Kq_inner * omega
                };
            } else {
                // PID Standard, PID Anti-Windup, LQR
                const Kp = controller.Kp;
                const Ki = controller.Ki;
                const Kd = controller.Kd;
                const N = controller.N || 15.0;
                const d_den = 1.0 + (omega / N) * (omega / N);
                C_jw = {
                    real: Kp + (Kd * omega * (omega / N)) / d_den,
                    imag: -Ki / omega + (Kd * omega) / d_den
                };
            }

            const L_jw = this._complexMul(C_jw, G_jw);
            const mag = Math.sqrt(L_jw.real * L_jw.real + L_jw.imag * L_jw.imag);
            const magDb = 20.0 * Math.log10(Math.max(mag, 1e-6));
            let phaseDeg = (Math.atan2(L_jw.imag, L_jw.real) * 180.0) / Math.PI;

            // Phase unwrapping
            if (phaseDeg > 0) phaseDeg -= 360.0;

            magsDb.push(magDb);
            phasesDeg.push(phaseDeg);

            // Detect 0 dB Crossover (Gain Crossover)
            if (i > 0 && gainCrossoverFreq === null) {
                if ((magsDb[i - 1] >= 0 && magDb < 0) || (magsDb[i - 1] <= 0 && magDb > 0)) {
                    gainCrossoverFreq = omega;
                    phaseAtGainCrossover = phaseDeg;
                }
            }

            // Detect -180 deg Crossover (Phase Crossover)
            if (i > 0 && phaseCrossoverFreq === null) {
                if ((phasesDeg[i - 1] >= -180 && phaseDeg < -180) || (phasesDeg[i - 1] <= -180 && phaseDeg > -180)) {
                    phaseCrossoverFreq = omega;
                    magAtPhaseCrossover = magDb;
                }
            }
        }

        // Phase Margin PM = 180 + Phase(w_gc)
        const PM = phaseAtGainCrossover !== null ? 180.0 + phaseAtGainCrossover : null;
        // Gain Margin GM = -Mag(w_pc)
        const GM = magAtPhaseCrossover !== null ? -magAtPhaseCrossover : null;

        // 2. Draw Top Magnitude Plot (dB)
        const magTop = margin.top;
        const magBottom = magTop + plotH;
        const dbMin = -40;
        const dbMax = 40;
        const magToY = (db) => magBottom - ((db - dbMin) / (dbMax - dbMin)) * plotH;

        // Grid lines
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
        ctx.lineWidth = 1;
        [-40, -20, 0, 20, 40].forEach(db => {
            const y = magToY(db);
            ctx.beginPath();
            ctx.moveTo(margin.left, y);
            ctx.lineTo(w - margin.right, y);
            ctx.stroke();

            ctx.fillStyle = db === 0 ? '#38bdf8' : '#64748b';
            ctx.font = '9px "JetBrains Mono", monospace';
            ctx.fillText(`${db}dB`, 15, y + 3);
        });

        // 0 dB Line
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(margin.left, magToY(0));
        ctx.lineTo(w - margin.right, magToY(0));
        ctx.stroke();

        // Magnitude Curve
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        for (let i = 0; i < N_pts; i++) {
            const x = freqToX(freqs[i]);
            const y = magToY(Math.max(Math.min(magsDb[i], dbMax), dbMin));
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();

        // 3. Draw Bottom Phase Plot (Degrees)
        const phaseTop = magBottom + margin.gap;
        const phaseBottom = phaseTop + plotH;
        const degMin = -270;
        const degMax = -90;
        const phaseToY = (deg) => phaseBottom - ((deg - degMin) / (degMax - degMin)) * plotH;

        [-270, -225, -180, -135, -90].forEach(deg => {
            const y = phaseToY(deg);
            ctx.beginPath();
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
            ctx.moveTo(margin.left, y);
            ctx.lineTo(w - margin.right, y);
            ctx.stroke();

            ctx.fillStyle = deg === -180 ? '#ef4444' : '#64748b';
            ctx.font = '9px "JetBrains Mono", monospace';
            ctx.fillText(`${deg}°`, 15, y + 3);
        });

        // -180 deg Line (Phase Crossover Line)
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(margin.left, phaseToY(-180));
        ctx.lineTo(w - margin.right, phaseToY(-180));
        ctx.stroke();

        // Phase Curve
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        for (let i = 0; i < N_pts; i++) {
            const x = freqToX(freqs[i]);
            const y = phaseToY(Math.max(Math.min(phasesDeg[i], degMax), degMin));
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();

        // 4. Mark Gain and Phase Margins
        if (gainCrossoverFreq !== null && PM !== null) {
            const x_gc = freqToX(gainCrossoverFreq);
            ctx.strokeStyle = '#22c55e';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 3]);
            ctx.beginPath();
            ctx.moveTo(x_gc, magToY(0));
            ctx.lineTo(x_gc, phaseToY(-180 + PM));
            ctx.stroke();
            ctx.setLineDash([]);

            // Phase Margin Arrow
            ctx.fillStyle = '#22c55e';
            ctx.font = 'bold 10px "JetBrains Mono", monospace';
            ctx.fillText(`PM = ${PM.toFixed(1)}° (@ \u03C9gc = ${gainCrossoverFreq.toFixed(2)} rad/s)`, x_gc + 6, phaseToY(-180) - 15);
        }

        // Labels & Titles
        ctx.fillStyle = '#e2e8f0';
        ctx.font = 'bold 10px "JetBrains Mono", monospace';
        ctx.fillText('Magnitude |L(j\u03C9)| [dB]', margin.left + 5, magTop + 14);
        ctx.fillText('Phase \u2220L(j\u03C9) [deg]', margin.left + 5, phaseTop + 14);

        ctx.fillStyle = '#64748b';
        ctx.fillText('0.1', freqToX(0.1), h - 8);
        ctx.fillText('1.0', freqToX(1.0), h - 8);
        ctx.fillText('10', freqToX(10.0), h - 8);
        ctx.fillText('Frequency \u03C9 (rad/s)', w * 0.5 - 40, h - 8);

        return {
            PM: PM,
            GM: GM,
            gainCrossoverFreq: gainCrossoverFreq,
            phaseCrossoverFreq: phaseCrossoverFreq
        };
    }
}

if (typeof window !== 'undefined') {
    window.ControlPlotSuite = ControlPlotSuite;
}
