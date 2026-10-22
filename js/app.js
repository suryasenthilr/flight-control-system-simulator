/**
 * AeroSim-Control: Master Application Coordinator
 * 
 * Connects Flight Dynamics, Autopilot Controllers, 3D Visualization,
 * Primary Flight Display, Real-time Plotting, and Routh-Hurwitz Analysis.
 */

class AeroSimApp {
    constructor() {
        this.simTime = 0.0;
        this.dt = 0.015; // 15ms physics step
        this.isRunning = true;
        this.targetThetaDeg = 8.0; // default step command to 8 degrees pitch
        this.manualDeflectionDeg = 0.0;
        this.windGust = 0.0;
        this.noiseEnabled = false;

        // Subsystems
        this.dynamics = new FlightDynamics();
        this.controller = new AutopilotController();
        this.pfd = new PrimaryFlightDisplay('pfdCanvas');
        this.aircraft3D = new Aircraft3DView('aircraft3dContainer');
        this.plotSuite = new ControlPlotSuite();

        // Data recording for CSV export
        this.recordedData = [];

        this.initUI();
        this.bindEvents();
        this.updateAnalyticalViews();

        // Start real-time loop
        this.lastFrameTime = performance.now();
        requestAnimationFrame((t) => this.loop(t));
    }

    initUI() {
        // Sync sliders with default controller values
        this.setSlider('sliderKp', this.controller.Kp, 'valKp');
        this.setSlider('sliderKi', this.controller.Ki, 'valKi');
        this.setSlider('sliderKd', this.controller.Kd, 'valKd');
        this.setSlider('sliderN', this.controller.N, 'valN');
        this.setSlider('sliderPitchCmd', this.targetThetaDeg, 'valPitchCmd', '°');
        this.setSlider('sliderWind', this.windGust, 'valWind', ' m/s');

        // Aircraft details card
        this.updateAircraftInfo();
    }

    setSlider(id, val, labelId, suffix = '') {
        const slider = document.getElementById(id);
        const label = document.getElementById(labelId);
        if (slider) slider.value = val;
        if (label) label.textContent = typeof val === 'number' ? val.toFixed(2) + suffix : val + suffix;
    }

    bindEvents() {
        // Preset Aircraft Selector
        const presetSelect = document.getElementById('aircraftSelect');
        if (presetSelect) {
            presetSelect.addEventListener('change', (e) => {
                this.dynamics.setPreset(e.target.value);
                this.updateAircraftInfo();
                this.resetSimulation();
            });
        }

        // Controller Mode Selector
        const modeSelect = document.getElementById('controllerSelect');
        if (modeSelect) {
            modeSelect.addEventListener('change', (e) => {
                const mode = e.target.value;
                this.controller.setMode(mode);
                this.updateControllerPanels(mode);
                this.updateAnalyticalViews();
            });
        }

        // Interactive Sliders
        this.bindSlider('sliderKp', (v) => { this.controller.Kp = v; this.updateAnalyticalViews(); });
        this.bindSlider('sliderKi', (v) => { this.controller.Ki = v; this.updateAnalyticalViews(); });
        this.bindSlider('sliderKd', (v) => { this.controller.Kd = v; this.updateAnalyticalViews(); });
        this.bindSlider('sliderN', (v) => { this.controller.N = v; this.updateAnalyticalViews(); });
        this.bindSlider('sliderPitchCmd', (v) => { this.targetThetaDeg = v; });
        this.bindSlider('sliderWind', (v) => { this.windGust = v; });
        this.bindSlider('sliderManualElev', (v) => { this.manualDeflectionDeg = v; });

        // Anti-Windup Toggle
        const antiWindupCheck = document.getElementById('checkAntiWindup');
        if (antiWindupCheck) {
            antiWindupCheck.addEventListener('change', (e) => {
                this.controller.antiWindupEnabled = e.target.checked;
            });
        }

        // Sensor Noise Toggle
        const noiseCheck = document.getElementById('checkSensorNoise');
        if (noiseCheck) {
            noiseCheck.addEventListener('change', (e) => {
                this.noiseEnabled = e.target.checked;
            });
        }

        // Play/Pause Button
        const btnPlay = document.getElementById('btnPlayPause');
        if (btnPlay) {
            btnPlay.addEventListener('click', () => {
                this.isRunning = !this.isRunning;
                btnPlay.innerHTML = this.isRunning 
                    ? '<i class="icon">⏸</i> Pause' 
                    : '<i class="icon">▶</i> Resume';
            });
        }

        // Reset Button
        const btnReset = document.getElementById('btnReset');
        if (btnReset) {
            btnReset.addEventListener('click', () => this.resetSimulation());
        }

        // Step Command Quick Buttons
        document.querySelectorAll('.btn-quick-step').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const target = parseFloat(btn.dataset.step);
                this.targetThetaDeg = target;
                this.setSlider('sliderPitchCmd', target, 'valPitchCmd', '°');
            });
        });

        // Scenario Mission Buttons
        document.querySelectorAll('.btn-scenario').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const scenario = btn.dataset.scenario;
                this.runScenario(scenario);
            });
        });

        // Camera mode buttons
        document.querySelectorAll('.btn-cam').forEach(btn => {
            btn.addEventListener('click', () => {
                document.querySelectorAll('.btn-cam').forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                this.aircraft3D.setCameraMode(btn.dataset.cam);
            });
        });

        // Tab Navigation (Scopes, Bode, S-Plane, Routh, Theory)
        document.querySelectorAll('.tab-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
                document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
                btn.classList.add('active');
                const targetPane = document.getElementById(btn.dataset.tab);
                if (targetPane) targetPane.classList.add('active');
                setTimeout(() => this.updateAnalyticalViews(), 40);
            });
        });

        // CSV Export Button
        const btnExportCsv = document.getElementById('btnExportCsv');
        if (btnExportCsv) {
            btnExportCsv.addEventListener('click', () => this.exportCsv());
        }

        // Report Snapshot Button
        const btnSnapshot = document.getElementById('btnSnapshot');
        if (btnSnapshot) {
            btnSnapshot.addEventListener('click', () => window.print());
        }

        // Beginner Walkthrough Modal
        const helpModal = document.getElementById('helpModal');
        const btnHelp = document.getElementById('btnHelpTour');
        const btnCloseHelp = document.getElementById('btnCloseHelp');
        const btnDismissHelp = document.getElementById('btnDismissHelp');

        if (btnHelp && helpModal) {
            btnHelp.addEventListener('click', () => helpModal.classList.add('open'));
        }
        if (btnCloseHelp && helpModal) {
            btnCloseHelp.addEventListener('click', () => helpModal.classList.remove('open'));
        }
        if (btnDismissHelp && helpModal) {
            btnDismissHelp.addEventListener('click', () => helpModal.classList.remove('open'));
        }
        if (helpModal) {
            helpModal.addEventListener('click', (e) => {
                if (e.target === helpModal) helpModal.classList.remove('open');
            });
        }
    }

    bindSlider(id, callback) {
        const slider = document.getElementById(id);
        const labelId = 'val' + id.replace('slider', '');
        const label = document.getElementById(labelId);
        if (slider) {
            slider.addEventListener('input', (e) => {
                const val = parseFloat(e.target.value);
                const suffix = id.includes('Pitch') ? '°' : (id.includes('Wind') ? ' m/s' : (id.includes('Elev') ? '°' : ''));
                if (label) label.textContent = val.toFixed(2) + suffix;
                callback(val);
            });
        }
    }

    updateControllerPanels(mode) {
        // Show/hide relevant slider groups depending on controller
        const pidControls = document.getElementById('pidControlsGroup');
        const manualControls = document.getElementById('manualControlsGroup');
        const antiWindupRow = document.getElementById('antiWindupOptionRow');

        if (mode === 'open_loop') {
            if (pidControls) pidControls.style.display = 'none';
            if (manualControls) manualControls.style.display = 'block';
            if (antiWindupRow) antiWindupRow.style.display = 'none';
        } else {
            if (pidControls) pidControls.style.display = 'block';
            if (manualControls) manualControls.style.display = 'none';
            if (antiWindupRow) antiWindupRow.style.display = (mode.includes('pid') ? 'flex' : 'none');
        }
    }

    updateAircraftInfo() {
        const infoEl = document.getElementById('aircraftInfoText');
        const p = this.dynamics.preset;
        if (infoEl) {
            infoEl.innerHTML = `
                <strong>${p.name}</strong><br>
                <span class="text-muted">${p.description}</span><br>
                <span class="badge badge-info">Cruise U₀: ${p.U0} m/s (${(p.U0 * 1.944).toFixed(0)} kts)</span>
                <span class="badge badge-info">Short-Period ωₙ: ${p.sp_wn} rad/s</span>
                <span class="badge badge-info">Damping ζ: ${p.sp_zeta}</span>
            `;
        }
    }

    resetSimulation() {
        this.simTime = 0.0;
        this.dynamics.reset();
        this.controller.reset();
        this.plotSuite.reset();
        if (this.aircraft3D && this.aircraft3D.reset) {
            this.aircraft3D.reset();
        }
        this.recordedData = [];
        this.updateAnalyticalViews();
    }

    runScenario(scenario) {
        // Reset disturbances and toggles to default
        this.windGust = 0.0;
        this.noiseEnabled = false;
        this.setSlider('sliderWind', 0.0, 'valWind', ' m/s');

        const checkNoise = document.getElementById('checkSensorNoise');
        if (checkNoise) checkNoise.checked = false;
        const checkAW = document.getElementById('checkAntiWindup');
        if (checkAW) checkAW.checked = true;
        this.controller.antiWindupEnabled = true;

        switch (scenario) {
            case 'step_climb':
                this.dynamics.setPreset('b747');
                document.getElementById('aircraftSelect').value = 'b747';
                this.updateAircraftInfo();
                this.targetThetaDeg = 10.0;
                this.setSlider('sliderPitchCmd', 10.0, 'valPitchCmd', '°');
                this.controller.Kp = 2.4;
                this.controller.Ki = 0.85;
                this.controller.Kd = 1.1;
                this.controller.N = 15.0;
                this.controller.setMode('pid_antiwindup');
                document.getElementById('controllerSelect').value = 'pid_antiwindup';
                this.initUI();
                break;

            case 'turbulence':
                this.dynamics.setPreset('b747');
                document.getElementById('aircraftSelect').value = 'b747';
                this.updateAircraftInfo();
                this.targetThetaDeg = 5.0;
                this.setSlider('sliderPitchCmd', 5.0, 'valPitchCmd', '°');
                this.windGust = 12.0; // strong 12 m/s updraft
                this.setSlider('sliderWind', 12.0, 'valWind', ' m/s');
                this.controller.Kp = 2.4;
                this.controller.Ki = 0.85;
                this.controller.Kd = 1.1;
                this.controller.N = 15.0;
                this.controller.setMode('pid_antiwindup');
                document.getElementById('controllerSelect').value = 'pid_antiwindup';
                this.initUI();
                break;

            case 'windup_test':
                this.dynamics.setPreset('b747');
                document.getElementById('aircraftSelect').value = 'b747';
                this.updateAircraftInfo();
                this.targetThetaDeg = 18.0;
                this.setSlider('sliderPitchCmd', 18.0, 'valPitchCmd', '°');
                this.controller.Kp = 2.4;
                this.controller.Ki = 0.85;
                this.controller.Kd = 1.1;
                this.controller.N = 15.0;
                this.controller.setMode('pid_standard'); // No anti-windup
                document.getElementById('controllerSelect').value = 'pid_standard';
                this.controller.antiWindupEnabled = false;
                if (checkAW) checkAW.checked = false;
                this.initUI();
                break;

            case 'unstable_stabilization':
                // Switch to High-Agility Canard Fighter Jet
                this.dynamics.setPreset('unstable_canard');
                document.getElementById('aircraftSelect').value = 'unstable_canard';
                this.updateAircraftInfo();
                this.targetThetaDeg = 6.0;
                this.setSlider('sliderPitchCmd', 6.0, 'valPitchCmd', '°');
                this.controller.Kp = 3.5;
                this.controller.Kd = 1.6;
                this.controller.Ki = 0.8;
                this.controller.N = 15.0;
                this.controller.setMode('pid_antiwindup');
                document.getElementById('controllerSelect').value = 'pid_antiwindup';
                this.initUI();
                break;

            case 'sensor_noise':
                this.dynamics.setPreset('b747');
                document.getElementById('aircraftSelect').value = 'b747';
                this.updateAircraftInfo();
                this.targetThetaDeg = 5.0;
                this.setSlider('sliderPitchCmd', 5.0, 'valPitchCmd', '°');
                this.noiseEnabled = true;
                if (checkNoise) checkNoise.checked = true;
                this.controller.Kp = 2.4;
                this.controller.Ki = 0.85;
                this.controller.Kd = 1.1;
                this.controller.N = 8.0; // heavy low-pass filtering
                this.controller.setMode('pid_antiwindup');
                document.getElementById('controllerSelect').value = 'pid_antiwindup';
                this.initUI();
                break;
        }

        this.resetSimulation();
        this.updateControllerPanels(this.controller.mode);
        this.updateAnalyticalViews();
    }

    loop(currentTime) {
        requestAnimationFrame((t) => this.loop(t));

        if (!this.isRunning) {
            this.lastFrameTime = currentTime;
            return;
        }

        const elapsed = (currentTime - this.lastFrameTime) / 1000.0;
        this.lastFrameTime = currentTime;

        // Run physics integration steps (sub-stepping for smooth 60fps display)
        const subSteps = 2;
        const dtSub = this.dt / subSteps;

        let lastState = null;
        for (let s = 0; s < subSteps; s++) {
            this.simTime += dtSub;

            // 1. Get Sensor Measurements
            const sensorData = this.dynamics.getSensors(this.noiseEnabled);

            // 2. Compute Controller Actuator Command
            const targetThetaRad = (this.targetThetaDeg * Math.PI) / 180.0;
            const manualElevRad = (this.manualDeflectionDeg * Math.PI) / 180.0;
            const fullFlightState = {
                u: this.dynamics.u,
                w: this.dynamics.w,
                q: this.dynamics.q,
                theta: this.dynamics.theta
            };

            const delta_e_cmd = this.controller.compute(
                targetThetaRad,
                sensorData,
                dtSub,
                fullFlightState,
                manualElevRad
            );

            // 3. Step Aircraft Dynamics
            lastState = this.dynamics.step(dtSub, delta_e_cmd, this.windGust);
        }

        // Push data to plotting history
        const thetaDeg = (lastState.theta * 180.0) / Math.PI;
        const deltaEDeg = (lastState.delta_e * 180.0) / Math.PI;
        this.plotSuite.pushData(
            this.simTime,
            thetaDeg,
            this.targetThetaDeg,
            deltaEDeg,
            lastState.altitude
        );

        // Record for CSV export (every 5 steps)
        if (this.recordedData.length < 2500 && Math.floor(this.simTime / 0.05) > this.recordedData.length) {
            this.recordedData.push({
                time: this.simTime.toFixed(3),
                thetaRef: this.targetThetaDeg.toFixed(2),
                thetaActual: thetaDeg.toFixed(2),
                pitchRate: ((lastState.q * 180.0) / Math.PI).toFixed(2),
                elevatorCmd: ((lastState.delta_e_cmd * 180.0) / Math.PI).toFixed(2),
                elevatorActual: deltaEDeg.toFixed(2),
                altitude: lastState.altitude.toFixed(1),
                airspeed: lastState.airspeed.toFixed(1)
            });
        }

        // Render Visuals
        this.pfd.render(lastState, this.controller, this.targetThetaDeg);
        this.aircraft3D.update(lastState);
        this.plotSuite.renderPitchScope();
        this.plotSuite.renderElevatorScope();

        // Update Transient Metrics HUD
        this.updateTransientMetricsDisplay();
    }

    updateTransientMetricsDisplay() {
        const metrics = this.plotSuite.computeTransientMetrics();

        const trEl = document.getElementById('metricTr');
        const tpEl = document.getElementById('metricTp');
        const mpEl = document.getElementById('metricMp');
        const tsEl = document.getElementById('metricTs');
        const essEl = document.getElementById('metricEss');

        if (trEl) trEl.textContent = metrics.tr > 0 ? metrics.tr.toFixed(2) + ' s' : '—';
        if (tpEl) tpEl.textContent = metrics.tp > 0 ? metrics.tp.toFixed(2) + ' s' : '—';
        if (mpEl) mpEl.textContent = metrics.Mp.toFixed(1) + ' %';
        if (tsEl) tsEl.textContent = metrics.ts > 0 ? metrics.ts.toFixed(2) + ' s' : '—';
        if (essEl) essEl.textContent = metrics.ess.toFixed(2) + '°';

        // Telemetry header
        const simTimeEl = document.getElementById('telemetrySimTime');
        const altEl = document.getElementById('telemetryAlt');
        const spdEl = document.getElementById('telemetrySpeed');
        const qEl = document.getElementById('telemetryPitchRate');

        if (simTimeEl) simTimeEl.textContent = `T+ ${this.simTime.toFixed(1)}s`;
        if (altEl) altEl.textContent = `${this.dynamics.altitude.toFixed(0)} m`;
        if (spdEl) spdEl.textContent = `${(this.dynamics.preset.U0 + this.dynamics.u).toFixed(1)} m/s`;
        if (qEl) qEl.textContent = `${((this.dynamics.q * 180.0) / Math.PI).toFixed(2)} °/s`;
    }

    /**
     * Recompute S-Plane Pole-Zero, Bode Plot, and Routh-Hurwitz Array
     */
    updateAnalyticalViews() {
        const plantTF = this.dynamics.getShortPeriodTransferFunction();

        // 1. Render S-Plane Pole-Zero Map
        this.plotSuite.renderSPlane(plantTF, this.controller);

        // 2. Render Bode Plot & Margins
        const margins = this.plotSuite.renderBode(plantTF, this.controller);
        if (margins) {
            const pmEl = document.getElementById('metricPm');
            const gmEl = document.getElementById('metricGm');
            if (pmEl) pmEl.textContent = margins.PM !== null ? `${margins.PM.toFixed(1)}°` : '—';
            if (gmEl) gmEl.textContent = margins.GM !== null ? `${margins.GM.toFixed(1)} dB` : '—';
        }

        // 3. Compute Routh-Hurwitz Stability Table
        const clTF = this.controller.getClosedLoopTransferFunction(plantTF);
        const routhAnalysis = RouthHurwitzAnalyzer.compute(clTF.den);
        const routhContainer = document.getElementById('routhContainer');
        if (routhContainer) {
            routhContainer.innerHTML = RouthHurwitzAnalyzer.renderToHTML(routhAnalysis);
        }
    }

    exportCsv() {
        if (this.recordedData.length === 0) {
            alert('No simulation data recorded yet. Please run the simulation for a few seconds first.');
            return;
        }

        const headers = ['Time_s', 'ThetaRef_deg', 'ThetaActual_deg', 'PitchRate_deg_s', 'ElevatorCmd_deg', 'ElevatorActual_deg', 'Altitude_m', 'Airspeed_mps'];
        const csvRows = [headers.join(',')];

        for (const row of this.recordedData) {
            csvRows.push([
                row.time,
                row.thetaRef,
                row.thetaActual,
                row.pitchRate,
                row.elevatorCmd,
                row.elevatorActual,
                row.altitude,
                row.airspeed
            ].join(','));
        }

        const blob = new Blob([csvRows.join('\n')], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `AeroSim_FlightData_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '_')}.csv`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }
}

// Auto-boot on DOM ready
document.addEventListener('DOMContentLoaded', () => {
    window.app = new AeroSimApp();
});
