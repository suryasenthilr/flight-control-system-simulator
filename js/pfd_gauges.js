/**
 * AeroSim-Control: Glass Cockpit Primary Flight Display (PFD)
 * 
 * Draws an authentic avionics Primary Flight Display on HTML5 Canvas:
 *  - Artificial Horizon with Pitch Ladder & Roll Pointer
 *  - Commanded Pitch Guidance Chevron (Flight Director)
 *  - Airspeed Tape (left) & Altimeter Tape (right)
 *  - Vertical Speed Indicator (VSI)
 *  - Flight Mode Annunciator (FMA) showing autopilot state & limits
 *  - Actuator Elevator Position Gauge (bottom)
 */

class PrimaryFlightDisplay {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) return;
        this.ctx = this.canvas.getContext('2d');
        this.resize();
        window.addEventListener('resize', () => this.resize());
    }

    resize() {
        if (!this.canvas) return;
        const rect = this.canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;
        this.ctx.scale(dpr, dpr);
        this.width = rect.width;
        this.height = rect.height;
    }

    render(flightState, controllerState, targetThetaDeg = 0.0) {
        if (!this.ctx) return;
        const ctx = this.ctx;
        const w = this.width;
        const h = this.height;

        ctx.clearRect(0, 0, w, h);

        const pitchDeg = (flightState.theta * 180.0) / Math.PI;
        const rollDeg = 0.0; // longitudinal study focuses on symmetric pitch
        const pitchCmdDeg = targetThetaDeg;
        const altitudeM = flightState.altitude;
        const altitudeFt = altitudeM * 3.28084;
        const airspeedKts = flightState.airspeed * 1.94384;
        const vsiFpm = flightState.h_dot * 196.85; // m/s to ft/min
        const elevatorDeg = (flightState.delta_e * 180.0) / Math.PI;

        // Center coordinates for attitude ball
        const cx = w * 0.5;
        const cy = h * 0.52;
        const horizonRadius = Math.min(w * 0.38, h * 0.38);

        // 1. Draw Horizon Ball (Clipped Circle)
        ctx.save();
        ctx.beginPath();
        ctx.arc(cx, cy, horizonRadius, 0, 2 * Math.PI);
        ctx.clip();

        // Translate and rotate for pitch & roll
        ctx.save();
        ctx.translate(cx, cy);
        ctx.rotate((-rollDeg * Math.PI) / 180.0);

        // Pixels per pitch degree
        const pxPerDeg = horizonRadius / 15.0; // 15 degrees visible in radius
        const pitchOffset = pitchDeg * pxPerDeg;

        // Draw Sky (Deep Aviation Blue)
        ctx.fillStyle = '#0f4c81';
        ctx.fillRect(-horizonRadius * 2, -horizonRadius * 2, horizonRadius * 4, horizonRadius * 2 + pitchOffset);

        // Draw Ground (Aviation Earth Brown / Dark Slate)
        ctx.fillStyle = '#5c3a21';
        ctx.fillRect(-horizonRadius * 2, pitchOffset, horizonRadius * 4, horizonRadius * 2);

        // Draw Horizon Line
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(-horizonRadius * 1.5, pitchOffset);
        ctx.lineTo(horizonRadius * 1.5, pitchOffset);
        ctx.stroke();

        // Pitch Ladder Bars (-30° to +30°)
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.85)';
        ctx.fillStyle = '#ffffff';
        ctx.font = '10px "Inter", "Segoe UI", sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.lineWidth = 1.5;

        for (let p = -30; p <= 30; p += 5) {
            if (p === 0) continue;
            const y = pitchOffset - p * pxPerDeg;
            const barW = (p % 10 === 0) ? 42 : 22;

            ctx.beginPath();
            if (p > 0) {
                // Positive pitch: solid lines with downward pips
                ctx.moveTo(-barW, y);
                ctx.lineTo(barW, y);
                ctx.moveTo(-barW, y);
                ctx.lineTo(-barW, y + 4);
                ctx.moveTo(barW, y);
                ctx.lineTo(barW, y + 4);
            } else {
                // Negative pitch: dashed lines with upward pips
                ctx.setLineDash([4, 4]);
                ctx.moveTo(-barW, y);
                ctx.lineTo(barW, y);
                ctx.stroke();
                ctx.setLineDash([]);
                ctx.beginPath();
                ctx.moveTo(-barW, y);
                ctx.lineTo(-barW, y - 4);
                ctx.moveTo(barW, y);
                ctx.lineTo(barW, y - 4);
            }
            ctx.stroke();

            // Ladder numbers
            if (p % 10 === 0) {
                ctx.fillText(Math.abs(p).toString(), -barW - 14, y);
                ctx.fillText(Math.abs(p).toString(), barW + 14, y);
            }
        }

        // Commanded Pitch Flight Director Guidance Cue (Magenta bar / chevrons)
        const cmdY = pitchOffset - pitchCmdDeg * pxPerDeg;
        ctx.strokeStyle = '#ff2df4';
        ctx.fillStyle = '#ff2df4';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-50, cmdY);
        ctx.lineTo(-20, cmdY);
        ctx.moveTo(20, cmdY);
        ctx.lineTo(50, cmdY);
        ctx.stroke();
        // Little chevron pointing to target
        ctx.beginPath();
        ctx.moveTo(0, cmdY - 4);
        ctx.lineTo(6, cmdY + 4);
        ctx.lineTo(-6, cmdY + 4);
        ctx.closePath();
        ctx.fill();

        ctx.restore(); // undo rotation/translation for horizon

        // Horizon border
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.restore(); // undo clipping

        // 2. Fixed Aircraft Symbol (Yellow reticle / crosshair)
        ctx.strokeStyle = '#ffea00';
        ctx.fillStyle = '#ffea00';
        ctx.lineWidth = 3;
        // Center dot
        ctx.beginPath();
        ctx.arc(cx, cy, 3, 0, 2 * Math.PI);
        ctx.fill();
        // Left wing reticle
        ctx.beginPath();
        ctx.moveTo(cx - 45, cy);
        ctx.lineTo(cx - 15, cy);
        ctx.lineTo(cx - 15, cy + 8);
        ctx.stroke();
        // Right wing reticle
        ctx.beginPath();
        ctx.moveTo(cx + 45, cy);
        ctx.lineTo(cx + 15, cy);
        ctx.lineTo(cx + 15, cy + 8);
        ctx.stroke();

        // 3. Airspeed Tape (Left Side)
        const tapeW = 48;
        const tapeH = horizonRadius * 1.8;
        const tapeY = cy - tapeH * 0.5;
        const spdX = cx - horizonRadius - tapeW - 6;

        ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.lineWidth = 1;
        ctx.fillRect(spdX, tapeY, tapeW, tapeH);
        ctx.strokeRect(spdX, tapeY, tapeW, tapeH);

        // Airspeed tick marks
        ctx.fillStyle = '#e2e8f0';
        ctx.font = '11px "JetBrains Mono", monospace';
        ctx.textAlign = 'right';
        const spdPxPerKts = 1.6;
        for (let s = Math.floor((airspeedKts - 40) / 10) * 10; s <= airspeedKts + 40; s += 10) {
            const yTick = cy - (s - airspeedKts) * spdPxPerKts;
            if (yTick >= tapeY + 5 && yTick <= tapeY + tapeH - 5) {
                ctx.beginPath();
                ctx.strokeStyle = 'rgba(255, 255, 255, 0.5)';
                ctx.moveTo(spdX + tapeW - 8, yTick);
                ctx.lineTo(spdX + tapeW, yTick);
                ctx.stroke();
                ctx.fillText(s.toFixed(0), spdX + tapeW - 12, yTick + 4);
            }
        }
        // Current Airspeed readout box
        ctx.fillStyle = '#0f172a';
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 1.5;
        ctx.fillRect(spdX - 2, cy - 14, tapeW + 4, 28);
        ctx.strokeRect(spdX - 2, cy - 14, tapeW + 4, 28);
        ctx.fillStyle = '#38bdf8';
        ctx.font = 'bold 13px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText(airspeedKts.toFixed(0), spdX + tapeW * 0.5, cy + 5);

        // 4. Altimeter Tape (Right Side)
        const altX = cx + horizonRadius + 6;
        ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.lineWidth = 1;
        ctx.fillRect(altX, tapeY, tapeW + 12, tapeH);
        ctx.strokeRect(altX, tapeY, tapeW + 12, tapeH);

        // Altimeter tick marks (100s of feet)
        const altPxPerFt = 0.22;
        ctx.fillStyle = '#e2e8f0';
        ctx.font = '10px "JetBrains Mono", monospace';
        ctx.textAlign = 'left';
        for (let a = Math.floor((altitudeFt - 300) / 100) * 100; a <= altitudeFt + 300; a += 100) {
            const yTick = cy - (a - altitudeFt) * altPxPerFt;
            if (yTick >= tapeY + 5 && yTick <= tapeY + tapeH - 5) {
                ctx.beginPath();
                ctx.strokeStyle = 'rgba(255, 255, 255, 0.5)';
                ctx.moveTo(altX, yTick);
                ctx.lineTo(altX + 8, yTick);
                ctx.stroke();
                ctx.fillText((a / 100).toFixed(0), altX + 12, yTick + 3);
            }
        }
        // Current Altitude readout box
        ctx.fillStyle = '#0f172a';
        ctx.strokeStyle = '#4ade80';
        ctx.lineWidth = 1.5;
        ctx.fillRect(altX - 2, cy - 14, tapeW + 16, 28);
        ctx.strokeRect(altX - 2, cy - 14, tapeW + 16, 28);
        ctx.fillStyle = '#4ade80';
        ctx.font = 'bold 12px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText(altitudeFt.toFixed(0) + 'ft', altX + (tapeW + 14) * 0.5, cy + 5);

        // 5. Vertical Speed Indicator (VSI) Tape (Far Right)
        const vsiX = altX + tapeW + 18;
        const vsiW = 14;
        ctx.fillStyle = 'rgba(15, 23, 42, 0.7)';
        ctx.fillRect(vsiX, tapeY + 20, vsiW, tapeH - 40);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.strokeRect(vsiX, tapeY + 20, vsiW, tapeH - 40);

        // VSI indicator needle
        const vsiMaxFpm = 2000;
        const vsiClamped = Math.max(Math.min(vsiFpm, vsiMaxFpm), -vsiMaxFpm);
        const vsiY = cy - (vsiClamped / vsiMaxFpm) * ((tapeH - 40) * 0.45);

        ctx.fillStyle = '#f59e0b';
        ctx.beginPath();
        ctx.moveTo(vsiX + vsiW, cy);
        ctx.lineTo(vsiX, vsiY);
        ctx.lineTo(vsiX + vsiW, vsiY);
        ctx.closePath();
        ctx.fill();

        // 6. Top Flight Mode Annunciator (FMA) Banner
        ctx.fillStyle = '#090d16';
        ctx.fillRect(0, 0, w, 32);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
        ctx.strokeRect(0, 0, w, 32);

        ctx.font = 'bold 11px "JetBrains Mono", monospace';
        ctx.textAlign = 'left';
        ctx.fillStyle = '#38bdf8';
        const modeTitle = (controllerState.mode || 'AUTOPILOT').toUpperCase();
        ctx.fillText(`AP: ${modeTitle}`, 12, 20);

        // Status indicators
        let statusX = w - 12;
        ctx.textAlign = 'right';

        if (flightState.isSaturated) {
            ctx.fillStyle = '#ef4444';
            ctx.fillText('[ACTUATOR SATURATED]', statusX, 20);
            statusX -= 150;
        }

        if (controllerState.isWindupActive) {
            ctx.fillStyle = '#f97316';
            ctx.fillText('[WINDUP CLAMPED]', statusX, 20);
            statusX -= 130;
        }

        if (flightState.isRateLimited) {
            ctx.fillStyle = '#eab308';
            ctx.fillText('[RATE LIMIT]', statusX, 20);
            statusX -= 100;
        }

        // 7. Elevator Deflection Bar Gauge (Bottom)
        const elevGaugeW = w * 0.6;
        const elevGaugeH = 10;
        const elevGaugeX = (w - elevGaugeW) * 0.5;
        const elevGaugeY = h - 22;

        ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
        ctx.fillRect(elevGaugeX, elevGaugeY, elevGaugeW, elevGaugeH);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.strokeRect(elevGaugeX, elevGaugeY, elevGaugeW, elevGaugeH);

        // Center zero mark
        ctx.strokeStyle = '#ffffff';
        ctx.beginPath();
        ctx.moveTo(elevGaugeX + elevGaugeW * 0.5, elevGaugeY - 2);
        ctx.lineTo(elevGaugeX + elevGaugeW * 0.5, elevGaugeY + elevGaugeH + 2);
        ctx.stroke();

        // Deflection indicator bar (±25 deg max)
        const maxDeflection = 25.0;
        const defFrac = Math.max(Math.min(elevatorDeg / maxDeflection, 1.0), -1.0);
        const defBarLength = (elevGaugeW * 0.5) * defFrac;

        ctx.fillStyle = Math.abs(elevatorDeg) >= 24.8 ? '#ef4444' : '#38bdf8';
        if (defBarLength >= 0) {
            ctx.fillRect(elevGaugeX + elevGaugeW * 0.5, elevGaugeY + 1, defBarLength, elevGaugeH - 2);
        } else {
            ctx.fillRect(elevGaugeX + elevGaugeW * 0.5 + defBarLength, elevGaugeY + 1, -defBarLength, elevGaugeH - 2);
        }

        ctx.font = '10px "JetBrains Mono", monospace';
        ctx.fillStyle = '#94a3b8';
        ctx.textAlign = 'center';
        ctx.fillText(`Elevator Deflection \u03B4e: ${elevatorDeg.toFixed(1)}\u00B0 [Limit: \u00B125\u00B0]`, w * 0.5, elevGaugeY - 5);
    }
}

if (typeof window !== 'undefined') {
    window.PrimaryFlightDisplay = PrimaryFlightDisplay;
}
