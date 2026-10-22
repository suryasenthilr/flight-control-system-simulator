/**
 * AeroSim-Control: Automated Routh-Hurwitz Stability Criterion Analyzer
 * 
 * Computes:
 *  - Full Routh Array from characteristic polynomial coefficients
 *  - Handles zero elements with epsilon perturbation (epsilon -> 0+)
 *  - Counts 1st-column sign changes (equal to number of RHP unstable poles)
 *  - Formats step-by-step mathematical explanation for viva / lab evaluation
 */

class RouthHurwitzAnalyzer {
    /**
     * Compute Routh Array for polynomial: a_n * s^n + a_{n-1} * s^{n-1} + ... + a_1 * s + a_0
     * @param {number[]} poly Coefficients in descending order of s powers [a_n, a_{n-1}, ..., a_0]
     */
    static compute(poly) {
        // Remove leading zeros
        let coeffs = [...poly];
        while (coeffs.length > 0 && Math.abs(coeffs[0]) < 1e-12) {
            coeffs.shift();
        }

        if (coeffs.length < 2) {
            return {
                isStable: false,
                reason: 'Polynomial degree too low',
                table: [],
                signChanges: 0,
                rhpRoots: 0
            };
        }

        // Normalize leading coefficient to positive if negative
        if (coeffs[0] < 0) {
            coeffs = coeffs.map(c => -c);
        }

        const n = coeffs.length - 1; // degree
        const numCols = Math.ceil((n + 1) / 2);
        const table = []; // each row: { power: string, values: number[] }

        // Row 1: s^n (coeffs with indices 0, 2, 4, ...)
        const row1 = [];
        for (let j = 0; j < coeffs.length; j += 2) {
            row1.push(coeffs[j]);
        }
        while (row1.length < numCols) row1.push(0.0);
        table.push({ power: `s^${n}`, values: row1 });

        // Row 2: s^{n-1} (coeffs with indices 1, 3, 5, ...)
        const row2 = [];
        for (let j = 1; j < coeffs.length; j += 2) {
            row2.push(coeffs[j]);
        }
        while (row2.length < numCols) row2.push(0.0);
        table.push({ power: `s^${n - 1}`, values: row2 });

        const EPSILON = 1e-6;

        // Populate subsequent rows: s^{n-2} down to s^0
        for (let r = 2; r <= n; r++) {
            const currentPower = `s^${n - r}`;
            const prevRow = table[r - 1].values;
            const prevPrevRow = table[r - 2].values;
            const newRow = [];

            let pivot = prevRow[0];
            if (Math.abs(pivot) < 1e-10) {
                // Epsilon replacement for zero pivot
                pivot = EPSILON;
            }

            for (let c = 0; c < numCols - 1; c++) {
                const a = prevPrevRow[0];
                const b = prevPrevRow[c + 1] || 0.0;
                const c_val = prevRow[0] === 0 ? EPSILON : prevRow[0];
                const d = prevRow[c + 1] || 0.0;

                // Determinant formula: (c_val * b - a * d) / c_val
                const val = (c_val * b - a * d) / c_val;
                newRow.push(Math.abs(val) < 1e-10 ? 0.0 : val);
            }
            newRow.push(0.0); // trailing zero
            table.push({ power: currentPower, values: newRow });
        }

        // Analyze first column for sign changes
        const firstCol = table.map(row => row.values[0]);
        let signChanges = 0;
        let prevSign = Math.sign(firstCol[0] >= 0 ? 1 : -1);

        const signs = [prevSign];
        for (let i = 1; i < firstCol.length; i++) {
            const val = firstCol[i];
            const currentSign = Math.sign(val >= 0 ? 1 : -1);
            if (currentSign !== prevSign) {
                signChanges++;
                prevSign = currentSign;
            }
            signs.push(currentSign);
        }

        let isStable = (signChanges === 0);
        const rhpRoots = signChanges;
        const lhpRoots = n - rhpRoots;

        const isMarginal = (Math.abs(firstCol[firstCol.length - 1]) < 1e-5);

        let verdict = '';
        if (isMarginal && signChanges === 0) {
            verdict = `MARGINALLY STABLE (Pole at s = 0 -> Open-loop integrator kinematic pole, neutral attitude drift).`;
            isStable = true;
        } else if (isStable) {
            verdict = `STABLE (0 sign changes in column 1 -> all ${n} poles reside in the Open Left-Half Plane).`;
        } else {
            verdict = `UNSTABLE (${signChanges} sign change(s) in column 1 -> ${rhpRoots} pole(s) in Right-Half Plane, ${lhpRoots} in Left-Half Plane).`;
        }

        return {
            degree: n,
            coeffs,
            table,
            firstCol,
            signs,
            signChanges,
            rhpRoots,
            lhpRoots,
            isStable,
            isMarginal,
            verdict
        };
    }

    /**
     * Render HTML table format of the Routh array
     */
    static renderToHTML(analysis) {
        const badgeClass = analysis.isMarginal ? 'badge-info' : (analysis.isStable ? 'badge-success' : 'badge-danger');
        const badgeText = analysis.isMarginal ? 'MARGINALLY STABLE (s=0)' : (analysis.isStable ? 'SYSTEM STABLE' : 'SYSTEM UNSTABLE');

        let html = `
        <div class="routh-card">
            <div class="routh-header">
                <span class="badge ${badgeClass}">
                    ${badgeText}
                </span>
                <span class="routh-degree">Degree: n = ${analysis.degree}</span>
            </div>
            
            <div class="routh-equation">
                <strong>Characteristic Equation:</strong><br>
                <code>${this.formatPolynomial(analysis.coeffs)} = 0</code>
            </div>

            <div class="table-responsive">
                <table class="routh-table">
                    <thead>
                        <tr>
                            <th>Order</th>
                            <th>Col 1 (Pivot)</th>
                            <th>Col 2</th>
                            <th>Col 3</th>
                        </tr>
                    </thead>
                    <tbody>
        `;

        for (let i = 0; i < analysis.table.length; i++) {
            const row = analysis.table[i];
            const pivotVal = row.values[0];
            const isNegative = pivotVal < 0;
            const col1Class = isNegative ? 'cell-neg' : 'cell-pos';

            html += `
                <tr>
                    <td class="row-power">${row.power}</td>
                    <td class="pivot-cell ${col1Class}">${pivotVal.toFixed(4)}</td>
                    <td>${(row.values[1] || 0.0).toFixed(4)}</td>
                    <td>${(row.values[2] || 0.0).toFixed(4)}</td>
                </tr>
            `;
        }

        html += `
                    </tbody>
                </table>
            </div>

            <div class="routh-summary ${analysis.isStable ? 'text-success' : 'text-danger'}">
                <strong>Verdict:</strong> ${analysis.verdict}
            </div>
            <div class="routh-explanation text-muted">
                <small>According to the Routh-Hurwitz Stability Criterion, the number of roots with positive real parts (unstable poles) equals the number of sign reversals in the first column of the Routh array.</small>
            </div>
        </div>
        `;

        return html;
    }

    static formatPolynomial(coeffs) {
        const n = coeffs.length - 1;
        const terms = [];

        for (let i = 0; i <= n; i++) {
            const power = n - i;
            const c = coeffs[i];
            if (Math.abs(c) < 1e-5 && i !== n) continue;

            const sign = (i > 0 && c >= 0) ? '+ ' : (c < 0 ? '- ' : '');
            const absC = Math.abs(c);
            const numStr = (absC === 1.0 && power > 0) ? '' : absC.toFixed(3);

            let termStr = '';
            if (power > 1) {
                termStr = `${sign}${numStr}s^${power}`;
            } else if (power === 1) {
                termStr = `${sign}${numStr}s`;
            } else {
                termStr = `${sign}${absC.toFixed(3)}`;
            }

            terms.push(termStr);
        }

        return terms.join(' ');
    }
}

if (typeof window !== 'undefined') {
    window.RouthHurwitzAnalyzer = RouthHurwitzAnalyzer;
}
