"""
AeroSim-Control: Analytical Control Engineering Verification Suite
Companion Python analysis tool using NumPy, SciPy, and Matplotlib.

Computes:
1. State-Space Matrices (A, B, C, D) for Aircraft Longitudinal Dynamics
2. Eigenvalues, Short-Period and Phugoid Modes (wn, zeta)
3. Transfer Functions G(s) = theta(s) / delta_e(s)
4. Routh-Hurwitz Stability Criterion Table Generation
5. Frequency Domain Analysis: Bode Plot, Gain Margin & Phase Margin
6. Time Domain Simulation: Step Responses and Anti-Windup Comparison
7. Generates publication-ready figures for project reports
"""

import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt
import os

# Set publication style
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.sans-serif'] = 'Arial'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.titlesize'] = 11
plt.rcParams['axes.labelsize'] = 10

class AircraftControlAnalysis:
    def __init__(self, aircraft_name="Boeing 747 Cruising"):
        self.name = aircraft_name
        self.g = 9.81
        
        # Nominal Boeing 747 Cruise at Mach 0.8 (40,000 ft)
        self.U0 = 236.0  # m/s
        self.Xu = -0.0033
        self.Xw = 0.0380
        self.Zu = -0.0900
        self.Zw = -0.3690
        self.Mu = 0.0001
        self.Mw = -0.0021
        self.Mdot_w = -0.00015
        self.Mq = -0.4920
        self.Zde = -8.50
        self.Mde = -1.15
        
        self.build_state_space()
        self.build_transfer_functions()

    def build_state_space(self):
        """Construct longitudinal 4th-order state space: x = [u, w, q, theta]^T"""
        theta0 = 0.0
        g = self.g
        U0 = self.U0
        
        self.A = np.array([
            [self.Xu, self.Xw, 0.0, -g * np.cos(theta0)],
            [self.Zu, self.Zw, U0, -g * np.sin(theta0)],
            [self.Mu + self.Mdot_w * self.Zu, self.Mw + self.Mdot_w * self.Zw, self.Mq + self.Mdot_w * U0, 0.0],
            [0.0, 0.0, 1.0, 0.0]
        ])
        
        self.B = np.array([
            [0.0],
            [self.Zde],
            [self.Mde + self.Mdot_w * self.Zde],
            [0.0]
        ])
        
        # Outputs: [theta, q, alpha]
        self.C = np.array([
            [0.0, 0.0, 0.0, 1.0],      # theta (rad)
            [0.0, 0.0, 1.0, 0.0],      # q (rad/s)
            [0.0, 1.0 / U0, 0.0, 0.0]  # alpha (rad)
        ])
        
        self.D = np.zeros((3, 1))

    def build_transfer_functions(self):
        """Derive transfer function theta(s) / delta_e(s)"""
        # Pitch state is index 3
        C_theta = np.array([[0.0, 0.0, 0.0, 1.0]])
        D_theta = np.array([[0.0]])
        
        num, den = signal.ss2tf(self.A, self.B, C_theta, D_theta)
        self.num_full = num[0]
        self.den_full = den
        
        # Short-period approximation parameters
        # G_sp(s) = Mde * (s + 1/T_theta2) / [s * (s^2 + 2*zeta*wn*s + wn^2)]
        self.sp_wn = 0.95
        self.sp_zeta = 0.45
        self.T_theta2 = 1.45
        self.K_theta = -1.15
        
        # Numerator: K*(s + 1/T2)
        # Denominator: s*(s^2 + 2*zeta*wn*s + wn^2)
        self.num_sp = np.array([self.K_theta, self.K_theta / self.T_theta2])
        self.den_sp = np.array([1.0, 2.0 * self.sp_zeta * self.sp_wn, self.sp_wn**2, 0.0])
        
        self.plant_sp = signal.TransferFunction(self.num_sp, self.den_sp)

    def analyze_modes(self):
        """Compute open-loop eigenvalues and modal frequencies"""
        eigenvals = np.linalg.eigvals(self.A)
        modes = []
        for eig in eigenvals:
            real = eig.real
            imag = eig.imag
            wn = np.abs(eig)
            zeta = -real / wn if wn > 1e-6 else 0.0
            modes.append({'pole': eig, 'wn': wn, 'zeta': zeta})
        return modes

    def compute_routh_array(self, poly):
        """Generate Routh-Hurwitz stability table"""
        poly = np.trim_zeros(poly, 'f')
        n = len(poly) - 1
        num_cols = int(np.ceil((n + 1) / 2))
        table = np.zeros((n + 1, num_cols))
        
        # Row 0 (s^n)
        table[0, :len(poly[0::2])] = poly[0::2]
        # Row 1 (s^(n-1))
        table[1, :len(poly[1::2])] = poly[1::2]
        
        for r in range(2, n + 1):
            for c in range(num_cols - 1):
                pivot = table[r - 1, 0]
                if np.abs(pivot) < 1e-9:
                    pivot = 1e-6
                table[r, c] = (pivot * table[r - 2, c + 1] - table[r - 2, 0] * table[r - 1, c + 1]) / pivot
                
        # Count sign changes in column 0
        first_col = table[:, 0]
        sign_changes = 0
        prev_sign = np.sign(first_col[0])
        for val in first_col[1:]:
            curr_sign = np.sign(val)
            if curr_sign != prev_sign and curr_sign != 0:
                sign_changes += 1
                prev_sign = curr_sign
                
        is_stable = (sign_changes == 0)
        return table, sign_changes, is_stable

    def design_pid_closed_loop(self, Kp=2.4, Ki=0.85, Kd=1.1, N=15.0):
        """Construct closed-loop transfer function with filtered PID"""
        # C(s) = (Kd*s^2 + Kp*s + Ki) / s
        C_num = [Kd, Kp, Ki]
        C_den = [1.0, 0.0]
        
        # Open-loop L(s) = -C(s) * G(s) (negative feedback with aerodynamic elevator sign)
        # In aero: trailing edge up (-delta_e) produces pitch up (+theta)
        L_num = np.convolve(C_num, -self.num_sp)
        L_den = np.convolve(C_den, self.den_sp)
        
        # Closed loop T(s) = L(s) / (1 + L(s))
        T_num = L_num
        pad_len = max(len(L_den), len(L_num))
        d_padded = np.pad(L_den, (pad_len - len(L_den), 0))
        n_padded = np.pad(L_num, (pad_len - len(L_num), 0))
        T_den = d_padded + n_padded
        
        return signal.TransferFunction(T_num, T_den), signal.TransferFunction(L_num, L_den)

    def generate_all_plots(self, output_dir="."):
        """Generate high-resolution analysis figures for project report"""
        os.makedirs(output_dir, exist_ok=True)
        
        # 1. Step Response Comparison
        t = np.linspace(0, 15, 1000)
        step_target = 10.0 # 10 degrees
        
        # Controller variations
        tfs = {
            'Proportional Only (Kp=2.4)': self.design_pid_closed_loop(Kp=2.4, Ki=0.0, Kd=0.0)[0],
            'PD Controller (Kp=2.4, Kd=1.1)': self.design_pid_closed_loop(Kp=2.4, Ki=0.0, Kd=1.1)[0],
            'Full PID (Kp=2.4, Ki=0.85, Kd=1.1)': self.design_pid_closed_loop(Kp=2.4, Ki=0.85, Kd=1.1)[0]
        }
        
        plt.figure(figsize=(9, 5), dpi=300)
        plt.axhline(step_target, color='magenta', linestyle='--', linewidth=1.8, label=r'Reference Step Command $\theta_{ref} = 10^\circ$')
        
        colors = ['#f59e0b', '#0284c7', '#22c55e']
        for (name, cl_tf), color in zip(tfs.items(), colors):
            _, y = signal.step(cl_tf, T=t)
            y_deg = y * step_target
            plt.plot(t, y_deg, label=name, color=color, linewidth=2.0)
            
        plt.title('Aircraft Pitch Attitude Step Response Comparison (Boeing 747 Cruise)', fontweight='bold')
        plt.xlabel('Time (seconds)')
        plt.ylabel(r'Pitch Attitude Angle $\theta$ (degrees)')
        plt.legend(loc='lower right', frameon=True)
        plt.grid(True, alpha=0.35)
        plt.tight_layout()
        step_path = os.path.join(output_dir, "step_response.png")
        plt.savefig(step_path)
        plt.close()
        print(f"Generated: {step_path}")

        # 2. Bode Frequency Response Plot
        _, L_tf = self.design_pid_closed_loop(Kp=2.4, Ki=0.85, Kd=1.1)
        w, mag, phase = signal.bode(L_tf, w=np.logspace(-2, 2, 500))
        
        fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(9, 6), sharex=True, dpi=300)
        
        # Magnitude
        ax_mag.semilogx(w, mag, color='#0284c7', linewidth=2.0)
        ax_mag.axhline(0, color='gray', linestyle=':', linewidth=1.2)
        ax_mag.set_ylabel('Magnitude (dB)')
        ax_mag.set_title(r'Open-Loop Bode Diagram $L(j\omega) = C(j\omega) \cdot G(j\omega)$', fontweight='bold')
        ax_mag.grid(True, which='both', alpha=0.35)
        
        # Phase
        ax_phase.semilogx(w, phase, color='#f59e0b', linewidth=2.0)
        ax_phase.axhline(-180, color='red', linestyle='--', linewidth=1.2, label=r'$-180^\circ$ Phase Crossover')
        ax_phase.set_ylabel('Phase (deg)')
        ax_phase.set_xlabel('Frequency (rad/s)')
        ax_phase.legend(loc='lower left')
        ax_phase.grid(True, which='both', alpha=0.35)
        
        plt.tight_layout()
        bode_path = os.path.join(output_dir, "bode_plot.png")
        plt.savefig(bode_path)
        plt.close()
        print(f"Generated: {bode_path}")

        # 3. Root Locus Diagram
        plt.figure(figsize=(8, 6), dpi=300)
        
        # Plot open-loop poles and zeros
        ol_poles = self.plant_sp.poles
        ol_zeros = self.plant_sp.zeros
        
        plt.scatter(np.real(ol_poles), np.imag(ol_poles), marker='x', color='red', s=90, linewidth=2.5, label='Open-Loop Poles')
        if len(ol_zeros) > 0:
            plt.scatter(np.real(ol_zeros), np.imag(ol_zeros), marker='o', facecolors='none', edgecolors='blue', s=90, linewidth=2, label='Open-Loop Zeros')
            
        # Draw damping lines
        for zeta in [0.707, 0.5, 0.3]:
            angle = np.arccos(zeta)
            r = 6.0
            plt.plot([0, -r*np.cos(angle)], [0, r*np.sin(angle)], 'k--', alpha=0.25)
            plt.plot([0, -r*np.cos(angle)], [0, -r*np.sin(angle)], 'k--', alpha=0.25)
            plt.text(-r*0.8*np.cos(angle), r*0.8*np.sin(angle), f'$\\zeta={zeta}$', fontsize=8, color='gray')
            
        plt.axvline(0, color='black', linewidth=1.5)
        plt.axhline(0, color='black', linewidth=0.8)
        plt.title('S-Plane Pole-Zero Map & Stability Regions', fontweight='bold')
        plt.xlabel(r'Real Axis $\sigma$ (1/s)')
        plt.ylabel(r'Imaginary Axis $j\omega$ (rad/s)')
        plt.xlim(-5, 2)
        plt.ylim(-4, 4)
        plt.legend(loc='upper left')
        plt.grid(True, alpha=0.35)
        plt.tight_layout()
        rl_path = os.path.join(output_dir, "root_locus.png")
        plt.savefig(rl_path)
        plt.close()
        print(f"Generated: {rl_path}")

        # 4. Anti-Windup Non-Linear Simulation Comparison
        self.generate_antiwindup_plot(output_dir)

    def generate_antiwindup_plot(self, output_dir):
        """Simulate time-domain difference between with vs without anti-windup clamping"""
        dt = 0.01
        t = np.arange(0, 12, dt)
        step_target = 15.0 * np.pi / 180.0 # large 15 deg step causing saturation
        max_de = 25.0 * np.pi / 180.0      # 25 deg saturation limit
        
        # Standard PID without anti-windup
        theta_no_aw = np.zeros_like(t)
        u_no_aw = np.zeros_like(t)
        int_no_aw = 0.0
        x_state_no_aw = np.zeros(4)
        
        # PID WITH anti-windup
        theta_aw = np.zeros_like(t)
        u_aw = np.zeros_like(t)
        int_aw = 0.0
        x_state_aw = np.zeros(4)
        
        Kp = 2.4
        Ki = 0.85
        Kd = 1.1
        
        for i in range(1, len(t)):
            # 1. No Anti-Windup
            e1 = step_target - x_state_no_aw[3]
            int_no_aw += e1 * dt
            d_term1 = -Kd * x_state_no_aw[2]
            raw_u1 = -(Kp * e1 + Ki * int_no_aw + d_term1)
            sat_u1 = np.clip(raw_u1, -max_de, max_de)
            u_no_aw[i] = sat_u1
            
            dx1 = self.A @ x_state_no_aw + self.B.flatten() * sat_u1
            x_state_no_aw += dx1 * dt
            theta_no_aw[i] = x_state_no_aw[3]
            
            # 2. With Anti-Windup (Back-Calculation)
            e2 = step_target - x_state_aw[3]
            d_term2 = -Kd * x_state_aw[2]
            raw_u2 = -(Kp * e2 + Ki * int_aw + d_term2)
            sat_u2 = np.clip(raw_u2, -max_de, max_de)
            u_aw[i] = sat_u2
            
            # Anti-windup correction
            tracking_corr = 2.0 * (sat_u2 - raw_u2)
            int_aw += (e2 + tracking_corr) * dt
            
            dx2 = self.A @ x_state_aw + self.B.flatten() * sat_u2
            x_state_aw += dx2 * dt
            theta_aw[i] = x_state_aw[3]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True, dpi=300)
        
        # Pitch Attitude
        ax1.axhline(step_target * 180 / np.pi, color='magenta', linestyle='--', label=r'Reference Step $\theta_{ref} = 15^\circ$')
        ax1.plot(t, theta_no_aw * 180 / np.pi, color='red', linewidth=2.0, label='Without Anti-Windup (Severe Overshoot & Lag)')
        ax1.plot(t, theta_aw * 180 / np.pi, color='#22c55e', linewidth=2.0, label='With Anti-Windup Back-Calculation (Clean & Damped)')
        ax1.set_ylabel(r'Pitch $\theta$ (deg)')
        ax1.set_title('Anti-Windup Demonstration under Actuator Saturation', fontweight='bold')
        ax1.legend(loc='lower right')
        ax1.grid(True, alpha=0.35)
        
        # Elevator Deflection
        ax2.axhline(max_de * 180 / np.pi, color='gray', linestyle=':', label=r'Saturation Limit $\pm 25^\circ$')
        ax2.axhline(-max_de * 180 / np.pi, color='gray', linestyle=':')
        ax2.plot(t, u_no_aw * 180 / np.pi, color='red', linewidth=1.8, label=r'Elevator $\delta_e$ (No Anti-Windup)')
        ax2.plot(t, u_aw * 180 / np.pi, color='#22c55e', linewidth=1.8, label=r'Elevator $\delta_e$ (With Anti-Windup)')
        ax2.set_ylabel(r'Elevator $\delta_e$ (deg)')
        ax2.set_xlabel('Time (seconds)')
        ax2.legend(loc='lower right')
        ax2.grid(True, alpha=0.35)
        
        plt.tight_layout()
        aw_path = os.path.join(output_dir, "anti_windup_comparison.png")
        plt.savefig(aw_path)
        plt.close()
        print(f"Generated: {aw_path}")


def run_full_report():
    print("=" * 70)
    print("AEROSIM-CONTROL: COMPREHENSIVE CONTROL ENGINEERING ANALYSIS")
    print("=" * 70)
    
    analysis = AircraftControlAnalysis()
    
    print("\n1. LONGITUDINAL STATE-SPACE A MATRIX (4x4):")
    print(np.array2string(analysis.A, precision=4, suppress_small=True))
    
    print("\n2. INPUT B MATRIX (4x1):")
    print(np.array2string(analysis.B, precision=4, suppress_small=True))
    
    print("\n3. OPEN-LOOP MODAL ANALYSIS (EIGENVALUES):")
    modes = analysis.analyze_modes()
    for idx, m in enumerate(modes, 1):
        pole = m['pole']
        wn = m['wn']
        zeta = m['zeta']
        sign = "+" if pole.imag >= 0 else "-"
        print(f"   Mode {idx}: s = {pole.real:8.4f} {sign} {abs(pole.imag):8.4f}j | wn = {wn:6.4f} rad/s | zeta = {zeta:6.4f}")
        
    print("\n4. SHORT-PERIOD TRANSFER FUNCTION G(s) = theta(s) / delta_e(s):")
    print(f"   Numerator:   {np.array2string(analysis.num_sp, precision=3)}")
    print(f"   Denominator: {np.array2string(analysis.den_sp, precision=3)}")
    
    print("\n5. ROUTH-HURWITZ STABILITY VERIFICATION:")
    cl_tf, _ = analysis.design_pid_closed_loop(Kp=2.4, Ki=0.85, Kd=1.1)
    table, sign_changes, is_stable = analysis.compute_routh_array(cl_tf.den)
    print("   Closed-loop Characteristic Polynomial:")
    print(f"   {np.array2string(cl_tf.den, precision=3)}")
    print(f"   Sign changes in column 1: {sign_changes}")
    print(f"   System Stability: {'STABLE (0 RHP roots)' if is_stable else 'UNSTABLE'}")
    
    print("\n6. GENERATING PUBLICATION-QUALITY FIGURES...")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    analysis.generate_all_plots(script_dir)
    print("\nAll figures generated successfully in:", script_dir)
    print("=" * 70)


if __name__ == '__main__':
    run_full_report()
