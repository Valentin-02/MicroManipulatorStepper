"""
Force Control Example for OpenMicroStage

Demonstrates the pure PI force control pipeline (M70–M73):
  Motor 0 → X,  Motor 1 → Y,  Motor 2 → Z

Pipeline:
  HEX Sensor (fx, fy, fz)  →  3× independent PI  →  Torque Commands  →  Motor Driver
  Encoders are used ONLY for safety soft-limits, NOT for feedback.

Commands used:
  M71  – set PI parameters  (Kp, Ki, output_limit, windup_limit)
  M72  – set safety position limits  (±rad per axis)
  M70  – set target forces & enable force control  /  M70 S0 to disable
  M73  – query full controller state (target, measured, error, torque, safety …)
  M60  – single HEX sensor reading (used for bias measurement)

Usage:
  1. Adjust PORT and target forces below.
  2. python force_control_example.py
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from open_micro_stage_api import OpenMicroStageInterface, SerialInterface


class ForceControlMonitor:
    """Monitors and plots force control data in real-time."""

    def __init__(self, max_samples=500, update_interval_ms=50):
        self.max_samples = max_samples
        self.update_interval_ms = update_interval_ms

        # Data buffers  (all forces stored in sensor-native units, typically mN)
        self.times         = deque(maxlen=max_samples)
        self.fx_measured   = deque(maxlen=max_samples)
        self.fy_measured   = deque(maxlen=max_samples)
        self.fz_measured   = deque(maxlen=max_samples)
        self.fx_target     = deque(maxlen=max_samples)
        self.fy_target     = deque(maxlen=max_samples)
        self.fz_target     = deque(maxlen=max_samples)
        self.fx_error      = deque(maxlen=max_samples)
        self.fy_error      = deque(maxlen=max_samples)
        self.fz_error      = deque(maxlen=max_samples)
        self.torque_x      = deque(maxlen=max_samples)
        self.torque_y      = deque(maxlen=max_samples)
        self.torque_z      = deque(maxlen=max_samples)
        self.motor_pos_x   = deque(maxlen=max_samples)
        self.motor_pos_y   = deque(maxlen=max_samples)
        self.motor_pos_z   = deque(maxlen=max_samples)

        self.stage = None
        self.start_time = None
        self.current_target_mN = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0}
        self.force_bias_mN     = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0}

    # --- helpers ---------------------------------------------------------------

    @staticmethod
    def _symmetric_ylim(ax, values, min_half_range=0.5):
        """Centre y-axis around zero with comfortable margin."""
        arr = np.asarray(values, dtype=float)
        abs_max = max(float(np.max(np.abs(arr))) if len(arr) else 0.0, min_half_range)
        margin = max(0.05 * abs_max, 0.05)
        ax.set_ylim(-(abs_max + margin), abs_max + margin)
        ax.axhline(0.0, color='k', ls=':', lw=0.8, alpha=0.5)

    # --- connection & init -----------------------------------------------------

    def connect(self, port, baud_rate=921600):
        print(f"Connecting to {port}...")
        self.stage = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
        self.stage.connect(port, baud_rate)
        self.start_time = time.time()
        return self.stage.serial is not None

    def initialize(self):
        """Enable motors and tare HEX sensor."""
        print("\n=== Initialization ===")

        print("Enabling motors...")
        status = self.stage.enable_motors(True)
        if status != SerialInterface.ReplyStatus.OK:
            print(f"  Error enabling motors: {status}")
            return False
        time.sleep(0.5)

        print("Taring HEX force/torque sensor...")
        status = self.stage.tare_hex_sensor()
        if status != SerialInterface.ReplyStatus.OK:
            print(f"  Error taring sensor: {status}")
            return False
        time.sleep(0.5)

        self._measure_force_bias(samples=20, interval_s=0.05)
        print("✓ Initialization complete\n")
        return True

    def _measure_force_bias(self, samples=20, interval_s=0.05):
        """Measure residual force offset after tare via M60."""
        fx, fy, fz = [], [], []
        for _ in range(samples):
            data = self.stage.read_hex_sensor()
            if data is not None:
                fx.append(data['fx'])
                fy.append(data['fy'])
                fz.append(data['fz'])
            time.sleep(interval_s)

        if fx:
            self.force_bias_mN = {
                'fx': float(np.mean(fx)),
                'fy': float(np.mean(fy)),
                'fz': float(np.mean(fz)),
            }
            print(f"  Residual bias: "
                  f"Fx={self.force_bias_mN['fx']:.2f} mN  "
                  f"Fy={self.force_bias_mN['fy']:.2f} mN  "
                  f"Fz={self.force_bias_mN['fz']:.2f} mN")
        else:
            print("  Warning: could not measure force bias")

    # --- force control ---------------------------------------------------------

    def enable_force_control(self, target_fx_mN=0.0, target_fy_mN=0.0, target_fz_mN=0.0,
                             kp=0.001, ki=0.01,
                             output_limit=1.4137, windup_limit=1.4137,
                             safety_limit_rad=1.5,
                             compensate_bias=True):
        """
        Configure and enable the pure-PI force controller (M71 → M72 → M70).

        :param target_fx_mN: Target force X in mN (sensor-native).
        :param target_fy_mN: Target force Y in mN.
        :param target_fz_mN: Target force Z in mN.
        :param kp: Proportional gain of the PI.
        :param ki: Integral gain of the PI.
        :param output_limit: Max torque command (field-angle offset, rad). ~π*0.45
        :param windup_limit: Anti-windup integral saturation (rad).
        :param safety_limit_rad: Symmetric encoder position limit per axis (rad).
        :param compensate_bias: Subtract measured post-tare bias from target.
        """
        print("=== Force Control Setup ===")

        # 1) PI parameters  (M71)
        print(f"  PI params: Kp={kp}  Ki={ki}  out_lim={output_limit:.4f}  wu_lim={windup_limit:.4f}")
        status = self.stage.set_force_pi_parameters(kp, ki, output_limit, windup_limit)
        if status != SerialInterface.ReplyStatus.OK:
            print(f"  Error setting PI params: {status}"); return False

        # 2) Safety limits  (M72)
        print(f"  Safety limits: ±{safety_limit_rad:.3f} rad per axis")
        status = self.stage.set_force_safety_limits(safety_limit_rad, safety_limit_rad, safety_limit_rad)
        if status != SerialInterface.ReplyStatus.OK:
            print(f"  Error setting safety limits: {status}"); return False

        # 3) Compute controller target (optional bias compensation)
        self.current_target_mN = {'fx': target_fx_mN, 'fy': target_fy_mN, 'fz': target_fz_mN}
        if compensate_bias:
            cmd_fx = target_fx_mN + self.force_bias_mN['fx']
            cmd_fy = target_fy_mN + self.force_bias_mN['fy']
            cmd_fz = target_fz_mN + self.force_bias_mN['fz']
            print("  Applying residual-bias compensation")
        else:
            cmd_fx, cmd_fy, cmd_fz = target_fx_mN, target_fy_mN, target_fz_mN

        # 4) Set target forces & activate force mode  (M70)
        print(f"  Targets → Fx={cmd_fx:.2f}  Fy={cmd_fy:.2f}  Fz={cmd_fz:.2f} mN")
        status = self.stage.set_force_target(cmd_fx, cmd_fy, cmd_fz)
        if status != SerialInterface.ReplyStatus.OK:
            print(f"  Error setting force target: {status}"); return False

        print("✓ Force control enabled\n")
        return True

    # --- data collection -------------------------------------------------------

    def collect_data(self, duration_s=30.0):
        """
        Collect force control data via M73 for the specified duration.
        M73 returns targets, measured forces, errors, torque outputs, motor
        positions, and safety flags in a single response.
        """
        print(f"=== Data Collection ({duration_s:.1f} s) ===")
        start = time.time()
        n = 0

        try:
            while time.time() - start < duration_s:
                state = self.stage.read_force_state()
                if state is None:
                    time.sleep(self.update_interval_ms / 1000.0)
                    continue

                elapsed = time.time() - start
                self.times.append(elapsed)

                # Measured forces (sensor-native mN)
                m = state.get('measured', [0, 0, 0])
                self.fx_measured.append(m[0])
                self.fy_measured.append(m[1])
                self.fz_measured.append(m[2])

                # Controller targets
                t = state.get('target', [0, 0, 0])
                self.fx_target.append(t[0])
                self.fy_target.append(t[1])
                self.fz_target.append(t[2])

                # Force errors
                e = state.get('error', [0, 0, 0])
                self.fx_error.append(e[0])
                self.fy_error.append(e[1])
                self.fz_error.append(e[2])

                # Torque outputs (field-angle offset)
                tq = state.get('torque', [0, 0, 0])
                self.torque_x.append(tq[0])
                self.torque_y.append(tq[1])
                self.torque_z.append(tq[2])

                # Motor positions (encoder rad)
                mp = state.get('motor_pos', [0, 0, 0])
                self.motor_pos_x.append(mp[0])
                self.motor_pos_y.append(mp[1])
                self.motor_pos_z.append(mp[2])

                n += 1

                # Progress output
                if n % 10 == 0:
                    safety = state.get('safety', [False, False, False])
                    s_flag = " SAFETY!" if any(safety) else ""
                    print(f"  [{elapsed:6.1f}s]  Fx={m[0]:+9.2f}  Fy={m[1]:+9.2f}  "
                          f"Fz={m[2]:+9.2f} mN  |  τ={tq[0]:+.4f} {tq[1]:+.4f} {tq[2]:+.4f}{s_flag}")

                time.sleep(self.update_interval_ms / 1000.0)

        except KeyboardInterrupt:
            print("\n  Data collection interrupted by user")

        print(f"✓ Collected {n} samples\n")
        return n

    # --- plotting --------------------------------------------------------------

    def plot_data(self):
        """Plot measured vs target forces, errors, and torque outputs."""
        if not self.times:
            print("No data to plot"); return

        t  = list(self.times)
        fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
        fig.suptitle('Pure PI Force Control  —  3-Axis Overview', fontsize=15, fontweight='bold')

        labels = ['X', 'Y', 'Z']
        colors_meas   = ['#1f77b4', '#2ca02c', '#ff7f0e']
        colors_target = ['#d62728', '#d62728', '#d62728']

        # Row 1 – measured forces
        ax = axes[0]
        for i, (meas, targ) in enumerate(zip(
            [self.fx_measured, self.fy_measured, self.fz_measured],
            [self.fx_target,   self.fy_target,   self.fz_target])):
            meas_arr = list(meas)
            ax.plot(t, meas_arr, color=colors_meas[i], lw=1.2, label=f'F{labels[i]} meas')
            tval = list(targ)[0] if targ else 0
            ax.axhline(tval, color=colors_target[i], ls='--', lw=1, alpha=0.6,
                       label=f'F{labels[i]} target ({tval:.1f})')
        ax.set_ylabel('Force [mN]')
        ax.set_title('Measured vs Target Forces')
        ax.legend(loc='upper right', fontsize=8, ncol=3)
        ax.grid(True, alpha=0.3)

        # Row 2 – force errors
        ax = axes[1]
        for i, err in enumerate([self.fx_error, self.fy_error, self.fz_error]):
            ax.plot(t, list(err), color=colors_meas[i], lw=1.0, label=f'e_{labels[i]}')
        self._symmetric_ylim(ax, list(self.fx_error) + list(self.fy_error) + list(self.fz_error))
        ax.set_ylabel('Error [mN]')
        ax.set_title('Force Error  (target − measured)')
        ax.legend(loc='upper right', fontsize=8, ncol=3)
        ax.grid(True, alpha=0.3)

        # Row 3 – torque outputs
        ax = axes[2]
        for i, tq in enumerate([self.torque_x, self.torque_y, self.torque_z]):
            ax.plot(t, list(tq), color=colors_meas[i], lw=1.0, label=f'τ_{labels[i]}')
        self._symmetric_ylim(ax, list(self.torque_x) + list(self.torque_y) + list(self.torque_z),
                             min_half_range=0.1)
        ax.set_ylabel('Torque Cmd [rad]')
        ax.set_title('PI Output  (field-angle offset)')
        ax.legend(loc='upper right', fontsize=8, ncol=3)
        ax.grid(True, alpha=0.3)

        # Row 4 – motor positions (safety context)
        ax = axes[3]
        for i, mp in enumerate([self.motor_pos_x, self.motor_pos_y, self.motor_pos_z]):
            ax.plot(t, list(mp), color=colors_meas[i], lw=1.0, label=f'θ_{labels[i]}')
        ax.set_ylabel('Motor Pos [rad]')
        ax.set_xlabel('Time [s]')
        ax.set_title('Encoder Positions  (safety monitoring)')
        ax.legend(loc='upper right', fontsize=8, ncol=3)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def plot_3d_trajectory(self):
        """Plot the 3D force vector trajectory."""
        if len(self.fx_measured) < 2:
            print("Not enough data for 3D plot"); return

        fx = list(self.fx_measured)
        fy = list(self.fy_measured)
        fz = list(self.fz_measured)

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(fx, fy, fz, 'b-', lw=0.8, alpha=0.6, label='Trajectory')
        ax.scatter([fx[0]],  [fy[0]],  [fz[0]],  color='green',  s=80, label='Start')
        ax.scatter([fx[-1]], [fy[-1]], [fz[-1]], color='red',    s=80, label='End')
        ax.scatter([self.current_target_mN['fx']],
                   [self.current_target_mN['fy']],
                   [self.current_target_mN['fz']],
                   color='orange', s=150, marker='*', edgecolors='k', lw=1.5, label='Target')
        ax.set_xlabel('Fx [mN]'); ax.set_ylabel('Fy [mN]'); ax.set_zlabel('Fz [mN]')
        ax.set_title('3D Force Vector Trajectory', fontweight='bold')
        ax.legend()
        ax.grid(True)
        plt.tight_layout()
        plt.show()

    # --- statistics ------------------------------------------------------------

    def get_statistics(self):
        if not self.fz_measured:
            print("No data"); return

        fx = np.asarray(self.fx_measured)
        fy = np.asarray(self.fy_measured)
        fz = np.asarray(self.fz_measured)

        print("=== Force Data Statistics (mN) ===")
        for name, arr in [('Fx', fx), ('Fy', fy), ('Fz', fz)]:
            print(f"\n  {name}:  mean={np.mean(arr):+.2f}  std={np.std(arr):.2f}"
                  f"  min={np.min(arr):+.2f}  max={np.max(arr):+.2f}")

        mag = np.sqrt(fx**2 + fy**2 + fz**2)
        print(f"\n  |F|: mean={np.mean(mag):.2f}  std={np.std(mag):.2f}"
              f"  min={np.min(mag):.2f}  max={np.max(mag):.2f}")

        tgt = self.current_target_mN
        tgt_mag = np.sqrt(tgt['fx']**2 + tgt['fy']**2 + tgt['fz']**2)
        err_mag = mag - tgt_mag
        print(f"\n  Tracking error vs |target|={tgt_mag:.2f} mN:"
              f"  mean={np.mean(err_mag):+.2f}  std={np.std(err_mag):.2f}\n")

    # --- cleanup ---------------------------------------------------------------

    def disconnect(self):
        if self.stage is not None:
            print("Disabling force control...")
            self.stage.disable_force_control()
            print("Disabling motors...")
            self.stage.enable_motors(False)
            print("Disconnecting...")
            self.stage.disconnect()
            self.stage = None


# ═══════════════════════════════════════════════════════════════════════════════

def main():
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║       OpenMicroStage  –  Pure PI Force Control Example      ║")
    print("╚══════════════════════════════════════════════════════════════╝\n")

    # ---- Configuration --------------------------------------------------------
    PORT            = 'COM8'    # <-- change to your serial port
    COLLECTION_S    = 30.0      # data-collection duration [s]

    # Target forces in mN  (sensor-native units)
    TARGET_FX_MN    = 0.0
    TARGET_FY_MN    = 10.0
    TARGET_FZ_MN    = 0.0

    # PI tuning  (start conservative, increase Ki for tighter tracking)
    KP              = 0.001     # proportional gain
    KI              = 0.01      # integral gain
    OUTPUT_LIMIT    = 1.4137    # ≈ π×0.45  max torque command [rad]
    WINDUP_LIMIT    = 1.4137    # ≈ π×0.45  anti-windup [rad]
    SAFETY_LIMIT    = 1.5       # encoder soft-limit [rad]
    # ---------------------------------------------------------------------------

    monitor = ForceControlMonitor(max_samples=2000, update_interval_ms=50)

    try:
        if not monitor.connect(PORT):
            print("Failed to connect"); return

        if not monitor.initialize():
            print("Failed to initialise"); return

        if not monitor.enable_force_control(
            target_fx_mN   = TARGET_FX_MN,
            target_fy_mN   = TARGET_FY_MN,
            target_fz_mN   = TARGET_FZ_MN,
            kp             = KP,
            ki             = KI,
            output_limit   = OUTPUT_LIMIT,
            windup_limit   = WINDUP_LIMIT,
            safety_limit_rad = SAFETY_LIMIT,
        ):
            print("Failed to enable force control"); return

        monitor.collect_data(duration_s=COLLECTION_S)
        monitor.get_statistics()

        print("Disabling force control...")
        monitor.stage.disable_force_control()
        print("✓ Force control disabled\n")

        print("Generating plots...")
        monitor.plot_data()
        monitor.plot_3d_trajectory()

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()

    finally:
        monitor.disconnect()
        print("\n✓ Example completed")


if __name__ == '__main__':
    main()
