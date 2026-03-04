"""
Force Control Example for OpenMicroStage

This script demonstrates how to use the force control feature of the OpenMicroStage API.
It connects to the device, enables force control, sets a target force, and plots the
measured force data in real-time.

Features:
- Connects to the robot
- Enables motors and homes all axes
- Tares the HEX force/torque sensor
- Enables force control mode
- Sets target force
- Reads and plots force sensor data
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from open_micro_stage_api import OpenMicroStageInterface, SerialInterface


class ForceControlMonitor:
    """Monitors and plots force control data in real-time"""
    
    def __init__(self, max_samples=500, update_interval_ms=100):
        """
        Initialize the force control monitor
        :param max_samples: Maximum number of samples to keep in buffer
        :param update_interval_ms: Update interval for data collection [ms]
        """
        self.max_samples = max_samples
        self.update_interval_ms = update_interval_ms
        
        # Data buffers
        self.times = deque(maxlen=max_samples)
        self.fx_measured = deque(maxlen=max_samples)
        self.fy_measured = deque(maxlen=max_samples)
        self.fz_measured = deque(maxlen=max_samples)
        self.fx_target = deque(maxlen=max_samples)
        self.fy_target = deque(maxlen=max_samples)
        self.fz_target = deque(maxlen=max_samples)
        
        self.stage = None
        self.start_time = None
        self.current_target = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0}  # user target [N]
        self.controller_target = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0}  # command sent to FW [N]
        self.force_bias = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0}  # post-tare residual bias [N]

    @staticmethod
    def _apply_symmetric_force_axis(ax, measured_values_n, target_values_n=None,
                                    min_half_range_n=0.5, include_target_in_scale=False):
        """Force plot helper: keep 0 centered and show measured dynamics clearly."""
        measured = np.array(measured_values_n, dtype=float)
        candidates = [float(np.max(np.abs(measured))), min_half_range_n]
        if include_target_in_scale and target_values_n is not None and len(target_values_n) > 0:
            targets = np.array(target_values_n, dtype=float)
            candidates.append(float(np.max(np.abs(targets))))

        abs_max = max(candidates)
        y_margin = max(0.05 * abs_max, 0.05)
        ax.set_ylim(-(abs_max + y_margin), abs_max + y_margin)
        ax.axhline(y=0.0, color='k', linestyle=':', linewidth=1.0, alpha=0.7)
        
    def connect(self, port, baud_rate=921600):
        """Connect to the device"""
        print(f"Connecting to {port}...")
        self.stage = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
        self.stage.connect(port, baud_rate)
        self.start_time = time.time()
        return self.stage.serial is not None
        
    def initialize(self):
        """Initialize the robot: enable motors, home, and tare sensor"""
        print("\n=== Initialization ===")
        
        # Enable motors
        print("Enabling motors...")
        status = self.stage.enable_motors(True)
        if status != SerialInterface.ReplyStatus.OK:
            print(f"Error enabling motors: {status}")
            return False
        time.sleep(0.5)
        
        # Tare HEX sensor
        print("Taring HEX force/torque sensor...")
        status = self.stage.tare_hex_sensor(timeout_s=30.0)
        if status != SerialInterface.ReplyStatus.OK:
            print(f"Error taring sensor: {status}")
            return False
        time.sleep(0.5)

        # Measure residual bias after tare (for diagnostics / optional compensation)
        self.measure_force_bias(samples=20, sample_interval_s=0.05)
        
        print("✓ Initialization complete\n")
        return True
    
    def measure_force_bias(self, samples=20, sample_interval_s=0.05):
        """Measure residual force bias after tare by averaging HEX readings."""
        fx_vals = []
        fy_vals = []
        fz_vals = []

        for _ in range(samples):
            status, sensor_data = self.stage.read_hex_sensor()
            if status == SerialInterface.ReplyStatus.OK and sensor_data:
                # API returns forces in mN
                fx_vals.append(sensor_data['fx'] / 1000.0)
                fy_vals.append(sensor_data['fy'] / 1000.0)
                fz_vals.append(sensor_data['fz'] / 1000.0)
            time.sleep(sample_interval_s)

        if len(fx_vals) > 0:
            self.force_bias = {
                'fx': float(np.mean(fx_vals)),
                'fy': float(np.mean(fy_vals)),
                'fz': float(np.mean(fz_vals))
            }
            print("Residual bias after tare:")
            print(f"  Fx={self.force_bias['fx']:.4f} N, Fy={self.force_bias['fy']:.4f} N, Fz={self.force_bias['fz']:.4f} N")
        else:
            print("Warning: Could not measure post-tare force bias")

    def enable_force_control(self, target_fx=0.0, target_fy=0.0, target_fz=0.0, compensate_bias=True):
        """
        Enable force control with target force
        :param target_fx: Target force X [N]
        :param target_fy: Target force Y [N]
        :param target_fz: Target force Z [N]
        :param compensate_bias: If True, adds measured post-tare bias to controller target
        """
        print("=== Force Control Setup ===")
        
        # Set force controller parameters
        print("Setting force controller parameters...")
        status = self.stage.set_force_parameters(
            kp=1.0,               # Proportional gain [mm/N]
            ki=5.0,               # Integral gain [mm/(N·s)]
            output_limit=1.0,     # Max position correction [mm]
            windup_limit=0.5,     # Integral windup limit [mm]
            filter_tc=0.005,      # Force filter time constant [s]
            max_displacement=2.0  # Max displacement from base pose [mm]
        )
        if status != SerialInterface.ReplyStatus.OK:
            print(f"Error setting parameters: {status}")
            return False

        # Desired force target in sensor frame [N]
        self.current_target = {'fx': target_fx, 'fy': target_fy, 'fz': target_fz}

        # Controller target (optional residual-bias compensation)
        if compensate_bias:
            self.controller_target = {
                'fx': target_fx + self.force_bias['fx'],
                'fy': target_fy + self.force_bias['fy'],
                'fz': target_fz + self.force_bias['fz']
            }
            print("Applying residual-bias compensation to target")
        else:
            self.controller_target = dict(self.current_target)
        
        # Set target force
        print(
            f"Setting controller target force: "
            f"Fx={self.controller_target['fx']:.4f} N, "
            f"Fy={self.controller_target['fy']:.4f} N, "
            f"Fz={self.controller_target['fz']:.4f} N"
        )
        status = self.stage.set_force_target(
            self.controller_target['fx'],
            self.controller_target['fy'],
            self.controller_target['fz']
        )
        if status != SerialInterface.ReplyStatus.OK:
            print(f"Error setting target force: {status}")
            return False
        
        # Enable force control
        print("Enabling force control mode...")
        status = self.stage.enable_force_control(enable=True)
        if status != SerialInterface.ReplyStatus.OK:
            print(f"Error enabling force control: {status}")
            return False
        
        print("✓ Force control enabled\n")
        return True
    
    def collect_data(self, duration_s=30.0):
        """
        Collect force data for specified duration
        :param duration_s: Duration to collect data [s]
        """
        print(f"=== Data Collection ({duration_s:.1f}s) ===")
        print("Collecting force measurements...")
        
        start_time = time.time()
        sample_count = 0
        
        try:
            while time.time() - start_time < duration_s:
                # Read sensor data
                status, sensor_data = self.stage.read_hex_sensor()
                
                if status == SerialInterface.ReplyStatus.OK and sensor_data:
                    elapsed = time.time() - start_time
                    
                    # Store measurements
                    self.times.append(elapsed)
                    self.fx_measured.append(sensor_data['fx'])
                    self.fy_measured.append(sensor_data['fy'])
                    self.fz_measured.append(sensor_data['fz'])
                    
                    # Store target (constant during this collection)
                    self.fx_target.append(self.current_target['fx'])
                    self.fy_target.append(self.current_target['fy'])
                    self.fz_target.append(self.current_target['fz'])
                    
                    sample_count += 1
                    
                    # Print progress (sensor data is in mN from API conversion)
                    if sample_count % 10 == 0:
                        fx_n = sensor_data['fx'] / 1000.0
                        fy_n = sensor_data['fy'] / 1000.0
                        fz_n = sensor_data['fz'] / 1000.0
                        print(f"  [{elapsed:.1f}s] Fx: {fx_n:+8.4f} N | "
                            f"Fy: {fy_n:+8.4f} N | "
                            f"Fz: {fz_n:+8.4f} N")
                
                # Wait before next sample
                time.sleep(self.update_interval_ms / 1000.0)
        
        except KeyboardInterrupt:
            print("\nData collection interrupted by user")
        
        print(f"✓ Collected {sample_count} samples\n")
        return sample_count
    
    def plot_data(self):
        """Plot the collected force data"""
        if len(self.times) == 0:
            print("No data to plot")
            return
        
        # Convert deques to lists
        times = list(self.times)
        fx_measured = list(self.fx_measured)
        fy_measured = list(self.fy_measured)
        fz_measured = list(self.fz_measured)
        fx_target = list(self.fx_target)
        fy_target = list(self.fy_target)
        fz_target = list(self.fz_target)
        
        # Create figure with subplots
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 9))
        fig.suptitle('Force Control Data', fontsize=16, fontweight='bold')
        
        # Convert measured values to Newton for display (sensor data from API is in mN)
        fx_meas_n = [x / 1000.0 for x in fx_measured]
        fy_meas_n = [x / 1000.0 for x in fy_measured]
        fz_meas_n = [x / 1000.0 for x in fz_measured]
        # Target values are already in Newton
        fx_targ_n = fx_target
        fy_targ_n = fy_target
        fz_targ_n = fz_target
        
        # Plot X-axis force
        ax1.plot(times, fx_meas_n, 'b-', label='Measured', linewidth=1.5)
        ax1.axhline(y=fx_targ_n[0] if fx_targ_n else 0, color='r', linestyle='--', 
                    label=f'Target ({fx_targ_n[0] if fx_targ_n else 0:.4f} N)', linewidth=2)
        ax1.set_ylabel('Force X [N]', fontsize=11)
        ax1.set_title('X-Axis Force Control')
        self._apply_symmetric_force_axis(ax1, fx_meas_n, fx_targ_n, include_target_in_scale=False)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best')
        
        # Plot Y-axis force
        ax2.plot(times, fy_meas_n, 'g-', label='Measured', linewidth=1.5)
        ax2.axhline(y=fy_targ_n[0] if fy_targ_n else 0, color='r', linestyle='--', 
                    label=f'Target ({fy_targ_n[0] if fy_targ_n else 0:.4f} N)', linewidth=2)
        ax2.set_ylabel('Force Y [N]', fontsize=11)
        ax2.set_title('Y-Axis Force Control')
        self._apply_symmetric_force_axis(ax2, fy_meas_n, fy_targ_n, include_target_in_scale=False)
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='best')
        
        # Plot Z-axis force
        ax3.plot(times, fz_meas_n, 'orange', label='Measured', linewidth=1.5)
        ax3.axhline(y=fz_targ_n[0] if fz_targ_n else 0, color='r', linestyle='--', 
                    label=f'Target ({fz_targ_n[0] if fz_targ_n else 0:.4f} N)', linewidth=2)
        ax3.set_xlabel('Time [s]', fontsize=11)
        ax3.set_ylabel('Force Z [N]', fontsize=11)
        ax3.set_title('Z-Axis Force Control')
        self._apply_symmetric_force_axis(ax3, fz_meas_n, fz_targ_n, include_target_in_scale=False)
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc='best')
        
        plt.tight_layout()
        plt.show()
    
    def plot_3d_trajectory(self):
        """Plot the 3D force vector trajectory"""
        if len(self.fx_measured) < 2:
            print("Not enough data for 3D plot")
            return
        
        fx = [x / 1000.0 for x in self.fx_measured]
        fy = [x / 1000.0 for x in self.fy_measured]
        fz = [x / 1000.0 for x in self.fz_measured]
        
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectory
        ax.plot(fx, fy, fz, 'b-', linewidth=1, alpha=0.6, label='Force trajectory')
        
        # Plot start point
        ax.scatter([fx[0]], [fy[0]], [fz[0]], color='green', s=100, label='Start')
        
        # Plot end point
        ax.scatter([fx[-1]], [fy[-1]], [fz[-1]], color='red', s=100, label='End')
        
        # Plot target point
        ax.scatter([self.current_target['fx']], [self.current_target['fy']], 
                  [self.current_target['fz']], color='orange', s=150, marker='*', 
                  label='Target', edgecolors='black', linewidth=2)
        
        ax.set_xlabel('Force X [N]', fontsize=11)
        ax.set_ylabel('Force Y [N]', fontsize=11)
        ax.set_zlabel('Force Z [N]', fontsize=11)
        ax.set_title('3D Force Vector Trajectory', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def get_statistics(self):
        """Calculate and print force data statistics"""
        if len(self.fz_measured) == 0:
            print("No data available for statistics")
            return
        
        # Convert from mN (API output) to N
        fx = np.array(list(self.fx_measured)) / 1000.0
        fy = np.array(list(self.fy_measured)) / 1000.0
        fz = np.array(list(self.fz_measured)) / 1000.0
        
        print("=== Force Data Statistics ===")
        print(f"\nX-Axis Force:")
        print(f"  Mean:   {np.mean(fx):8.4f} N")
        print(f"  Std:    {np.std(fx):8.4f} N")
        print(f"  Min:    {np.min(fx):8.4f} N")
        print(f"  Max:    {np.max(fx):8.4f} N")
        
        print(f"\nY-Axis Force:")
        print(f"  Mean:   {np.mean(fy):8.4f} N")
        print(f"  Std:    {np.std(fy):8.4f} N")
        print(f"  Min:    {np.min(fy):8.4f} N")
        print(f"  Max:    {np.max(fy):8.4f} N")
        
        print(f"\nZ-Axis Force:")
        print(f"  Mean:   {np.mean(fz):8.4f} N")
        print(f"  Std:    {np.std(fz):8.4f} N")
        print(f"  Min:    {np.min(fz):8.4f} N")
        print(f"  Max:    {np.max(fz):8.4f} N")
        
        # Calculate magnitude
        magnitude = np.sqrt(fx**2 + fy**2 + fz**2)
        print(f"\nForce Magnitude:")
        print(f"  Mean:   {np.mean(magnitude):8.4f} N")
        print(f"  Std:    {np.std(magnitude):8.4f} N")
        print(f"  Min:    {np.min(magnitude):8.4f} N")
        print(f"  Max:    {np.max(magnitude):8.4f} N")
        
        # Calculate error from target
        target_magnitude = np.sqrt(
            self.current_target['fx']**2 + 
            self.current_target['fy']**2 + 
            self.current_target['fz']**2
        )
        error = magnitude - target_magnitude
        
        print(f"\nTracking Error (vs target {target_magnitude:.4f} N):")
        print(f"  Mean Error: {np.mean(error):8.4f} N")
        print(f"  Std Error:  {np.std(error):8.4f} N")
        print()
    
    def disconnect(self):
        """Disconnect from the device"""
        if self.stage is not None:
            print("Disabling force control...")
            self.stage.enable_force_control(enable=False)
            
            print("Disabling motors...")
            self.stage.enable_motors(False)
            
            print("Disconnecting...")
            self.stage.disconnect()
            self.stage = None


def main():
    """Main execution"""
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║          OpenMicroStage Force Control Example                ║")
    print("╚══════════════════════════════════════════════════════════════╝\n")
    
    # Configuration
    PORT = 'COM8'           # Serial port (change to your port)
    COLLECTION_TIME = 30.0  # Data collection time [s]
    TARGET_FZ = 0         # Target force Z-axis [N]
    TARGET_FX = 0        # Target force X-axis [N]
    TARGET_FY = 10       # Target force Y-axis [N]
    
    monitor = ForceControlMonitor(max_samples=1000, update_interval_ms=50)
    
    try:
        # Connect to device
        if not monitor.connect(PORT):
            print("Failed to connect to device")
            return
        
        # Initialize robot
        if not monitor.initialize():
            print("Failed to initialize robot")
            return
        
        # Enable force control
        if not monitor.enable_force_control(
            target_fx=TARGET_FX,
            target_fy=TARGET_FY,
            target_fz=TARGET_FZ
        ):
            print("Failed to enable force control")
            return
        
        # Collect data
        monitor.collect_data(duration_s=COLLECTION_TIME)
        
        # Display statistics
        monitor.get_statistics()
        
        # Disable force control
        print("Disabling force control...")
        status = monitor.stage.enable_force_control(enable=False)
        if status == SerialInterface.ReplyStatus.OK:
            print("✓ Force control disabled\n")
        
        # Plot results
        print("Generating plots...")
        monitor.plot_data()
        monitor.plot_3d_trajectory()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        monitor.disconnect()
        print("\n✓ Example completed")


if __name__ == '__main__':
    main()
