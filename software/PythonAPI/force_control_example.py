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
        self.current_target = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0}
        
    def connect(self, port, baud_rate=921600):
        """Connect to the device"""
        print(f"Connecting to {port}...")
        self.stage = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
        self.stage.connect(port, baud_rate)
        self.start_time = time.time()
        return self.stage is not None
        
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
        
        # Home all axes
        print("Homing all axes...")
        status = self.stage.home()
        if status != SerialInterface.ReplyStatus.OK:
            print(f"Error homing: {status}")
            return False
        time.sleep(0.5)
        
        # Tare HEX sensor
        print("Taring HEX force/torque sensor...")
        status = self.stage.tare_hex_sensor()
        if status != SerialInterface.ReplyStatus.OK:
            print(f"Error taring sensor: {status}")
            return False
        time.sleep(0.5)
        
        print("✓ Initialization complete\n")
        return True
    
    def enable_force_control(self, target_fx=0.0, target_fy=0.0, target_fz=0.0):
        """
        Enable force control with target force
        :param target_fx: Target force X [mN]
        :param target_fy: Target force Y [mN]
        :param target_fz: Target force Z [mN]
        """
        print("=== Force Control Setup ===")
        
        # Set force controller parameters
        print("Setting force controller parameters...")
        status = self.stage.set_force_parameters(
            kp=0.01,              # Proportional gain [mm/mN]
            ki=0.002,             # Integral gain [mm/(mN·s)]
            output_limit=2.0,     # Max position correction [mm]
            windup_limit=1.0,     # Integral windup limit [mm]
            filter_tc=0.005,      # Force filter time constant [s]
            max_displacement=5.0  # Max displacement from base pose [mm]
        )
        if status != SerialInterface.ReplyStatus.OK:
            print(f"Error setting parameters: {status}")
            return False
        
        # Set target force
        print(f"Setting target force: Fx={target_fx:.1f} mN, Fy={target_fy:.1f} mN, Fz={target_fz:.1f} mN")
        status = self.stage.set_force_target(target_fx, target_fy, target_fz)
        if status != SerialInterface.ReplyStatus.OK:
            print(f"Error setting target force: {status}")
            return False
        
        # Enable force control
        print("Enabling force control mode...")
        status = self.stage.enable_force_control(enable=True)
        if status != SerialInterface.ReplyStatus.OK:
            print(f"Error enabling force control: {status}")
            return False
        
        self.current_target = {'fx': target_fx, 'fy': target_fy, 'fz': target_fz}
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
                    
                    # Print progress
                    if sample_count % 10 == 0:
                        print(f"  [{elapsed:.1f}s] Fx: {sensor_data['fx']:7.2f} mN | "
                              f"Fy: {sensor_data['fy']:7.2f} mN | "
                              f"Fz: {sensor_data['fz']:7.2f} mN")
                
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
        
        # Plot X-axis force
        ax1.plot(times, fx_measured, 'b-', label='Measured', linewidth=1.5)
        ax1.axhline(y=fx_target[0] if fx_target else 0, color='r', linestyle='--', 
                    label=f'Target ({fx_target[0] if fx_target else 0:.1f} mN)', linewidth=2)
        ax1.set_ylabel('Force X [mN]', fontsize=11)
        ax1.set_title('X-Axis Force Control')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best')
        
        # Plot Y-axis force
        ax2.plot(times, fy_measured, 'g-', label='Measured', linewidth=1.5)
        ax2.axhline(y=fy_target[0] if fy_target else 0, color='r', linestyle='--', 
                    label=f'Target ({fy_target[0] if fy_target else 0:.1f} mN)', linewidth=2)
        ax2.set_ylabel('Force Y [mN]', fontsize=11)
        ax2.set_title('Y-Axis Force Control')
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='best')
        
        # Plot Z-axis force
        ax3.plot(times, fz_measured, 'orange', label='Measured', linewidth=1.5)
        ax3.axhline(y=fz_target[0] if fz_target else 0, color='r', linestyle='--', 
                    label=f'Target ({fz_target[0] if fz_target else 0:.1f} mN)', linewidth=2)
        ax3.set_xlabel('Time [s]', fontsize=11)
        ax3.set_ylabel('Force Z [mN]', fontsize=11)
        ax3.set_title('Z-Axis Force Control')
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc='best')
        
        plt.tight_layout()
        plt.show()
    
    def plot_3d_trajectory(self):
        """Plot the 3D force vector trajectory"""
        if len(self.fx_measured) < 2:
            print("Not enough data for 3D plot")
            return
        
        fx = list(self.fx_measured)
        fy = list(self.fy_measured)
        fz = list(self.fz_measured)
        
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
        
        ax.set_xlabel('Force X [mN]', fontsize=11)
        ax.set_ylabel('Force Y [mN]', fontsize=11)
        ax.set_zlabel('Force Z [mN]', fontsize=11)
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
        
        fx = np.array(list(self.fx_measured))
        fy = np.array(list(self.fy_measured))
        fz = np.array(list(self.fz_measured))
        
        print("=== Force Data Statistics ===")
        print(f"\nX-Axis Force:")
        print(f"  Mean:   {np.mean(fx):8.2f} mN")
        print(f"  Std:    {np.std(fx):8.2f} mN")
        print(f"  Min:    {np.min(fx):8.2f} mN")
        print(f"  Max:    {np.max(fx):8.2f} mN")
        
        print(f"\nY-Axis Force:")
        print(f"  Mean:   {np.mean(fy):8.2f} mN")
        print(f"  Std:    {np.std(fy):8.2f} mN")
        print(f"  Min:    {np.min(fy):8.2f} mN")
        print(f"  Max:    {np.max(fy):8.2f} mN")
        
        print(f"\nZ-Axis Force:")
        print(f"  Mean:   {np.mean(fz):8.2f} mN")
        print(f"  Std:    {np.std(fz):8.2f} mN")
        print(f"  Min:    {np.min(fz):8.2f} mN")
        print(f"  Max:    {np.max(fz):8.2f} mN")
        
        # Calculate magnitude
        magnitude = np.sqrt(fx**2 + fy**2 + fz**2)
        print(f"\nForce Magnitude:")
        print(f"  Mean:   {np.mean(magnitude):8.2f} mN")
        print(f"  Std:    {np.std(magnitude):8.2f} mN")
        print(f"  Min:    {np.min(magnitude):8.2f} mN")
        print(f"  Max:    {np.max(magnitude):8.2f} mN")
        
        # Calculate error from target
        target_magnitude = np.sqrt(
            self.current_target['fx']**2 + 
            self.current_target['fy']**2 + 
            self.current_target['fz']**2
        )
        error = magnitude - target_magnitude
        
        print(f"\nTracking Error (vs target {target_magnitude:.2f} mN):")
        print(f"  Mean Error: {np.mean(error):8.2f} mN")
        print(f"  Std Error:  {np.std(error):8.2f} mN")
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
    TARGET_FZ = 50.0        # Target force Z-axis [mN]
    TARGET_FX = 0.0         # Target force X-axis [mN]
    TARGET_FY = 0.0         # Target force Y-axis [mN]
    
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
