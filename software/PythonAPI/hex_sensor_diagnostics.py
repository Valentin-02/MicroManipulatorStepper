"""
HEX Sensor Diagnostics Script
=====================================================
This script helps diagnose issues with the HEX force/torque sensor readings.
It tests raw sensor values, tare operations, and communication timing.

Usage:
    python hex_sensor_diagnostics.py
"""

import time
import serial
import re
from open_micro_stage_api import OpenMicroStageInterface, SerialInterface


def print_separator(title=""):
    """Print a formatted separator"""
    if title:
        print(f"\n{'='*60}")
        print(f"  {title}")
        print(f"{'='*60}")
    else:
        print(f"{'-'*60}")


def test_raw_sensor_values(stage, num_readings=10):
    """Read raw sensor values without any processing"""
    print_separator("RAW SENSOR VALUES TEST")
    print(f"Reading {num_readings} raw sensor frames...\n")
    
    readings = []
    for i in range(num_readings):
        status, data = stage.read_hex_sensor()
        if status == SerialInterface.ReplyStatus.OK:
            # Convert back from mN to N for display
            fx_n = data['fx'] / 1000.0
            fy_n = data['fy'] / 1000.0
            fz_n = data['fz'] / 1000.0
            
            print(f"  [{i+1:2d}] Fx: {fx_n:8.4f} N | Fy: {fy_n:8.4f} N | Fz: {fz_n:8.4f} N | "
                  f"Temp: {data['temperature']:6.2f}°C")
            readings.append({'fx': fx_n, 'fy': fy_n, 'fz': fz_n})
        else:
            print(f"  [{i+1:2d}] ERROR: {status}")
            return None
        
        time.sleep(0.2)
    
    # Analyze readings
    if readings:
        print_separator()
        print("ANALYSIS:")
        print(f"  Mean Fx: {sum(r['fx'] for r in readings)/len(readings):.4f} N")
        print(f"  Mean Fy: {sum(r['fy'] for r in readings)/len(readings):.4f} N")
        print(f"  Mean Fz: {sum(r['fz'] for r in readings)/len(readings):.4f} N")
        
        # Check for suspicious values
        for i, r in enumerate(readings):
            magnitude = (r['fx']**2 + r['fy']**2 + r['fz']**2)**0.5
            if magnitude > 100:
                print(f"  ⚠️  HIGH VALUE at reading {i+1}: {magnitude:.2f} N")
    
    return readings


def test_tare_operation(stage):
    """Test the tare operation"""
    print_separator("TARE OPERATION TEST")
    
    print("Step 1: Reading values BEFORE tare...")
    status, data_before = stage.read_hex_sensor()
    if status == SerialInterface.ReplyStatus.OK:
        fx_before = data_before['fx'] / 1000.0
        fy_before = data_before['fy'] / 1000.0
        fz_before = data_before['fz'] / 1000.0
        print(f"  Before: Fx={fx_before:.4f} N | Fy={fy_before:.4f} N | Fz={fz_before:.4f} N")
    else:
        print(f"  ERROR reading before tare: {status}")
        return False
    
    time.sleep(0.5)
    
    print("\nStep 2: Sending TARE command...")
    status = stage.tare_hex_sensor()
    print(f"  Tare command status: {status}")
    
    print("  Waiting 3 seconds for tare to settle...")
    time.sleep(3.0)
    
    print("\nStep 3: Reading values AFTER tare...")
    status, data_after = stage.read_hex_sensor()
    if status == SerialInterface.ReplyStatus.OK:
        fx_after = data_after['fx'] / 1000.0
        fy_after = data_after['fy'] / 1000.0
        fz_after = data_after['fz'] / 1000.0
        print(f"  After:  Fx={fx_after:.4f} N | Fy={fy_after:.4f} N | Fz={fz_after:.4f} N")
    else:
        print(f"  ERROR reading after tare: {status}")
        return False
    
    print_separator()
    print("TARE EFFECTIVENESS:")
    print(f"  Fx offset: {abs(fx_after):.6f} N (should be ~0.000...)")
    print(f"  Fy offset: {abs(fy_after):.6f} N (should be ~0.000...)")
    print(f"  Fz offset: {abs(fz_after):.6f} N (should be ~0.000...)")
    
    if max(abs(fx_after), abs(fy_after), abs(fz_after)) > 0.01:
        print("  ⚠️  WARNING: Tare did not properly zero the sensor!")
        return False
    else:
        print("  ✓ Tare operation successful!")
        return True


def manual_pull_test(stage, duration_s=5.0):
    """Let user manually pull/push sensor and record values"""
    print_separator("MANUAL FORCE APPLICATION TEST")
    print(f"This test records forces while you manually pull/push the sensor.")
    print(f"Duration: {duration_s} seconds\n")
    
    input("Press ENTER when ready to start, then apply forces to the sensor...")
    
    readings = []
    start_time = time.time()
    sample_count = 0
    
    print(f"\nRecording for {duration_s:.1f} seconds...")
    while time.time() - start_time < duration_s:
        status, data = stage.read_hex_sensor()
        if status == SerialInterface.ReplyStatus.OK:
            elapsed = time.time() - start_time
            fx_n = data['fx'] / 1000.0
            fy_n = data['fy'] / 1000.0
            fz_n = data['fz'] / 1000.0
            magnitude = (fx_n**2 + fy_n**2 + fz_n**2)**0.5
            
            readings.append({
                'time': elapsed,
                'fx': fx_n,
                'fy': fy_n,
                'fz': fz_n,
                'magnitude': magnitude
            })
            
            if sample_count % 5 == 0:
                print(f"  [{elapsed:.1f}s] Fx: {fx_n:8.4f} N | Fy: {fy_n:8.4f} N | "
                      f"Fz: {fz_n:8.4f} N | Mag: {magnitude:8.4f} N")
            
            sample_count += 1
        
        time.sleep(0.1)
    
    print(f"\nCollected {sample_count} samples")
    
    if readings:
        print_separator()
        print("STATISTICS:")
        magnitudes = [r['magnitude'] for r in readings]
        print(f"  Max force magnitude: {max(magnitudes):.4f} N")
        print(f"  Min force magnitude: {min(magnitudes):.4f} N")
        print(f"  Mean force magnitude: {sum(magnitudes)/len(magnitudes):.4f} N")
        
        # Find peak
        peak_idx = magnitudes.index(max(magnitudes))
        peak = readings[peak_idx]
        print(f"\n  Peak force at {peak['time']:.1f}s:")
        print(f"    Fx={peak['fx']:.4f} N, Fy={peak['fy']:.4f} N, Fz={peak['fz']:.4f} N")
        print(f"    Magnitude: {peak['magnitude']:.4f} N")
    
    return readings


def test_communication_timing(stage, num_reads=20):
    """Test communication timing to detect serial issues"""
    print_separator("COMMUNICATION TIMING TEST")
    print(f"Testing {num_reads} rapid reads to check for timing issues...\n")
    
    times = []
    errors = 0
    
    for i in range(num_reads):
        start = time.time()
        status, data = stage.read_hex_sensor()
        elapsed = time.time() - start
        times.append(elapsed * 1000)  # Convert to ms
        
        if status != SerialInterface.ReplyStatus.OK:
            print(f"  [{i+1:2d}] ERROR: {status} ({elapsed*1000:.1f}ms)")
            errors += 1
        else:
            if elapsed * 1000 > 100:  # Flag if slow
                print(f"  [{i+1:2d}] SLOW: {elapsed*1000:.1f}ms ⚠️")
            else:
                print(f"  [{i+1:2d}] OK: {elapsed*1000:.1f}ms")
    
    print_separator()
    print("TIMING STATISTICS:")
    if times:
        times_ms = [t for t in times if t > 0]
        print(f"  Min response: {min(times_ms):.1f} ms")
        print(f"  Max response: {max(times_ms):.1f} ms")
        print(f"  Avg response: {sum(times_ms)/len(times_ms):.1f} ms")
        print(f"  Errors: {errors}/{num_reads}")
        
        if max(times_ms) > 500:
            print("  ⚠️  WARNING: Some reads are taking >500ms (timeout might be occurring)")
        if errors > 0:
            print(f"  ⚠️  WARNING: {errors} read errors occurred")


def main():
    """Main diagnostics routine"""
    print("\n")
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║           HEX Force/Torque Sensor Diagnostics                ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    
    PORT = 'COM8'
    BAUD_RATE = 921600
    
    print(f"\nConnecting to {PORT} at {BAUD_RATE} baud...")
    
    try:
        stage = OpenMicroStageInterface(show_communication=False, show_log_messages=False)
        stage.connect(PORT, BAUD_RATE)
        
        print("✓ Connected!\n")
        
        # Run diagnostic tests
        while True:
            print("\n╔══════════════════════════════════════════════════════════════╗")
            print("║                    DIAGNOSTIC MENU                           ║")
            print("╠══════════════════════════════════════════════════════════════╣")
            print("║ 1. Read raw sensor values (10 samples)                       ║")
            print("║ 2. Test tare operation                                       ║")
            print("║ 3. Manual force test (pull/push sensor)                      ║")
            print("║ 4. Communication timing test                                 ║")
            print("║ 5. Enable motors and run all tests                           ║")
            print("║ 6. Exit                                                      ║")
            print("╚══════════════════════════════════════════════════════════════╝")
            
            choice = input("\nSelect test (1-6): ").strip()
            
            if choice == '1':
                test_raw_sensor_values(stage, num_readings=10)
            
            elif choice == '2':
                test_tare_operation(stage)
            
            elif choice == '3':
                manual_pull_test(stage, duration_s=5.0)
            
            elif choice == '4':
                test_communication_timing(stage, num_reads=20)
            
            elif choice == '5':
                print("\nEnabling motors for full diagnostic...")
                stage.enable_motors(True)
                time.sleep(0.5)
                
                test_raw_sensor_values(stage, num_readings=5)
                test_tare_operation(stage)
                manual_pull_test(stage, duration_s=5.0)
                test_communication_timing(stage, num_reads=10)
                
                print("\nDisabling motors...")
                stage.enable_motors(False)
            
            elif choice == '6':
                print("\nExiting diagnostics...")
                break
            
            else:
                print("Invalid choice, please try again")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\nDisconnecting...")
        if 'stage' in locals():
            stage.disconnect()
        print("✓ Done")


if __name__ == '__main__':
    main()
