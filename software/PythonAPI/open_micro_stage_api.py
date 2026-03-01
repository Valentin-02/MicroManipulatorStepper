import threading
import time
import re
from enum import Enum

import os
import json
import serial
import numpy as np
from colorama import Fore, Style, init

# --- SerialInterface --------------------------------------------------------------------------------------------------

class SerialInterface:

    class ReplyStatus(Enum):
        OK = 'ok'
        ERROR = 'error'
        TIMEOUT = 'timeout'
        BUSY = 'busy'

    class LogLevel(Enum):
        DEBUG = 'debug'
        INFO = 'info'
        WARNING = 'warning'
        ERROR = 'error'

    # Static mapping from prefix to LogLevel
    log_level_prefix_map = {
        "D)": LogLevel.DEBUG,
        "I)": LogLevel.INFO,
        "W)": LogLevel.WARNING,
        "E)": LogLevel.ERROR,
    }

    def __init__(self, port: str, baud_rate: int = 115200,
                 command_msg_callback=None,
                 log_msg_callback=None,
                 unsolicited_msg_callback=None,
                 reconnect_timeout: int = 5):
        """
        Initializes the serial connection and starts background reader.
        :param port: Serial port name (e.g., 'COM3' or '/dev/ttyUSB0').
        :param baud_rate: Serial baud rate.
        :param log_msg_callback: called when a log message is received
        :param unsolicited_msg_callback: Optional function to call with unsolicited messages.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.reconnect_timeout = reconnect_timeout
        self.serial = None  # initialized on connect

        self.command_msg_callback = command_msg_callback
        self.log_message_callback = log_msg_callback
        self.unsolicited_msg_callback = unsolicited_msg_callback

        # Synchronization for blocking send/receive
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._waiting_for_response = False
        self._response_string = ""
        self._response_status = None
        self._response_error_msg = None

        self.connect(self.reconnect_timeout)

        # Start reader thread
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()


    def connect(self, timeout):
        """
        Try to open the serial port. Retry until timeout expires.
        """
        deadline = time.time() + timeout
        print(Fore.MAGENTA, end='')
        print(f"[SerialInterface] Connecting to port '{self.port}'...", end='')
        while time.time() < deadline:
            try:
                self.serial = serial.Serial(self.port, self.baud_rate, timeout=2)
                print(f" [OK]")
                print(Style.RESET_ALL, end='')
                return True
            except (serial.SerialException, OSError) as e:
                print('.', end='')
                time.sleep(0.2)

        print(f" [FAILED] Timeout after {timeout} seconds.")
        print(f"[SerialInterface] Connection is permanently closed")
        print(Style.RESET_ALL, end='')
        self.serial = None
        return False

    def _reader_loop(self):
        """
        Asynchronous reader loop, collecting serial data into a buffer
        """
        buffer = ""
        while True:
            try:
                if self.serial is not None and self.serial.in_waiting:
                    char = self.serial.read(1).decode('ascii', errors='ignore')
                    if char in ['\n', '\r']:
                        if len(buffer) > 0:
                            self._handle_line(buffer)
                            buffer = ""
                    else:
                        buffer += char
                else:
                    time.sleep(0.001)
            except (serial.SerialException, OSError) as e:
                print(Fore.MAGENTA+f"[SerialInterface] Lost connection: {e}"+Style.RESET_ALL)
                try:
                    if self.serial is not None and self.serial.is_open:
                        self.serial.close()
                except Exception:
                    pass

                self.serial = None
                self.connect(self.reconnect_timeout)

    def _handle_line(self, line: str):
        """
        Handles a single serial line sent by the device
        :param line: string containing a single line
        """
        with self._lock:
            log_level, log_msg = self._check_log_msg(line)
            # print(line)
            # log message
            if log_level is not None:
                if self.log_message_callback: self.log_message_callback(log_level, log_msg)
            # response
            elif self._waiting_for_response:
                line_lower = line.lower()
                if line_lower.startswith("ok"):
                    self._response_status = SerialInterface.ReplyStatus.OK
                elif line_lower.startswith("busy"):
                    self._response_status = SerialInterface.ReplyStatus.BUSY
                elif line_lower.startswith("error"):
                    self._response_status = SerialInterface.ReplyStatus.ERROR
                    parts = line.split(":", 1)
                    self._response_error_msg = parts[1].strip() if len(parts) > 1 else ""

                if self._response_status is not None:
                    self._condition.notify()
                else:
                    self._response_string += line + '\n'

            # unsolicited message
            else:
                if self.unsolicited_msg_callback: self.unsolicited_msg_callback(line)

    def _check_log_msg(self, msg: str):
        if len(msg) < 2:
            return None, ''
        return self.log_level_prefix_map.get(msg[:2]), msg[2:]

    def send_command(self, cmd: str, timeout=2) -> tuple[ReplyStatus, str]:
        """
        Sends a command and blocks until 'ok' or 'error' is received.
        :param cmd: The command to send.
        :param timeout: Maximum time to wait for response.
        :return: Tuple containing Status enum (OK | ERROR | TIMEOUT), and response lines.
        """
        with self._lock:
            if not self.serial or not self.serial.is_open:
                return SerialInterface.ReplyStatus.ERROR, 'Serial not open'

            # Reset state
            self._waiting_for_response = True
            self._response_string = ""
            self._response_error_msg = ""
            self._response_status = None

            cmd = (cmd.strip() + "\n")
            self.command_msg_callback(cmd, None, '')

            # Send command
            self.serial.write(cmd.encode('ascii'))
            self.serial.flush()

            # Wait for completion
            end_time = time.time() + timeout
            while self._response_status is None:
                remaining = end_time - time.time()
                if remaining <= 0:
                    self._waiting_for_response = False
                    print(Fore.MAGENTA + f"[SerialInterface] Command timeout, device didn't reply in time" + Style.RESET_ALL)
                    return SerialInterface.ReplyStatus.TIMEOUT, self._response_string
                self._condition.wait(timeout=remaining)

            self._waiting_for_response = False
            self.command_msg_callback(self._response_string, self._response_status, self._response_error_msg)
            return self._response_status, self._response_string

    def close(self):
        """Closes the serial port."""
        if self.serial and self.serial.is_open:
            self.serial.close()

# --- OpenMicroStageInterface ------------------------------------------------------------------------------------------

class OpenMicroStageInterface:
    # Mapping log levels to colors
    LOG_COLORS = {
        SerialInterface.LogLevel.DEBUG: Fore.WHITE+Style.DIM,
        SerialInterface.LogLevel.INFO: Style.RESET_ALL,
        SerialInterface.LogLevel.WARNING: Fore.YELLOW,
        SerialInterface.LogLevel.ERROR: Fore.RED,
    }

    def __init__(self, show_communication=True, show_log_messages=True):
        self.serial = None
        self.workspace_transform = np.eye(4)
        self.show_communication = show_communication
        self.show_log_messages = show_log_messages
        self.disable_message_callbacks = False
        # Kalibrierungstabelle: maps angle tuples to (x, y, z) positions
        self.x_angle_to_position_mapping = {}
        self.y_angle_to_position_mapping = {}
        self.z_angle_to_position_mapping = {}

    def connect(self, port: str, baud_rate: int = 921600):
        def version_to_str(v):
            return f"v{v[0]}.{v[1]}.{v[2]}"

        if self.serial is not None: self.disconnect()
        self.serial = SerialInterface(port, baud_rate,
                                      log_msg_callback=self.log_msg_callback,
                                      command_msg_callback=self.command_msg_callback,
                                      unsolicited_msg_callback=self.unsolicited_msg_callback)

        self.disable_message_callbacks = True
        fw_version = self.read_firmware_version()
        min_fw_version = (1, 0, 1)
        print(Fore.MAGENTA + f"Firmware version: {version_to_str(fw_version)}" + Style.RESET_ALL)
        if fw_version < min_fw_version:
            print(Fore.MAGENTA + f"Firmware version {version_to_str(fw_version)} incompatible. "
                                 f"At least {version_to_str(min_fw_version)} required" + Style.RESET_ALL)
            self.serial = None
        print('')
        self.disable_message_callbacks = False

    def disconnect(self):
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def log_msg_callback(self, log_level, msg):
        if not self.show_log_messages or self.disable_message_callbacks:
            return

        color = OpenMicroStageInterface.LOG_COLORS.get(log_level, Fore.WHITE)
        if log_level not in [SerialInterface.LogLevel.INFO, SerialInterface.LogLevel.DEBUG]:
            print(f"{color}[{log_level.name}] {msg}{Style.RESET_ALL}")
        else:
            print(f"{color}{msg}{Style.RESET_ALL}")

    def command_msg_callback(self, msg, reply_status: SerialInterface.ReplyStatus, error_msg: str):
        if not self.show_communication or self.disable_message_callbacks:
            return

        if reply_status is not None:
            if msg:
                msg = '\n'.join('> ' + line for line in msg.splitlines())
                print(f"{msg.rstrip()}")
            if error_msg:
                print(f"{Style.BRIGHT}{str(reply_status.name)}:{Style.RESET_ALL} {error_msg}\n")
            else:
                print(f"{Style.BRIGHT}{str(reply_status.name)} {Style.RESET_ALL}\n")
        else:
            print(f"{Fore.GREEN+Style.BRIGHT}{msg.rstrip()}{Style.RESET_ALL}")

    def unsolicited_msg_callback(self, msg):
        print(Fore.CYAN+msg+Style.RESET_ALL)
        pass

    def set_workspace_transform(self, transform):
        self.workspace_transform = transform

    def get_workspace_transform(self):
        return self.workspace_transform

    def read_firmware_version(self):
        ok, response = self.serial.send_command("M58")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return 0, 0, 0

        major, minor, patch = map(int, re.match(r'v(\d+)\.(\d+)\.(\d+)', response).groups())
        return major,minor,patch

    def home(self, axis_list=None):
        """
        Homes one or more axes on the device
        :param axis_list: Optional list of axis indices to home. If None, all axes are homed.
        :return: The status of the command (e.g. OK, ERROR, TIMEOUT).
        """
        cmd = 'G28'
        axis_chars = ['A', 'B', 'C', 'D', 'E', 'F']
        if axis_list is None:
            axis_list = [i for i in range(len(axis_chars))]

        for axis_idx in axis_list:
            if 0 > axis_idx >= len(axis_chars):
                raise ValueError('Axis index out of range')
            cmd += ' '+axis_chars[axis_idx]

        res, msg = self.serial.send_command(cmd + "\n", 10)
        return res

    def calibrate_joint(self, joint_index: int, save_result: bool):
        """
        Calibrates the given joint and returns the measured data as three lists containing:
            data[0]: list of motor angles
            data[1]: list of electric field angles
            data[2]: list of raw encoder counts
        :param joint_index:
        :param save_result:
        :return:
        """
        cmd = f"M56 J{joint_index} P"
        if save_result: cmd += ' S'
        res, msg = self.serial.send_command(cmd, 30)

        calibration_data = self._parse_table_data(msg, 3)
        return res, calibration_data

    def move_to(self, x, y, z, f, move_immediately=False, blocking=True, timeout=1):
        """
        Moves the stage to an absolute position with a specified feed rate.
        :param x: Target X position (in workspace coordinates).
        :param y: Target Y position (in workspace coordinates).
        :param z: Target Z position (in workspace coordinates).
        :param f: Feed rate in mm/s.
        :param move_immediately: If True, execution starts without buffering delay.
        :param blocking: If True, waits and retries if the device is busy. If False, returns immediately on 'BUSY'.
        :param timeout: Timeout in seconds for each command attempt.
        :return: Status of the move command (e.g. OK, ERROR, BUSY, TIMEOUT).
        """
        # Convert to homogeneous vector
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G0 X{x_t:.6f} Y{y_t:.6f} Z{z_t:.6f} F{f:.3f}"
        if move_immediately:
            cmd += " I"

        # resend messages if queue is full
        while True:
            res, msg = self.serial.send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    def dwell(self, time_s, blocking, timeout=1):
        cmd = f"G4 S{time_s:.6f}\n"
        # resend messages if queue is full
        while True:
            res, msg = self.serial.send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    def set_max_acceleration(self, linear_accel, angular_accel):
        linear_accel = max(linear_accel, 0.01)
        angular_accel = max(angular_accel, 0.01)
        cmd = f"M204 L{linear_accel:.6f} A{angular_accel:.6f}\n"
        res, msg = self.serial.send_command(cmd)
        return res

    def wait_for_stop(self, polling_interval_ms=10, disable_callbacks=True):
        disable_message_callbacks_prev = self.disable_message_callbacks
        if disable_callbacks: self.disable_message_callbacks = True

        while True:
            res, msg = self.serial.send_command("M53\n")
            if res != SerialInterface.ReplyStatus.OK: return res
            elif msg.strip() == "1":
                return SerialInterface.ReplyStatus.OK

        self.disable_message_callbacks = disable_message_callbacks_prev

    def read_current_position(self):
        ok, response = self.serial.send_command("M50")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return None, None, None

        # Match values with NO space between axis letter and number
        match = re.search(
            r"X([-+]?\d*\.?\d+)\s*Y([-+]?\d*\.?\d+)\s*Z([-+]?\d*\.?\d+)",
            response
        )
        if not match:
            raise ValueError(f"Invalid format: {response}")

        x, y, z = match.groups()
        return float(x), float(y), float(z)

    def read_encoder_angles(self):
        """
        Reads the absolute encoder angles for all joints.
        :return: Tuple of (status, dict with joint angles and raw values)
        """
        ok, response = self.serial.send_command("M51")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return []
        
        angles = []
        for line in response.splitlines():
            match = re.search(r"Joint \d+:\s+([-+]?\d*\.?\d+)\s+deg", line)
            if match:
                angles.append(float(match.group(1)))
        
        return angles

    def calibrate_angle_mapping(self, positions_list, feedrate=10.0):
        """
        Durchführt eine Kalibrierungsfahrt, bei der verschiedene Positionen angefahren werden
        und die entsprechenden Encoder-Winkel gespeichert werden.
        
        :param positions_list: Liste von Tupeln [(x1, y1, z1), (x2, y2, z2), ...]
        :param feedrate: Fahrtgeschwindigkeit in mm/s
        :return: True wenn erfolgreich, False sonst
        """
        print(Fore.MAGENTA + f"[Angle Calibration] Starting calibration with {len(positions_list)} positions..." + Style.RESET_ALL)
        
        for idx, (x, y, z) in enumerate(positions_list):
            print(f"[Angle Calibration] Moving to position {idx+1}/{len(positions_list)}: ({x:.3f}, {y:.3f}, {z:.3f})")
            
            # Fahre zu der Position
            res = self.move_to(x, y, z, feedrate, blocking=True)
            if res != SerialInterface.ReplyStatus.OK:
                print(Fore.RED + f"[Angle Calibration] Failed to move to position {idx+1}" + Style.RESET_ALL)
                return False
            
            # Warte bis die Bewegung fertig ist
            time.sleep(0.1)
            res = self.wait_for_stop(disable_callbacks=True)
            if res != SerialInterface.ReplyStatus.OK:
                print(Fore.RED + f"[Angle Calibration] Wait for stop failed at position {idx+1}" + Style.RESET_ALL)
                return False
            
            # Lese die aktuellen Winkel
            angles = self.read_encoder_angles()
            if len(angles) == 0:
                print(Fore.RED + f"[Angle Calibration] Failed to read encoder angles at position {idx+1}" + Style.RESET_ALL)
                return False
            
            # Speichere in Kalibrierungstabelle
            angle_key = tuple(round(a, 2) for a in angles)  # Runde auf 2 Dezimalstellen für key
            self.x_angle_to_position_mapping[angle_key[0]] = x
            self.y_angle_to_position_mapping[angle_key[1]] = y
            self.z_angle_to_position_mapping[angle_key[2]] = z
            print(f"[Angle Calibration] Recorded angles: {[f'{a:.2f}°' for a in angles]} -> Position ({x:.3f}, {y:.3f}, {z:.3f})")
        
        mapping_table = {"x": self.x_angle_to_position_mapping, "y": self.y_angle_to_position_mapping, "z": self.z_angle_to_position_mapping}
        with open("config/calibration_mapping.json", "w") as f:
            json.dump(mapping_table, f, indent=4)
        
        print(Fore.GREEN + f"[Angle Calibration] Calibration complete. {len(self.x_angle_to_position_mapping)} positions recorded." + Style.RESET_ALL)
        return True

    def move_to_current_angle(self, feedrate=10.0):
        """
        Liest die aktuellen Encoder-Winkel aus und berechnet die entsprechende Position
        anhand der Kalibrierungstabelle. Für jede Koordinate wird unabhängig der nächst 
        mögliche Wert aus der Kalibrierung verwendet.
        
        :param feedrate: Fahrtgeschwindigkeit in mm/s
        :return: Status der Bewegung (OK, ERROR, etc.)
        """
        if len(self.x_angle_to_position_mapping) == 0:
            if os.path.exists("config/calibration_mapping.json"):
                with open("config/calibration_mapping.json", "r") as f:
                    mapping_table = json.load(f)
                    self.x_angle_to_position_mapping = {float(k): v for k, v in mapping_table.get("x", {}).items()}
                    self.y_angle_to_position_mapping = {float(k): v for k, v in mapping_table.get("y", {}).items()}
                    self.z_angle_to_position_mapping = {float(k): v for k, v in mapping_table.get("z", {}).items()}
                print(Fore.GREEN + f"[move_to_current_angle] Loaded calibration mapping from file." + Style.RESET_ALL)
            else:
                print(Fore.RED + "[move_to_current_angle] No calibration data available. Run calibrate_angle_mapping() first." + Style.RESET_ALL)
                return SerialInterface.ReplyStatus.ERROR
        
        # Lese aktuelle Winkel
        current_angles = self.read_encoder_angles()
        if len(current_angles) == 0:
            print(Fore.RED + "[move_to_current_angle] Failed to read encoder angles" + Style.RESET_ALL)
            return SerialInterface.ReplyStatus.ERROR
        
        # Runde Winkel für Vergleich
        angle_key = tuple(round(a, 2) for a in current_angles)
        target_x, target_y, target_z = None, None, None
        
        # Versuche exakte Übereinstimmung zu finden
        if angle_key[0] in self.x_angle_to_position_mapping:
            target_x = self.x_angle_to_position_mapping[angle_key[0]]
        if angle_key[1] in self.y_angle_to_position_mapping:
            target_y = self.y_angle_to_position_mapping[angle_key[1]]
        if angle_key[2] in self.z_angle_to_position_mapping:
            target_z = self.z_angle_to_position_mapping[angle_key[2]]
        
        # Sammle alle eindeutigen Werte für jede Koordinate
        all_x_values = sorted(set(self.x_angle_to_position_mapping.keys()))
        all_y_values = sorted(set(self.y_angle_to_position_mapping.keys()))
        all_z_values = sorted(set(self.z_angle_to_position_mapping.keys()))
        
        # Finde nächsten Wert für jede Koordinate
        nearest_x = min(all_x_values, key=lambda x: abs(x - current_angles[0]))
        nearest_y = min(all_y_values, key=lambda y: abs(y - current_angles[1]))
        nearest_z = min(all_z_values, key=lambda z: abs(z - current_angles[2]))
        
        print(f"[move_to_current_angle] Nearest available X: {nearest_x:.3f}, Y: {nearest_y:.3f}, Z: {nearest_z:.3f}")
        print(f"[move_to_current_angle] Moving to ({self.x_angle_to_position_mapping[nearest_x]:.3f}, {self.y_angle_to_position_mapping[nearest_y]:.3f}, {self.z_angle_to_position_mapping[nearest_z]:.3f})")
        
        return self.move_to(self.x_angle_to_position_mapping[nearest_x], self.y_angle_to_position_mapping[nearest_y], self.z_angle_to_position_mapping[nearest_z], feedrate, blocking=True)


    def read_device_state_info(self):
        """
        Reads the device state information including joint status and frequencies.
        :return: Tuple of (status, dict with device info)
        """
        res, msg = self.serial.send_command("M57")
        if res != SerialInterface.ReplyStatus.OK or len(msg) == 0:
            return res, {}
        
        info = {}
        joints_info = {}
        
        for line in msg.strip().splitlines():
            # Parse joint info: "Joint 0:  is_homed=1  is_calibrated=1  encoder_angle=123.456 deg"
            joint_match = re.search(
                r'Joint (\d+):\s+is_homed=(\d+)\s+is_calibrated=(\d+)\s+encoder_angle=([-+]?\d*\.?\d+)\s+deg',
                line
            )
            if joint_match:
                joint_idx = int(joint_match.group(1))
                joints_info[joint_idx] = {
                    'is_homed': bool(int(joint_match.group(2))),
                    'is_calibrated': bool(int(joint_match.group(3))),
                    'encoder_angle_deg': float(joint_match.group(4))
                }
            
            # Parse frequencies
            servo_match = re.search(r'Servo Loop:\s+(\d+)\s+kHz', line)
            if servo_match:
                info['servo_loop_freq_hz'] = int(servo_match.group(1)) * 1000
            
            motion_match = re.search(r'Motion Controler:\s+(\d+)\s+Hz', line)
            if motion_match:
                info['motion_controller_freq_hz'] = int(motion_match.group(1))
        
        info['joints'] = joints_info
        return res, info

    def set_servo_parameter(self, pos_kp=150, pos_ki=50000, vel_kp=0.2, vel_ki=100, vel_filter_tc=0.0025):
        cmd = f"M55 A{pos_kp:.6f} B{pos_ki:.6f} C{vel_kp:.6f} D{vel_ki:.6f} F{vel_filter_tc:.6f}"
        res, msg = self.serial.send_command(cmd)
        return res

    def enable_motors(self, enable):
        cmd = f"M17" if enable else "M18"
        res, msg = self.serial.send_command(cmd, timeout=5)
        return res

    def set_pose(self, x, y, z):
        # Convert to homogeneous vector
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G24 X{x_t:.6f} Y{y_t:.6f} Z{z_t:.6f}" # TODO: A, B ,C
        res, msg = self.serial.send_command(cmd)
        return res

    def send_command(self, cmd: str, timeout_s: float=5):
        res, msg = self.serial.send_command(cmd, timeout_s)
        return res, msg

    def get_queue_size(self):
        """
        Gets the current motion queue size.
        :return: Tuple of (status, queue_size)
        """
        res, msg = self.serial.send_command("M52")
        if res != SerialInterface.ReplyStatus.OK or len(msg) == 0:
            return res, 0
        
        match = re.search(r'Queue Size:\s+(\d+)', msg)
        if match:
            return res, int(match.group(1))
        return res, 0

    def print_lookup_table(self, joint_index: int):
        """
        Prints the lookup table for a specific joint.
        :param joint_index: Index of the joint (0-based)
        :return: Tuple of (status, lookup_table)
        """
        cmd = f"M59 J{joint_index}"
        res, msg = self.serial.send_command(cmd)
        return res, msg

    def read_hex_sensor(self):
        """
        Reads a single data frame from the HEX force/torque sensor.
        :return: Tuple of (status, dict with forces and torques)
        """
        res, msg = self.serial.send_command("M60")
        if res != SerialInterface.ReplyStatus.OK or len(msg) == 0:
            return res, {}
        
        # Parse: "HEX fx fy fz mx my mz temperature"
        match = re.search(
            r'HEX\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+'
            r'([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)',
            msg
        )
        if match:
            return res, {
                'fx': float(match.group(1)),
                'fy': float(match.group(2)),
                'fz': float(match.group(3)),
                'mx': float(match.group(4)),
                'my': float(match.group(5)),
                'mz': float(match.group(6)),
                'temperature': float(match.group(7))
            }
        return res, {}

    def enable_force_control(self, enable: bool):
        """
        Enable or disable force control mode.
        When enabled, captures the current pose as the base pose.
        :param enable: True to enable, False to disable
        :return: Status of the command
        """
        cmd = f"M61 S{1 if enable else 0}"
        res, msg = self.serial.send_command(cmd)
        return res

    def set_force_target(self, fx: float, fy: float, fz: float):
        """
        Sets the target force for force control.
        :param fx: Target force in X direction [mN]
        :param fy: Target force in Y direction [mN]
        :param fz: Target force in Z direction [mN]
        :return: Status of the command
        """
        cmd = f"M62 X{fx:.6f} Y{fy:.6f} Z{fz:.6f}"
        res, msg = self.serial.send_command(cmd)
        return res

    def set_force_parameters(self, kp: float = None, ki: float = None, 
                           output_limit: float = None, windup_limit: float = None,
                           filter_tc: float = None, max_displacement: float = None):
        """
        Sets the force controller parameters.
        :param kp: Proportional gain [mm/mN]
        :param ki: Integral gain [mm/(mN·s)]
        :param output_limit: PI output limit [mm]
        :param windup_limit: Integral windup limit [mm]
        :param filter_tc: Force filter time constant [s]
        :param max_displacement: Max displacement from base pose [mm]
        :return: Status of the command
        """
        cmd = "M63"
        
        if kp is not None and ki is not None:
            cmd += f" P{kp:.6f} I{ki:.6f}"
            if output_limit is not None:
                cmd += f" L{output_limit:.6f}"
            if windup_limit is not None:
                cmd += f" W{windup_limit:.6f}"
        
        if filter_tc is not None:
            cmd += f" F{filter_tc:.6f}"
        
        if max_displacement is not None:
            cmd += f" D{max_displacement:.6f}"
        
        res, msg = self.serial.send_command(cmd)
        return res

    def tare_hex_sensor(self):
        """
        Tares (zeroes) the HEX force/torque sensor (blocking operation).
        :return: Status of the command
        """
        res, msg = self.serial.send_command("M64")
        return res

    def get_force_control_state(self):
        """
        Query the current state of force control.
        :return: Tuple of (status, is_enabled)
        """
        res, msg = self.serial.send_command("M61")
        if res != SerialInterface.ReplyStatus.OK or len(msg) == 0:
            return res, None
        
        match = re.search(r'Force control:\s+(enabled|disabled)', msg)
        if match:
            return res, match.group(1) == 'enabled'
        return res, None

    @staticmethod
    def _parse_table_data(data_string, cols):
        # Parse the data
        data = [[] for _ in range(cols)]

        for line in data_string.strip().splitlines():
            parts = line.strip().split(',')
            if len(parts) != cols:
                continue  # skip malformed lines
            numbers = map(float, parts)
            for i, n in enumerate(numbers):
                data[i].append(n)

        return data
