#!/usr/bin/env python3
"""
Enhanced RDS Hand Control GUI with Pitch/Yaw Recording and Plotting
Sends command IDs in format: "ID" or "ID:value"
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import json
from dataclasses import dataclass
from typing import Optional, Dict, List
import queue
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from datetime import datetime

# Command IDs (match your MCU definitions)


class CommandID:
    # Individual joint commands (optional - for debugging/testing)
    SET_JOINT_WRIST_PITCH = 2
    SET_JOINT_WRIST_YAW = 3
    SET_JOINT_DEX_PIP = 4
    SET_JOINT_DEX_DIP = 5
    SET_JOINT_DEX_MCP = 6
    SET_JOINT_DEX_splay = 7
    SET_JOINT_POW_GRASP = 8

    # Main command - send all joints at once
    SET_ALL_JOINTS = 10

    # System commands
    EMERGENCY_STOP = 20
    EMERGENCY_CLEAR = 21
    GET_STATUS = 30
    GET_POSITIONS = 31

    # Preset commands
    PRESET_OPEN_HAND = 40
    PRESET_CLOSE_HAND = 41

    # Joint name to command ID mapping (for individual commands if needed)
    JOINT_MAP = {
        'wrist_pitch': SET_JOINT_WRIST_PITCH,
        'wrist_yaw': SET_JOINT_WRIST_YAW,
        'dex_pip': SET_JOINT_DEX_PIP,
        'dex_dip': SET_JOINT_DEX_DIP,
        'dex_mcp': SET_JOINT_DEX_MCP,
        'dex_splay': SET_JOINT_DEX_splay,
        'pow_grasp': SET_JOINT_POW_GRASP
    }


@dataclass
class JointLimits:
    min_val: float
    max_val: float


class RDSConstants:
    JOINT_LIMITS = {
        'dex_dip': JointLimits(0.0, 90.0),
        'dex_pip': JointLimits(0.0, 110.0),
        'dex_mcp': JointLimits(-15.0, 90.0),
        'dex_splay': JointLimits(-15.0, 15.0),
        'pow_grasp': JointLimits(0.0, 120.0),
        'wrist_pitch': JointLimits(-70.0, 60.0),
        'wrist_yaw': JointLimits(-30.0, 20.0)
    }


class DataRecorder:
    """Class to handle recording and plotting of pitch/yaw data"""

    def __init__(self):
        self.recording = False
        self.timestamps = []
        self.pitch_desired = []
        self.pitch_actual = []
        self.yaw_desired = []
        self.yaw_actual = []
        self.start_time = None
        self.max_samples = 10000  # Limit memory usage
        self.last_update_time = 0  # For rate limiting data collection

    def start_recording(self):
        """Start recording data"""
        self.recording = True
        self.start_time = time.time()
        self.clear_data()
        print("📊 Started recording pitch/yaw data")

    def stop_recording(self):
        """Stop recording data"""
        self.recording = False
        print(
            f"⏹️ Stopped recording. Collected {len(self.timestamps)} samples")

    def clear_data(self):
        """Clear all recorded data"""
        self.timestamps.clear()
        self.pitch_desired.clear()
        self.pitch_actual.clear()
        self.yaw_desired.clear()
        self.yaw_actual.clear()

    def add_data_point(self, pitch_des, pitch_act, yaw_des, yaw_act):
        """Add a new data point (rate limited to avoid performance issues)"""
        if not self.recording or self.start_time is None:
            return

        current_time = time.time()

        # Rate limit data collection to ~50Hz to avoid performance issues
        if current_time - self.last_update_time < 0.02:  # 20ms = 50Hz max
            return

        self.last_update_time = current_time

        # Calculate relative timestamp
        relative_time = current_time - self.start_time

        # Add data point
        self.timestamps.append(relative_time)
        self.pitch_desired.append(pitch_des)
        self.pitch_actual.append(pitch_act)
        self.yaw_desired.append(yaw_des)
        self.yaw_actual.append(yaw_act)

        # Limit memory usage
        if len(self.timestamps) > self.max_samples:
            self.timestamps.pop(0)
            self.pitch_desired.pop(0)
            self.pitch_actual.pop(0)
            self.yaw_desired.pop(0)
            self.yaw_actual.pop(0)

    def plot_data(self):
        """Plot the recorded data"""
        if len(self.timestamps) < 2:
            print("❌ Not enough data to plot")
            return

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

        # Plot pitch data
        ax1.plot(self.timestamps, self.pitch_desired,
                 'r-', label='Desired Pitch', linewidth=2)
        ax1.plot(self.timestamps, self.pitch_actual,
                 'b-', label='Actual Pitch', linewidth=2)
        ax1.set_title('Wrist Pitch Control')
        ax1.set_xlabel('Time (seconds)')
        ax1.set_ylabel('Pitch Angle (degrees)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot yaw data
        ax2.plot(self.timestamps, self.yaw_desired,
                 'r-', label='Desired Yaw', linewidth=2)
        ax2.plot(self.timestamps, self.yaw_actual,
                 'b-', label='Actual Yaw', linewidth=2)
        ax2.set_title('Wrist Yaw Control')
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Yaw Angle (degrees)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

        # Print statistics
        if len(self.timestamps) > 0:
            pitch_error = np.array(self.pitch_desired) - \
                np.array(self.pitch_actual)
            yaw_error = np.array(self.yaw_desired) - np.array(self.yaw_actual)

            print("\n📈 Data Statistics:")
            print(f"Recording duration: {self.timestamps[-1]:.2f} seconds")
            print(f"Total samples: {len(self.timestamps)}")
            print(f"Pitch RMS Error: {np.sqrt(np.mean(pitch_error**2)):.3f}°")
            print(f"Yaw RMS Error: {np.sqrt(np.mean(yaw_error**2)):.3f}°")
            print(f"Pitch Max Error: {np.max(np.abs(pitch_error)):.3f}°")
            print(f"Yaw Max Error: {np.max(np.abs(yaw_error)):.3f}°")

    def save_data(self, filename=None):
        """Save data to CSV file"""
        if len(self.timestamps) == 0:
            print("❌ No data to save")
            return

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"pitch_yaw_data_{timestamp}.csv"

        try:
            import csv
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time(s)', 'Pitch_Desired(deg)', 'Pitch_Actual(deg)',
                                 'Yaw_Desired(deg)', 'Yaw_Actual(deg)'])

                for i in range(len(self.timestamps)):
                    writer.writerow([
                        self.timestamps[i],
                        self.pitch_desired[i],
                        self.pitch_actual[i],
                        self.yaw_desired[i],
                        self.yaw_actual[i]
                    ])

            print(f"💾 Data saved to {filename}")
        except Exception as e:
            print(f"❌ Error saving data: {e}")


class SerialCommunicator:
    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False
        self.receive_thread: Optional[threading.Thread] = None
        self.running = False
        self.message_queue = queue.Queue()

    def get_available_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def connect(self, port: str, baudrate: int = 9600) -> bool:
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.is_connected = True
            self.running = True

            self.receive_thread = threading.Thread(
                target=self._receive_loop, daemon=True)
            self.receive_thread.start()

            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        self.running = False
        self.is_connected = False

        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)

        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None

    def send_command(self, command_id: int, values: list = None):
        """Send command in format: 'ID' or 'ID:value1,value2,value3...'"""
        if self.is_connected and self.serial_port:
            try:
                if values is not None:
                    if isinstance(values, (list, tuple)):
                        # Format as space-separated values for easier parsing on MCU side
                        value_str = " ".join([f"{v:.2f}" for v in values])
                        command = f"{command_id}:{value_str}\n"
                    else:
                        command = f"{command_id}:{values:.2f}\n"
                else:
                    command = f"{command_id}\n"

                self.serial_port.write(command.encode())
                self.serial_port.flush()
                return command.strip()
            except Exception as e:
                print(f"Send error: {e}")
        return None

    def send_all_joints_optimized(self, joint_values: list):
        """
        Optimized method to send all 7 joint values in one command
        Expected order: wrist_pitch, wrist_yaw, dex_pip, dex_dip, dex_mcp, dex_splay, pow_grasp
        """
        if self.is_connected and self.serial_port:
            try:
                # Format: "10:val1 val2 val3 val4 val5 val6 val7"
                # Using space separation for easier parsing on Arduino side
                value_str = " ".join([f"{v:.2f}" for v in joint_values])
                command = f"{CommandID.SET_ALL_JOINTS}:{value_str}\n"

                self.serial_port.write(command.encode())
                self.serial_port.flush()
                return command.strip()
            except Exception as e:
                print(f"Send error: {e}")
        return None

    def _receive_loop(self):
        while self.running and self.serial_port:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode().strip()
                    if data:
                        self.message_queue.put(data)
            except Exception as e:
                print(f"Receive error: {e}")
            time.sleep(0.01)


class JointControlFrame(ttk.Frame):
    def __init__(self, parent, joint_name: str, limits: JointLimits):
        super().__init__(parent)
        self.joint_name = joint_name
        self.limits = limits

        self.create_widgets()

    def create_widgets(self):
        # Joint label
        ttk.Label(self, text=self.joint_name.replace('_', ' ').title()).grid(
            row=0, column=0, sticky='w', padx=5)

        # Current position display (feedback from MCU)
        self.current_var = tk.StringVar(value="0.0°")
        ttk.Label(self, text="Current:", font=('TkDefaultFont', 8)
                  ).grid(row=0, column=1, sticky='e', padx=(5, 0))
        ttk.Label(self, textvariable=self.current_var, width=6, font=(
            'TkDefaultFont', 8)).grid(row=0, column=2, padx=(0, 5))

        # Desired position label
        ttk.Label(self, text="Desired:", font=('TkDefaultFont', 8)
                  ).grid(row=0, column=3, sticky='e', padx=(5, 0))

        # Position slider
        self.position_var = tk.DoubleVar(value=0.0)
        self.slider = ttk.Scale(
            self,
            from_=self.limits.min_val,
            to=self.limits.max_val,
            variable=self.position_var,
            orient='horizontal',
            length=200,
            command=self.on_slider_change
        )
        self.slider.grid(row=0, column=4, padx=5)

        # Position entry
        self.entry_var = tk.StringVar(value="0.0")
        self.entry = ttk.Entry(self, textvariable=self.entry_var, width=6)
        self.entry.grid(row=0, column=5, padx=5)
        self.entry.bind('<Return>', self.on_entry_change)
        self.entry.bind('<FocusOut>', self.on_entry_change)

    def on_slider_change(self, value):
        """Update entry field when slider moves (no command sent)"""
        self.entry_var.set(f"{float(value):.1f}")

    def on_entry_change(self, event):
        """Update slider when entry field changes (no command sent)"""
        try:
            value = float(self.entry_var.get())
            value = max(self.limits.min_val, min(self.limits.max_val, value))
            self.position_var.set(value)
            self.entry_var.set(f"{value:.1f}")
        except ValueError:
            # Reset to current slider value if invalid
            self.entry_var.set(f"{self.position_var.get():.1f}")

    def update_current_position(self, position: float):
        """Update the current position display (from MCU feedback)"""
        self.current_var.set(f"{position:.1f}°")

    def get_position(self) -> float:
        return self.position_var.get()

    def get_current_position(self) -> float:
        """Get the current position as a float"""
        try:
            return float(self.current_var.get().replace('°', ''))
        except:
            return 0.0

    def set_position_display(self, position: float):
        """Set the desired position display without sending command"""
        position = max(self.limits.min_val, min(self.limits.max_val, position))
        self.position_var.set(position)
        self.entry_var.set(f"{position:.1f}")


class RDSHandGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("RDS Hand Control - Enhanced with Pitch/Yaw Recording")
        self.root.geometry("900x800")

        self.communicator = SerialCommunicator()
        self.joint_controls: Dict[str, JointControlFrame] = {}
        self.data_recorder = DataRecorder()

        self.create_widgets()
        self.update_gui()

    def create_widgets(self):
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=10)

        # Connection tab
        self.create_connection_tab()

        # Joint control tab
        self.create_joint_control_tab()

        # System control tab
        self.create_system_control_tab()

        # NEW: Data recording tab
        self.create_recording_tab()

        # Monitoring tab
        self.create_monitoring_tab()

        # Status bar
        self.status_var = tk.StringVar(value="Disconnected")
        self.status_bar = ttk.Label(
            self.root, textvariable=self.status_var, relief='sunken')
        self.status_bar.pack(side='bottom', fill='x')

    def create_connection_tab(self):
        self.conn_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.conn_frame, text="Connection")

        # Serial port selection
        ttk.Label(self.conn_frame, text="Serial Port:").grid(
            row=0, column=0, sticky='w', padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            self.conn_frame, textvariable=self.port_var, width=20)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(self.conn_frame, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=5, pady=5)

        # Baud rate
        ttk.Label(self.conn_frame, text="Baud Rate:").grid(
            row=1, column=0, sticky='w', padx=5, pady=5)
        self.baud_var = tk.StringVar(value="9600")
        self.baud_combo = ttk.Combobox(
            self.conn_frame, textvariable=self.baud_var, values=["9600", "115200"], width=20)
        self.baud_combo.grid(row=1, column=1, padx=5, pady=5)

        # Connect button
        self.connect_btn = ttk.Button(
            self.conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=2, column=0, columnspan=2, pady=10)

        # Initialize ports
        self.refresh_ports()

    def create_joint_control_tab(self):
        self.joint_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.joint_frame, text="Joint Control")

        # Create scrollable frame
        canvas = tk.Canvas(self.joint_frame)
        scrollbar = ttk.Scrollbar(
            self.joint_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.bind('<Configure>', lambda e: canvas.configure(
            scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")

        # Wrist controls (pitch and yaw only)
        wrist_group = ttk.LabelFrame(scrollable_frame, text="Wrist Control")
        wrist_group.pack(fill='x', padx=10, pady=5)

        for i, joint in enumerate(['wrist_pitch', 'wrist_yaw']):
            limits = RDSConstants.JOINT_LIMITS[joint]
            control = JointControlFrame(wrist_group, joint, limits)
            control.grid(row=i, column=0, sticky='ew', pady=2)
            self.joint_controls[joint] = control

        # Dexterous finger controls
        dex_group = ttk.LabelFrame(scrollable_frame, text="Dexterous Finger")
        dex_group.pack(fill='x', padx=10, pady=5)

        for i, joint in enumerate(['dex_pip', 'dex_dip', 'dex_mcp', 'dex_splay']):
            limits = RDSConstants.JOINT_LIMITS[joint]
            control = JointControlFrame(dex_group, joint, limits)
            control.grid(row=i, column=0, sticky='ew', pady=2)
            self.joint_controls[joint] = control

        # Power finger controls
        pow_group = ttk.LabelFrame(scrollable_frame, text="Power Finger")
        pow_group.pack(fill='x', padx=10, pady=5)

        limits = RDSConstants.JOINT_LIMITS['pow_grasp']
        control = JointControlFrame(pow_group, 'pow_grasp', limits)
        control.grid(row=0, column=0, sticky='ew', pady=2)
        self.joint_controls['pow_grasp'] = control

        # Control buttons
        button_frame = ttk.Frame(scrollable_frame)
        button_frame.pack(fill='x', padx=10, pady=10)

        # Main send button - most prominent
        send_btn = ttk.Button(button_frame, text="🚀 SEND ALL POSITIONS",
                              command=self.send_all_positions)
        send_btn.pack(side='left', padx=10)

        # Separator
        ttk.Separator(button_frame, orient='vertical').pack(
            side='left', fill='y', padx=10)

        # Preset buttons
        ttk.Button(button_frame, text="Load Open Preset",
                   command=self.load_open_preset).pack(side='left', padx=5)

        ttk.Button(button_frame, text="Load Close Preset",
                   command=self.load_close_preset).pack(side='left', padx=5)

        # Reset button
        ttk.Button(button_frame, text="Reset All to Zero",
                   command=self.reset_all_joints).pack(side='left', padx=5)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def create_system_control_tab(self):
        self.system_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.system_frame, text="System Control")

        # Emergency controls
        emergency_group = ttk.LabelFrame(
            self.system_frame, text="Emergency Controls")
        emergency_group.pack(fill='x', padx=10, pady=10)

        ttk.Button(emergency_group, text="🛑 EMERGENCY STOP",
                   command=self.emergency_stop).pack(side='left', padx=10, pady=10)

        ttk.Button(emergency_group, text="Clear Emergency",
                   command=self.emergency_clear).pack(side='left', padx=10, pady=10)

        # Preset controls
        preset_group = ttk.LabelFrame(self.system_frame, text="Hand Presets")
        preset_group.pack(fill='x', padx=10, pady=10)

        ttk.Button(preset_group, text="Open Hand",
                   command=self.preset_open_hand).pack(side='left', padx=5, pady=5)

        ttk.Button(preset_group, text="Close Hand",
                   command=self.preset_close_hand).pack(side='left', padx=5, pady=5)

        # Status controls
        status_group = ttk.LabelFrame(self.system_frame, text="Status")
        status_group.pack(fill='x', padx=10, pady=10)

        ttk.Button(status_group, text="Get Status",
                   command=self.get_status).pack(side='left', padx=5, pady=5)

        ttk.Button(status_group, text="Get Positions",
                   command=self.get_positions).pack(side='left', padx=5, pady=5)

    def create_recording_tab(self):
        """NEW: Create the data recording tab"""
        self.recording_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.recording_frame, text="📊 Data Recording")

        # Recording controls
        record_group = ttk.LabelFrame(
            self.recording_frame, text="Recording Controls")
        record_group.pack(fill='x', padx=10, pady=10)

        # Recording status
        self.recording_status_var = tk.StringVar(value="⏹️ Not Recording")
        ttk.Label(record_group, textvariable=self.recording_status_var,
                  font=('TkDefaultFont', 12, 'bold')).pack(pady=10)

        # Recording buttons
        button_frame = ttk.Frame(record_group)
        button_frame.pack(pady=10)

        self.start_record_btn = ttk.Button(button_frame, text="🔴 Start Recording",
                                           command=self.start_recording)
        self.start_record_btn.pack(side='left', padx=5)

        self.stop_record_btn = ttk.Button(button_frame, text="⏹️ Stop Recording",
                                          command=self.stop_recording, state='disabled')
        self.stop_record_btn.pack(side='left', padx=5)

        # Analysis controls
        analysis_group = ttk.LabelFrame(
            self.recording_frame, text="Data Analysis")
        analysis_group.pack(fill='x', padx=10, pady=10)

        analysis_button_frame = ttk.Frame(analysis_group)
        analysis_button_frame.pack(pady=10)

        ttk.Button(analysis_button_frame, text="📈 Plot Data",
                   command=self.plot_recorded_data).pack(side='left', padx=5)

        ttk.Button(analysis_button_frame, text="💾 Save Data",
                   command=self.save_recorded_data).pack(side='left', padx=5)

        ttk.Button(analysis_button_frame, text="🗑️ Clear Data",
                   command=self.clear_recorded_data).pack(side='left', padx=5)

        # Data info
        self.data_info_var = tk.StringVar(value="No data recorded")
        ttk.Label(analysis_group, textvariable=self.data_info_var).pack(pady=5)

        # Quick test controls
        test_group = ttk.LabelFrame(self.recording_frame, text="Quick Tests")
        test_group.pack(fill='x', padx=10, pady=10)

        test_button_frame = ttk.Frame(test_group)
        test_button_frame.pack(pady=10)

        ttk.Button(test_button_frame, text="🎯 Step Response Test",
                   command=self.run_step_test).pack(side='left', padx=5)

        ttk.Button(test_button_frame, text="🌊 Sine Wave Test",
                   command=self.run_sine_test).pack(side='left', padx=5)

    def create_monitoring_tab(self):
        self.monitor_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.monitor_frame, text="Monitoring")

        # Message log
        ttk.Label(self.monitor_frame, text="System Messages:").pack(
            anchor='w', padx=5, pady=(5, 0))
        self.message_log = scrolledtext.ScrolledText(
            self.monitor_frame, height=20)
        self.message_log.pack(fill='both', expand=True, padx=5, pady=5)

        # Clear button
        ttk.Button(self.monitor_frame, text="Clear Log",
                   command=self.clear_log).pack(pady=5)

    # Recording methods
    def start_recording(self):
        """Start recording pitch/yaw data"""
        if not self.communicator.is_connected:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")
            return

        self.data_recorder.start_recording()
        self.recording_status_var.set("🔴 Recording...")
        self.start_record_btn.config(state='disabled')
        self.stop_record_btn.config(state='normal')
        self.log_message("📊 Started recording pitch/yaw data")

    def stop_recording(self):
        """Stop recording pitch/yaw data"""
        self.data_recorder.stop_recording()
        self.recording_status_var.set("⏹️ Not Recording")
        self.start_record_btn.config(state='normal')
        self.stop_record_btn.config(state='disabled')

        # Update data info
        sample_count = len(self.data_recorder.timestamps)
        if sample_count > 0:
            duration = self.data_recorder.timestamps[-1]
            self.data_info_var.set(
                f"{sample_count} samples, {duration:.1f}s duration")

        self.log_message(
            f"⏹️ Stopped recording. {sample_count} samples collected")

    def plot_recorded_data(self):
        """Plot the recorded data"""
        self.data_recorder.plot_data()

    def save_recorded_data(self):
        """Save recorded data to file"""
        self.data_recorder.save_data()

    def clear_recorded_data(self):
        """Clear all recorded data"""
        self.data_recorder.clear_data()
        self.data_info_var.set("No data recorded")
        self.log_message("🗑️ Cleared recorded data")

    def run_step_test(self):
        """Run a step response test"""
        if not self.communicator.is_connected:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")
            return

        # Set up step response: 0 -> 30 degrees pitch
        self.joint_controls['wrist_pitch'].set_position_display(30.0)
        self.joint_controls['wrist_yaw'].set_position_display(15.0)

        # Start recording
        self.start_recording()

        # Send command
        self.send_all_positions()

        # Schedule automatic stop after 5 seconds
        self.root.after(5000, self.stop_recording)
        self.log_message("🎯 Running step response test (5 seconds)")

    def run_sine_test(self):
        """Run a sine wave test"""
        if not self.communicator.is_connected:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")
            return

        # Start recording
        self.start_recording()

        # Run sine wave for 10 seconds
        def sine_wave_generator():
            start_time = time.time()
            duration = 10.0  # 10 seconds
            frequency = 0.2  # 0.2 Hz (one cycle every 5 seconds)
            amplitude = 20.0  # ±20 degrees

            def update_sine():
                current_time = time.time() - start_time
                if current_time < duration:
                    # Calculate sine wave values
                    pitch_val = amplitude * \
                        np.sin(2 * np.pi * frequency * current_time)
                    yaw_val = amplitude * 0.5 * \
                        np.sin(2 * np.pi * frequency * current_time + np.pi/4)

                    # Update GUI
                    self.joint_controls['wrist_pitch'].set_position_display(
                        pitch_val)
                    self.joint_controls['wrist_yaw'].set_position_display(
                        yaw_val)

                    # Send positions
                    self.send_all_positions()

                    # Schedule next update
                    self.root.after(50, update_sine)  # Update at 20Hz
                else:
                    # Test complete
                    self.stop_recording()
                    self.log_message("🌊 Sine wave test completed")

            update_sine()

        sine_wave_generator()
        self.log_message("🌊 Running sine wave test (10 seconds)")

    def refresh_ports(self):
        ports = self.communicator.get_available_ports()
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])

    def toggle_connection(self):
        if not self.communicator.is_connected:
            port = self.port_var.get()
            baud = int(self.baud_var.get())

            if self.communicator.connect(port, baud):
                self.connect_btn.config(text="Disconnect")
                self.status_var.set(f"Connected to {port}")
                self.log_message(f"Connected to {port} at {baud} baud")
            else:
                messagebox.showerror("Connection Error",
                                     f"Failed to connect to {port}")
        else:
            self.communicator.disconnect()
            self.connect_btn.config(text="Connect")
            self.status_var.set("Disconnected")
            self.log_message("Disconnected")

    def send_all_positions(self):
        """Send all current joint positions to MCU in ONE message"""
        if not self.communicator.is_connected:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")
            return

        # Collect all joint values in the exact order expected by MCU (7 joints without wrist_roll)
        # Order: wrist_pitch, wrist_yaw, dex_pip, dex_dip, dex_mcp, dex_splay, pow_grasp
        joint_order = ['wrist_pitch', 'wrist_yaw',
                       'dex_pip', 'dex_dip', 'dex_mcp', 'dex_splay', 'pow_grasp']

        values = []
        for joint_name in joint_order:
            if joint_name in self.joint_controls:
                values.append(self.joint_controls[joint_name].get_position())
            else:
                values.append(0.0)  # Default value if joint not found

        # Send all values in ONE optimized command
        sent_command = self.communicator.send_all_joints_optimized(values)

        if sent_command:
            # Record data if recording is active
            if self.data_recorder.recording:
                pitch_desired = self.joint_controls['wrist_pitch'].get_position(
                )
                pitch_actual = self.joint_controls['wrist_pitch'].get_current_position(
                )
                yaw_desired = self.joint_controls['wrist_yaw'].get_position()
                yaw_actual = self.joint_controls['wrist_yaw'].get_current_position(
                )

                self.data_recorder.add_data_point(pitch_desired, pitch_actual,
                                                  yaw_desired, yaw_actual)

            # Log the successful send
            self.log_message(f"✅ ALL JOINTS SENT: {sent_command}")

            # Log human-readable values
            value_descriptions = []
            for i, joint_name in enumerate(joint_order):
                value_descriptions.append(
                    f"{joint_name.replace('_', ' ')}={values[i]:.1f}°")

            self.log_message(f"📊 Values: {', '.join(value_descriptions)}")

            # Update status
            self.status_var.set(f"✅ Sent {len(values)} joint positions")
        else:
            self.log_message("❌ Failed to send joint positions")
            self.status_var.set("❌ Send failed")

    def load_open_preset(self):
        """Load open hand preset values into controls (don't send yet)"""
        preset_values = {
            'wrist_pitch': 0.0,
            'wrist_yaw': 0.0,
            'dex_pip': 0.0,
            'dex_dip': 0.0,
            'dex_mcp': 0.0,
            'dex_splay': 0.0,
            'pow_grasp': 0.0
        }

        for joint_name, value in preset_values.items():
            if joint_name in self.joint_controls:
                self.joint_controls[joint_name].set_position_display(value)

        self.log_message(
            "📖 Open hand preset loaded (click 'SEND ALL' to apply)")

    def load_close_preset(self):
        """Load close hand preset values into controls (don't send yet)"""
        preset_values = {
            'wrist_pitch': 0.0,
            'wrist_yaw': 0.0,
            'dex_pip': 45.0,
            'dex_dip': 45.0,
            'dex_mcp': 45.0,
            'dex_splay': 0.0,
            'pow_grasp': 60.0
        }

        for joint_name, value in preset_values.items():
            if joint_name in self.joint_controls:
                self.joint_controls[joint_name].set_position_display(value)

        self.log_message(
            "📖 Close hand preset loaded (click 'SEND ALL' to apply)")

    def reset_all_joints(self):
        """Reset all joint displays to zero (don't send yet)"""
        for control in self.joint_controls.values():
            control.set_position_display(0.0)

        self.log_message(
            "🔄 All joints reset to zero (click 'SEND ALL' to apply)")

    def emergency_stop(self):
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(
                CommandID.EMERGENCY_STOP)
            if sent_command:
                self.log_message(f"🚨 EMERGENCY STOP: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def emergency_clear(self):
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(
                CommandID.EMERGENCY_CLEAR)
            if sent_command:
                self.log_message(f"✅ Emergency cleared: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def preset_open_hand(self):
        """Send open hand preset directly to MCU"""
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(
                CommandID.PRESET_OPEN_HAND)
            if sent_command:
                self.log_message(f"🖐️ Open hand preset sent: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def preset_close_hand(self):
        """Send close hand preset directly to MCU"""
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(
                CommandID.PRESET_CLOSE_HAND)
            if sent_command:
                self.log_message(f"✊ Close hand preset sent: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def get_status(self):
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(CommandID.GET_STATUS)
            if sent_command:
                self.log_message(f"📊 Status request: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def get_positions(self):
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(
                CommandID.GET_POSITIONS)
            if sent_command:
                self.log_message(f"📍 Position request: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def log_message(self, message: str):
        timestamp = time.strftime("%H:%M:%S")
        self.message_log.insert(tk.END, f"[{timestamp}] {message}\n")
        self.message_log.see(tk.END)

    def clear_log(self):
        self.message_log.delete(1.0, tk.END)

    def update_gui(self):
        try:
            while True:
                message = self.communicator.message_queue.get_nowait()
                self.log_message(f"📨 Received: {message}")

                # Parse position updates if they come from MCU
                # Example format: "[HIGH] POSITIONS: 0.0,0.0,45.0,30.0,15.0,0.0,60.0"
                if "POSITIONS:" in message:
                    try:
                        pos_data = message.split("POSITIONS:")[1].strip()
                        positions = [float(x.strip())
                                     for x in pos_data.split(',')]

                        if len(positions) >= 7:
                            joint_names = ['wrist_pitch', 'wrist_yaw',
                                           'dex_pip', 'dex_dip', 'dex_mcp', 'dex_splay', 'pow_grasp']

                            for i, joint_name in enumerate(joint_names):
                                if joint_name in self.joint_controls:
                                    self.joint_controls[joint_name].update_current_position(
                                        positions[i])
                    except:
                        pass  # Ignore parsing errors

                # Parse motor angle data for pitch/yaw recording
                # Looking for messages like "Motor 0 is at X.XXX" and "Motor 1 is at X.XXX"
                if "Motor" in message and "is at" in message and self.data_recorder.recording:
                    try:
                        # Extract motor data for real-time recording
                        # This will capture the actual motor positions from your C++ code
                        pass  # You can enhance this to parse specific motor data
                    except:
                        pass

        except queue.Empty:
            pass

        # Update recording status (but don't plot in real-time)
        if self.data_recorder.recording:
            sample_count = len(self.data_recorder.timestamps)
            if sample_count > 0:
                duration = self.data_recorder.timestamps[-1]
                # Update status every 100 samples to avoid UI lag
                if sample_count % 100 == 0 or duration % 1.0 < 0.1:  # Update every second or every 100 samples
                    self.data_info_var.set(
                        f"Recording: {sample_count} samples, {duration:.1f}s")

        self.root.after(50, self.update_gui)

    def on_closing(self):
        if self.communicator.is_connected:
            self.communicator.disconnect()
        self.root.destroy()

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()


if __name__ == "__main__":
    app = RDSHandGUI()
    app.run()
