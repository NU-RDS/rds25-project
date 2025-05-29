#!/usr/bin/env python3
"""
Simple RDS Hand Control GUI
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
from typing import Optional, Dict
import queue

# Command IDs (match your MCU definitions)


class CommandID:
    # Individual joint commands
    SET_JOINT_WRIST_ROLL = 1
    SET_JOINT_WRIST_PITCH = 2
    SET_JOINT_WRIST_YAW = 3
    SET_JOINT_DEX_PIP = 4
    SET_JOINT_DEX_DIP = 5
    SET_JOINT_DEX_MCP = 6
    SET_JOINT_DEX_SPLAIN = 7
    SET_JOINT_POW_GRASP = 8
    SET_ALL_JOINTS = 10

    # System commands
    EMERGENCY_STOP = 20
    EMERGENCY_CLEAR = 21
    GET_STATUS = 30
    GET_POSITIONS = 31

    # Preset commands
    PRESET_OPEN_HAND = 40
    PRESET_CLOSE_HAND = 41

    # Joint name to command ID mapping
    JOINT_MAP = {
        'wrist_roll': SET_JOINT_WRIST_ROLL,
        'wrist_pitch': SET_JOINT_WRIST_PITCH,
        'wrist_yaw': SET_JOINT_WRIST_YAW,
        'dex_pip': SET_JOINT_DEX_PIP,
        'dex_dip': SET_JOINT_DEX_DIP,
        'dex_mcp': SET_JOINT_DEX_MCP,
        'dex_splain': SET_JOINT_DEX_SPLAIN,
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
        'dex_splain': JointLimits(-15.0, 15.0),
        'pow_grasp': JointLimits(0.0, 120.0),
        'wrist_roll': JointLimits(-90.0, 90.0),
        'wrist_pitch': JointLimits(-70.0, 60.0),
        'wrist_yaw': JointLimits(-30.0, 20.0)
    }


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
                        value_str = ",".join([f"{v:.2f}" for v in values])
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

    def set_position_display(self, position: float):
        """Set the desired position display without sending command"""
        position = max(self.limits.min_val, min(self.limits.max_val, position))
        self.position_var.set(position)
        self.entry_var.set(f"{position:.1f}")


class RDSHandGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("RDS Hand Control - Simple Command ID System")
        self.root.geometry("800x700")

        self.communicator = SerialCommunicator()
        self.joint_controls: Dict[str, JointControlFrame] = {}

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

        # Wrist controls
        wrist_group = ttk.LabelFrame(scrollable_frame, text="Wrist Control")
        wrist_group.pack(fill='x', padx=10, pady=5)

        for i, joint in enumerate(['wrist_roll', 'wrist_pitch', 'wrist_yaw']):
            limits = RDSConstants.JOINT_LIMITS[joint]
            control = JointControlFrame(wrist_group, joint, limits)
            control.grid(row=i, column=0, sticky='ew', pady=2)
            self.joint_controls[joint] = control

        # Dexterous finger controls
        dex_group = ttk.LabelFrame(scrollable_frame, text="Dexterous Finger")
        dex_group.pack(fill='x', padx=10, pady=5)

        for i, joint in enumerate(['dex_pip', 'dex_dip', 'dex_mcp', 'dex_splain']):
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
        ttk.Button(button_frame, text="SEND ALL POSITIONS",
                   command=self.send_all_positions,
                   style="Accent.TButton").pack(side='left', padx=10)

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

        ttk.Button(emergency_group, text="EMERGENCY STOP",
                   command=self.emergency_stop,
                   style="Danger.TButton").pack(side='left', padx=10, pady=10)

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
        """Send all current joint positions to MCU in one message"""
        if not self.communicator.is_connected:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")
            return

        # Collect all joint values in the correct order
        # Order: wrist_roll, wrist_pitch, wrist_yaw, dex_pip, dex_dip, dex_mcp, dex_splain, pow_grasp
        joint_order = ['wrist_roll', 'wrist_pitch', 'wrist_yaw',
                       'dex_pip', 'dex_dip', 'dex_mcp', 'dex_splain', 'pow_grasp']

        values = []
        for joint_name in joint_order:
            if joint_name in self.joint_controls:
                values.append(self.joint_controls[joint_name].get_position())
            else:
                values.append(0.0)  # Default value if joint not found

        # Send all values in one command
        sent_command = self.communicator.send_command(
            CommandID.SET_ALL_JOINTS, values)
        if sent_command:
            value_str = ", ".join(
                [f"{joint_order[i]}={values[i]:.1f}°" for i in range(len(values))])
            self.log_message(f"Sent all positions: {sent_command}")
            self.log_message(f"Values: {value_str}")

    def load_open_preset(self):
        """Load open hand preset values into controls (don't send yet)"""
        preset_values = {
            'wrist_roll': 0.0,
            'wrist_pitch': 0.0,
            'wrist_yaw': 0.0,
            'dex_pip': 0.0,
            'dex_dip': 0.0,
            'dex_mcp': 0.0,
            'dex_splain': 0.0,
            'pow_grasp': 0.0
        }

        for joint_name, value in preset_values.items():
            if joint_name in self.joint_controls:
                self.joint_controls[joint_name].set_position_display(value)

        self.log_message("Open hand preset loaded (click 'SEND ALL' to apply)")

    def load_close_preset(self):
        """Load close hand preset values into controls (don't send yet)"""
        preset_values = {
            'wrist_roll': 0.0,
            'wrist_pitch': 0.0,
            'wrist_yaw': 0.0,
            'dex_pip': 45.0,
            'dex_dip': 45.0,
            'dex_mcp': 45.0,
            'dex_splain': 0.0,
            'pow_grasp': 60.0
        }

        for joint_name, value in preset_values.items():
            if joint_name in self.joint_controls:
                self.joint_controls[joint_name].set_position_display(value)

        self.log_message(
            "Close hand preset loaded (click 'SEND ALL' to apply)")

    def reset_all_joints(self):
        """Reset all joint displays to zero (don't send yet)"""
        for control in self.joint_controls.values():
            control.set_position_display(0.0)

        self.log_message(
            "All joints reset to zero (click 'SEND ALL' to apply)")

    def emergency_stop(self):
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(
                CommandID.EMERGENCY_STOP)
            if sent_command:
                self.log_message(f"EMERGENCY STOP: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def emergency_clear(self):
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(
                CommandID.EMERGENCY_CLEAR)
            if sent_command:
                self.log_message(f"Emergency cleared: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def preset_open_hand(self):
        """Send open hand preset directly to MCU"""
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(
                CommandID.PRESET_OPEN_HAND)
            if sent_command:
                self.log_message(f"Open hand preset sent: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def preset_close_hand(self):
        """Send close hand preset directly to MCU"""
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(
                CommandID.PRESET_CLOSE_HAND)
            if sent_command:
                self.log_message(f"Close hand preset sent: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def get_status(self):
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(CommandID.GET_STATUS)
            if sent_command:
                self.log_message(f"Status request: {sent_command}")
        else:
            messagebox.showwarning(
                "Not Connected", "Please connect to MCU first")

    def get_positions(self):
        if self.communicator.is_connected:
            sent_command = self.communicator.send_command(
                CommandID.GET_POSITIONS)
            if sent_command:
                self.log_message(f"Position request: {sent_command}")
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
                self.log_message(f"Received: {message}")

                # Parse position updates if they come from MCU
                # Example format: "[HIGH] POSITIONS: 0.0,0.0,0.0,45.0,30.0,15.0,0.0,60.0"
                if "POSITIONS:" in message:
                    try:
                        pos_data = message.split("POSITIONS:")[1].strip()
                        positions = [float(x.strip())
                                     for x in pos_data.split(',')]

                        if len(positions) >= 8:
                            joint_names = ['wrist_roll', 'wrist_pitch', 'wrist_yaw',
                                           'dex_pip', 'dex_dip', 'dex_mcp', 'dex_splain', 'pow_grasp']

                            for i, joint_name in enumerate(joint_names):
                                if joint_name in self.joint_controls:
                                    self.joint_controls[joint_name].update_current_position(
                                        positions[i])
                    except:
                        pass  # Ignore parsing errors

        except queue.Empty:
            pass

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
