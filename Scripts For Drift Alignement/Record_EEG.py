import sys
import re
import serial
import socket
import time
import serial.tools.list_ports
import threading
import numpy as np
import pyqtgraph as pg

# Use pyqtgraph's built-in Qt wrapper to prevent version clashes
from pyqtgraph.Qt import QtWidgets, QtCore 
from collections import deque
from pylsl import StreamInfo, StreamOutlet, local_clock

# ============================================
# Configuration
# ============================================
BAUD_RATE = 921600 
MAX_SAMPLES = 1000
TARGET_PORT = "COM12"
NOMINAL_SRATE = 200 # 5ms interval from the nRF51822

# Dynamic Data Structures
node_data = {}      # Holds the GUI plotting queues
drift_models = {}   # Holds the individual clock sync math
outlets = {}        # Holds the individual LSL streams
last_seq_trackers = {}   # Tracks sequence numbers for dropped packets
virtual_t_nodes = {}     # Tracks the mathematically perfect internal time
last_print_times = {}    # Tracks console I/O throttling

data_lock = threading.Lock()
ser = None  

# ============================================
# Synchronization Math Models
# ============================================
class ClockDriftModel:
    def __init__(self):
        self.sync_points = [] # Holds tuples of (t_node, t_esp)
        self.m = 1.0
        self.b = 0.0
        self.last_t_node_raw = 0
        self.wrap_add = 0
        
    def unwrap(self, t_node_raw):
        if t_node_raw < self.last_t_node_raw - 1000000:
            self.wrap_add += (2**32)
        self.last_t_node_raw = t_node_raw
        return t_node_raw + self.wrap_add

    def add_sync(self, t1_esp, t2_esp, t_node_raw):
        t_node_unwrapped = self.unwrap(t_node_raw)
        t_esp_est = t1_esp + ((t2_esp - t1_esp) / 2.0) 
        
        # Store the exact (x, y) coordinate: (Node Time, ESP Time)
        self.sync_points.append((t_node_unwrapped, t_esp_est))
        
        # Keep a sliding window of the 15 most recent sync points
        if len(self.sync_points) > 15:
            self.sync_points.pop(0)
            
        # Perform Linear Regression (y = mx + b)
        if len(self.sync_points) >= 2:
            x = np.array([p[0] for p in self.sync_points])
            y = np.array([p[1] for p in self.sync_points])
            
            # np.polyfit(x, y, 1) returns the [slope, intercept]
            self.m, self.b = np.polyfit(x, y, 1)
        elif len(self.sync_points) == 1:
            # Fallback if we only have one point
            self.m = 1.0
            self.b = t_esp_est - t_node_unwrapped

    def translate(self, virtual_t_node_unwrapped):
        # y = mx + b mapping
        return (self.m * virtual_t_node_unwrapped) + self.b
        
'''
class ClockDriftModel:
    def __init__(self):
        # Automatically drops the oldest point when it hits 50 items
        self.sync_points = deque(maxlen=50) 
        self.m = 1.0
        self.b = 0.0
        self.last_t_node_raw = 0
        self.wrap_add = 0
        self.x_base = 0.0

    def unwrap(self, t_node_raw):
        if t_node_raw < self.last_t_node_raw - 1000000:
            self.wrap_add += (2**32)
        self.last_t_node_raw = t_node_raw
        return t_node_raw + self.wrap_add

    def add_sync(self, t1_esp, t2_esp, t_node_raw):
        rtt = t2_esp - t1_esp
        if rtt > 15000: 
            return 
            
        t_node_unwrapped = self.unwrap(t_node_raw)
        t_esp_est = t1_esp + (rtt / 2.0) 
        
        # deque automatically handles the 50-point limit
        self.sync_points.append((t_node_unwrapped, t_esp_est))
        
        if len(self.sync_points) >= 2:
            x = np.array([p[0] for p in self.sync_points], dtype=np.float64)
            y = np.array([p[1] for p in self.sync_points], dtype=np.float64)
            
            self.x_base = x[0]
            x_centered = x - self.x_base
            self.m, self.b = np.polyfit(x_centered, y, 1)
            
        elif len(self.sync_points) == 1:
            self.m = 1.0
            self.x_base = t_node_unwrapped
            self.b = t_esp_est

    def translate(self, virtual_t_node_unwrapped):
        # Apply the centered mapping
        if len(self.sync_points) == 0:
            return virtual_t_node_unwrapped
        return (self.m * (virtual_t_node_unwrapped - self.x_base)) + self.b
'''

class PCOffsetTracker:
    def __init__(self):
        self.offsets = []
        
    def update(self, t2_esp_us, pc_time_sec):
        t2_sec = t2_esp_us / 1000000.0
        self.offsets.append(pc_time_sec - t2_sec)
        if len(self.offsets) > 20:
            self.offsets.pop(0)
            
    def get_offset(self):
        if not self.offsets:
            return 0.0
        return np.mean(self.offsets)

pc_offset_tracker = PCOffsetTracker()

# ============================================
# Command LabRecorder Through RCS Port (Background)
# ============================================
def send_labrecorder_command(command_str):
    try:
        s = socket.create_connection(("localhost", 22345), timeout=1)
        s.sendall(f"{command_str}\n".encode('utf-8'))
        s.close()
        print(f"[RCS] Sent: {command_str}")
    except ConnectionRefusedError:
        print("[RCS] Error: Connection refused. Is LabRecorder RCS enabled?")
    except Exception as e:
        print(f"[RCS] Error: {e}")

def labrecorder_worker(cmd_char):
    """Runs in a background thread to prevent GUI freezing"""
    if cmd_char == 'a':
        send_labrecorder_command("update")
        time.sleep(0.5) 
        send_labrecorder_command("select all")
        send_labrecorder_command("start")
    elif cmd_char == 's':
        send_labrecorder_command("stop")

# ============================================
# Dynamic Node Registration
# ============================================
def register_new_node(node_name):
    with data_lock:
        if node_name not in node_data:
            print(f"[SYSTEM] Discovered new device: {node_name}. Initializing streams...")
            node_data[node_name] = deque([np.nan]*MAX_SAMPLES, maxlen=MAX_SAMPLES)
            
    # LSL initialization moved outside the data_lock to prevent blocking
    if node_name not in drift_models:
        drift_models[node_name] = ClockDriftModel()
        last_print_times[node_name] = 0

        stream_name = f'EEG_{node_name.replace(" ", "_")}'
        info = StreamInfo(stream_name, 'EEG', 1, NOMINAL_SRATE, 'float32', f'uid_{stream_name}')
        chns = info.desc().append_child("channels")
        ch = chns.append_child("channel")
        ch.append_child_value("label", node_name)
        ch.append_child_value("type", "EEG")
        outlets[node_name] = StreamOutlet(info)

# ============================================
# Background Thread: Serial Data Acquisition
# ============================================
def serial_reader():
    global ser
    
    print(f"Attempting to connect to ESP32 on {TARGET_PORT}...")
    try:
        ser = serial.Serial(TARGET_PORT, BAUD_RATE, timeout=1)
        ser.set_buffer_size(rx_size=256000) 
        print(f"Successfully connected to {TARGET_PORT}!")
    except serial.SerialException as e:
        print(f"\nError: Could not connect to {TARGET_PORT}.")
        return

    print("Listening for Bluetooth devices...")

    try:
        while True:
            raw_line = ser.readline()
            if not raw_line: 
                continue 
                
            line = raw_line.decode('utf-8', errors='ignore').strip()
            if not line:
                continue
                
            if line.startswith("[DATA]"):
                clean = line[7:].strip()
                
                # Copy keys outside lock to prevent contention
                known_nodes = sorted(list(drift_models.keys()), key=len, reverse=True)
                
                matched_node = None
                for node in known_nodes:
                    if clean.startswith(node + " "):
                        matched_node = node
                        break
                        
                if matched_node:
                    rest = clean[len(matched_node):].strip()
                    if not rest: continue
                        
                    vals = rest.split()
                    try:
                        seq_counter = int(vals[0])
                        eeg_data = [float(v) for v in vals[1:]]
                        
                        dropped_packets = 0
                        if matched_node in last_seq_trackers:
                            expected_seq = (last_seq_trackers[matched_node] + 1) % 256
                            if seq_counter != expected_seq:
                                dropped_packets = (seq_counter - expected_seq) % 256
                                padding = [np.nan] * (dropped_packets * 9)
                                eeg_data = padding + eeg_data
                                
                                # Throttled Console Print (max 1 per second per node)
                                current_time = time.time()
                                if current_time - last_print_times[matched_node] > 1.0:
                                    print(f"[WARNING] {matched_node} dropped {dropped_packets} packets! Padding timeline.")
                                    last_print_times[matched_node] = current_time
                                
                        last_seq_trackers[matched_node] = seq_counter
                        
                        if matched_node not in virtual_t_nodes:
                            target_t = (local_clock() - pc_offset_tracker.get_offset()) * 1000000.0
                            virtual_t_nodes[matched_node] = (target_t - drift_models[matched_node].b) / drift_models[matched_node].m

                        plot_values = []
                        for sample_val in eeg_data:
                            plot_value = sample_val
                            #if not np.isnan(plot_value):
                            #    if "DEMO NODE 4" in matched_node: plot_value /= 5.0
                            #    if "NODE 3" in matched_node: plot_value /= 2.0
                            plot_values.append(plot_value)
                            
                        t_esp_us = drift_models[matched_node].translate(virtual_t_nodes[matched_node])
                        t_pc_lsl_base = (t_esp_us / 1000000.0) + pc_offset_tracker.get_offset()

                        virtual_t_nodes[matched_node] += (len(eeg_data) * 5000)

                        # SCALED DOWN LOCK: Only lock for the actual GUI append operation
                        with data_lock:
                            node_data[matched_node].extend(plot_values)
                            
                        # Push to LSL as a single chunk to save CPU cycles
                        # LSL expects a list of lists for chunks (e.g., [[val1], [val2], [val3]])
                        chunk = [[val] for val in plot_values]
                        
                        # Calculate the timestamp of the LAST sample in the chunk
                        t_last_sample = t_pc_lsl_base + ((len(plot_values) - 1) * 0.005)
                        
                        # Push the chunk. LSL will automatically interpolate the timestamps 
                        # backwards based on the stream's nominal sample rate.
                        outlets[matched_node].push_chunk(chunk, t_last_sample)

                    except ValueError:
                        pass

            elif line.startswith("[SYNC]"):
                sync_match = re.match(r'\[SYNC\]\s+(.+?)\s+T1=(\d+)\s+T2=(\d+)\s+Tnode=(\d+)', line)
                if sync_match:
                    node_name = sync_match.group(1).strip()
                    t1_esp = int(sync_match.group(2))
                    t2_esp = int(sync_match.group(3))
                    t_node = int(sync_match.group(4))
                    
                    register_new_node(node_name)
                    drift_models[node_name].add_sync(t1_esp, t2_esp, t_node)
                    pc_offset_tracker.update(t2_esp, local_clock())

            else:
                print(f"[Serial Monitor] {line}")

    except Exception as e:
        print(f"Serial stream error: {e}")

# ============================================
# Main GUI Thread: PyQtGraph Rendering & Controls
# ============================================
class RealTimeEEGViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Time EEG Viewer (Dynamic LSL Sync Active)")
        self.resize(1000, 600)
        
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)
        
        self.btn_layout = QtWidgets.QHBoxLayout()
        
        self.btn_connect = QtWidgets.QPushButton("Connect Nodes (c)")
        self.btn_connect.setMinimumHeight(40)
        self.btn_connect.clicked.connect(lambda: self.send_command('c'))
        self.btn_layout.addWidget(self.btn_connect)
        
        self.btn_start = QtWidgets.QPushButton("Start Sampling (a)")
        self.btn_start.setMinimumHeight(40)
        self.btn_start.clicked.connect(lambda: self.send_command('a'))
        self.btn_layout.addWidget(self.btn_start)
        
        self.btn_stop = QtWidgets.QPushButton("Stop Sampling (s)")
        self.btn_stop.setMinimumHeight(40)
        self.btn_stop.clicked.connect(lambda: self.send_command('s'))
        self.btn_layout.addWidget(self.btn_stop)
        
        self.layout.addLayout(self.btn_layout)
        
        pg.setConfigOptions(antialias=True)
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground('w')
        self.graph_widget.setTitle("Live EEG (Dynamically Assigned)", color="k")
        self.graph_widget.setLabel('left', 'Amplitude')
        self.graph_widget.setLabel('bottom', 'Sample Index')
        self.graph_widget.showGrid(x=True, y=True, alpha=0.3)
        self.graph_widget.addLegend()
        self.graph_widget.enableAutoRange(axis='y', enable=True)
        self.graph_widget.setAutoVisible(y=True)
        
        self.layout.addWidget(self.graph_widget)
        
        self.curves = {}  
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100) 
        
    def send_command(self, cmd_char):
        global ser
        if ser and ser.is_open:
            
            # Dump any stale data sitting in the OS buffer before starting
            if cmd_char == 'a':
                ser.reset_input_buffer() 
                
            # Send hardware command
            ser.write(cmd_char.encode('utf-8'))
            print(f"Sent command: {cmd_char}")
            
            # Offload LabRecorder automation to background thread
            threading.Thread(target=labrecorder_worker, args=(cmd_char,), daemon=True).start()
        else:
            print("Cannot send command. Serial port is not connected.")

    def update_plots(self):
        # 1. Grab a lightning-fast snapshot of the data and RELEASE THE LOCK
        plot_copies = {}
        with data_lock:
            for node_name, data_deque in node_data.items():
                plot_copies[node_name] = list(data_deque)
                
        # 2. Render the graphics completely OUTSIDE the lock
        for node_name, data_array in plot_copies.items():
            if node_name not in self.curves:
                color = self.colors[len(self.curves) % len(self.colors)]
                self.curves[node_name] = self.graph_widget.plot(
                    data_array, 
                    name=node_name, 
                    pen=pg.mkPen(color=color, width=2)
                )
            else:
                self.curves[node_name].setData(data_array)

if __name__ == '__main__':
    app = pg.mkQApp("RealTimeEEGViewer")
    thread = threading.Thread(target=serial_reader, daemon=True)
    thread.start()
    window = RealTimeEEGViewer()
    window.show()
    sys.exit(pg.exec())