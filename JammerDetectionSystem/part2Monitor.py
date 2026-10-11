import serial
import matplotlib.pyplot as plt
import numpy as np
import time
from collections import deque
import threading
import sys

# --- Configuration ---
# CHANGE THIS TO YOUR ESP32 PORT (e.g., 'COM4' on Windows, '/dev/ttyUSB0' on Linux)
SERIAL_PORT = 'COM3'  
BAUD_RATE = 115200
MAX_POINTS = 100      # Number of points to show on the graph
UPDATE_INTERVAL = 0.1  # Seconds between UI updates
NUM_CHANNELS = 26     # Total channels in the 2.4 GHz band

# --- Data Buffers ---
# Using deques with maxlen to automatically drop old data (sliding window)
time_buffer = deque(maxlen=MAX_POINTS)
pps_buffer = deque(maxlen=MAX_POINTS)
entropy_buffer = deque(maxlen=MAX_POINTS)
occupancy_buffer = deque(maxlen=MAX_POINTS)
bandwidth_buffer = deque(maxlen=MAX_POINTS)
class_buffer = deque(maxlen=MAX_POINTS)

# Spectrum Analyzer Data (Current PPS for each channel)
# We maintain a list of 26 values, updated as the JDCS hops
channel_pps = [0.0] * NUM_CHANNELS

# Class Labels for the Legend
CLASS_NAMES = {
    0: "NORMAL",
    1: "WI-FI JAM",
    2: "BLE JAM",
    3: "FULL BAND",
    4: "PULSE JAM"
}

# --- Matplotlib Setup ---
plt.ion()  # Turn on interactive mode
# 4 Plots: PPS, Entropy, Occupancy/Class, and Spectrum Analyzer
fig, axs = plt.subplots(4, 1, figsize=(12, 12))
fig.suptitle('JDCS Real-Time Spectrum Analyzer', fontsize=14, fontweight='bold')

# Plot 1: PPS (Packets Per Second) - Proxy for Signal Strength
ax1 = axs[0]
ax1.set_title('Packets Per Second (PPS)')
ax1.set_ylim(0, 100)
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.set_ylabel('PPS')
line1, = ax1.plot([], [], 'b-', label='PPS', linewidth=2)
ax1.legend(loc='upper left')

# Plot 2: Spectral Entropy - Measure of Signal Complexity
ax2 = axs[1]
ax2.set_title('Spectral Entropy')
ax2.set_ylim(0, 4)
ax2.grid(True, linestyle='--', alpha=0.6)
ax2.set_ylabel('Entropy')
line2, = ax2.plot([], [], 'g-', label='Entropy', linewidth=2)
ax2.legend(loc='upper left')

# Plot 3: Channel Occupancy & Jammer Class
ax3 = axs[2]
ax3.set_title('Channel Occupancy (%) & Jammer Classification')
ax3.set_ylim(0, 110)
ax3.grid(True, linestyle='--', alpha=0.6)
ax3.set_ylabel('Occupancy %')
line3, = ax3.plot([], [], 'r-', label='Occupancy', linewidth=2)
# Step plot for the class (discrete values)
line4, = ax3.step([], [], 'k--', where='post', label='Class', linewidth=2)
ax3.legend(loc='upper left')

# Plot 4: Spectrum Analyzer (Bar Chart)
ax4 = axs[3]
ax4.set_title('2.4 GHz Spectrum Activity (Per Channel)')
ax4.set_ylim(0, 100)
ax4.set_xlim(0.5, NUM_CHANNELS + 0.5)
ax4.set_xticks(range(1, NUM_CHANNELS + 1))
ax4.set_xticklabels([str(i) for i in range(1, NUM_CHANNELS + 1)], fontsize=8)
ax4.set_ylabel('PPS')
# Create bar containers
bars = ax4.bar(range(1, NUM_CHANNELS + 1), [0.0] * NUM_CHANNELS, color='lightblue', width=0.8)
# Highlight critical Wi-Fi channels (1, 6, 11)
for bar in bars:
    if bar.get_x() in [1, 6, 11]:
        bar.set_color('orange')

# --- Data Parser ---
def parse_line(line):
    """
    Parses the serial line: CH,PPS_MEAN,PPS_VAR,OCCUPANCY,ENTROPY,BANDWIDTH,CLASS
    """
    try:
        parts = line.strip().split(',')
        if len(parts) != 7:
            return None
        return {
            'channel': int(parts[0]),
            'pps_mean': float(parts[1]),
            'pps_var': float(parts[2]),
            'occupancy': float(parts[3]),
            'entropy': float(parts[4]),
            'bandwidth': int(parts[5]),
            'class': int(parts[6])
        }
    except (ValueError, IndexError):
        return None

# --- Update Function ---
def update_plot():
    """
    Updates the matplotlib plots with the latest data.
    """
    if not time_buffer:
        return

    # Extract data from buffers
    times = list(time_buffer)
    pps = list(pps_buffer)
    entropy = list(entropy_buffer)
    occupancy = list(occupancy_buffer)
    classes = list(class_buffer)
    bandwidth = list(bandwidth_buffer)

    # Update Plot 1: PPS
    line1.set_data(times, pps)
    ax1.set_xlim([times[0], times[-1]])

    # Update Plot 2: Entropy
    line2.set_data(times, entropy)
    ax2.set_xlim([times[0], times[-1]])

    # Update Plot 3: Occupancy & Class
    line3.set_data(times, occupancy)
    line4.set_data(times, classes)
    ax3.set_xlim([times[0], times[-1]])

    # Update Plot 4: Spectrum Analyzer (Bar Heights)
    # The channel_pps list is updated by the serial_reader thread
    # We just need to update the heights of the existing bars
    for i, bar in enumerate(bars):
        bar.set_height(channel_pps[i])

    # Update Title with Current Classification & Bandwidth
    current_class = classes[-1] if classes else 0
    current_name = CLASS_NAMES.get(current_class, 'UNKNOWN')
    current_bw = bandwidth[-1] if bandwidth else 0
    
    # Color code the title based on the threat level
    if current_class == 0:
        color = 'green'
    elif current_class in [1, 2]:
        color = 'orange'
    else:
        color = 'red'
        
    fig.suptitle(f'JDCS Real-Time Analyzer | Status: {current_name} | Bandwidth: {current_bw} Ch', 
                 fontsize=14, fontweight='bold', color=color)

    plt.pause(UPDATE_INTERVAL)

# --- Serial Reader Thread ---
def serial_reader():
    """
    Reads data from the serial port in a separate thread.
    """
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"[INFO] Connected to {SERIAL_PORT}")
        
        while True:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
                
            # Ignore initialization messages
            if line.startswith('[JDCS]') or line.startswith('Format'):
                print(f"[ESP32] {line}")
                continue

            data = parse_line(line)
            if data:
                current_time = time.time()
                
                # Update buffers
                time_buffer.append(current_time)
                pps_buffer.append(data['pps_mean'])
                entropy_buffer.append(data['entropy'])
                occupancy_buffer.append(data['occupancy'])
                bandwidth_buffer.append(data['bandwidth'])
                class_buffer.append(data['class'])
                
                # Update Spectrum Analyzer Data
                # channel_pps is 0-indexed, channels are 1-indexed
                ch_idx = data['channel'] - 1
                if 0 <= ch_idx < NUM_CHANNELS:
                    channel_pps[ch_idx] = data['pps_mean']
                
                # Print to console for debugging
                class_name = CLASS_NAMES.get(data['class'], 'UNKNOWN')
                print(f"CH:{data['channel']:>2} | PPS:{data['pps_mean']:>5.1f} | "
                      f"ENT:{data['entropy']:>4.2f} | OCC:{data['occupancy']:>5.1f}% | "
                      f"BW:{data['bandwidth']:>2} | CLASS: {class_name:<10}")

    except serial.SerialException as e:
        print(f"[ERROR] Serial connection failed: {e}")
        print(f"[ERROR] Ensure {SERIAL_PORT} is correct and not in use by another program.")
    finally:
        ser.close()

# --- Main Execution ---
if __name__ == "__main__":
    # Start Serial Reader in a separate thread
    # This thread will continuously read data from the ESP32 and update
    # the shared buffers (time_buffer, pps_buffer, channel_pps, etc.)
    serial_thread = threading.Thread(target=serial_reader, daemon=True)
    serial_thread.start()
    
    print("[INFO] Dashboard Started. Press Ctrl+C to exit.")
    print(f"[INFO] Monitoring Port: {SERIAL_PORT}")
    print(f"[INFO] Tracking {NUM_CHANNELS} Channels in 2.4 GHz Band")
    
    try:
        # Infinite loop to keep the Matplotlib window open and updating
        while True:
            update_plot()
            time.sleep(0.01)  # Small sleep to prevent CPU spike
    except KeyboardInterrupt:
        print("\n[INFO] Dashboard Stopped.")
        plt.ioff()
        plt.show()