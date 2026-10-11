"""
BLE Jammer Monitor — monitors jammer activity over serial.
Usage: python monitor.py [COM_PORT]
       python monitor.py          (auto-detects ESP32)
       python monitor.py COM5     (manual port)

Requires: pip install pyserial
"""

import sys
import time
import threading
import re
import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports


def find_esp32_port():
    """Auto-detect ESP32 COM port."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        # Common ESP32 USB-serial chips: CP210x, CH340, FTDI
        if any(chip in desc for chip in ["cp210", "ch340", "ch910", "ftdi", "usb-serial", "uart"]):
            return p.device
        if any(vid in hwid for vid in ["10c4", "1a86", "0403"]):  # SiLabs, WCH, FTDI vendor IDs
            return p.device
    # Fallback: return first available port
    if ports:
        return ports[0].device
    return None


class JammerMonitor:
    def __init__(self, port=None):
        self.port = port or find_esp32_port()
        self.ser = None
        self.running = False

        # State
        self.connected = False
        self.jammer_live = False
        self.radio1_ok = False
        self.radio2_ok = False
        self.adv_ch = "-"
        self.data_ch = "-"
        self.speed = "0"
        self.last_telemetry = 0
        self.log_lines = []

        self.build_ui()

    def build_ui(self):
        self.root = tk.Tk()
        self.root.title("BLE Jammer Monitor")
        self.root.geometry("480x400")
        self.root.configure(bg="#1a1a2e")
        self.root.resizable(False, False)

        # Title
        tk.Label(self.root, text="BLE JAMMER MONITOR", font=("Consolas", 16, "bold"),
                 fg="#e0e0e0", bg="#1a1a2e").pack(pady=(10, 5))

        # Status frame
        status_frame = tk.Frame(self.root, bg="#16213e", bd=1, relief="solid")
        status_frame.pack(fill="x", padx=15, pady=5)

        # Connection status
        conn_row = tk.Frame(status_frame, bg="#16213e")
        conn_row.pack(fill="x", padx=10, pady=(8, 2))
        tk.Label(conn_row, text="PORT:", font=("Consolas", 10),
                 fg="#888", bg="#16213e").pack(side="left")
        self.lbl_port = tk.Label(conn_row, text=self.port or "None",
                                  font=("Consolas", 10), fg="#e0e0e0", bg="#16213e")
        self.lbl_port.pack(side="left", padx=(5, 0))

        # Jammer status (big indicator)
        self.lbl_status = tk.Label(status_frame, text="OFFLINE", font=("Consolas", 28, "bold"),
                                    fg="#ff4444", bg="#16213e")
        self.lbl_status.pack(pady=5)

        # Radio status row
        radio_row = tk.Frame(status_frame, bg="#16213e")
        radio_row.pack(fill="x", padx=10, pady=(0, 4))
        tk.Label(radio_row, text="Radio 1:", font=("Consolas", 10),
                 fg="#888", bg="#16213e").pack(side="left")
        self.lbl_r1 = tk.Label(radio_row, text="--", font=("Consolas", 10, "bold"),
                                fg="#888", bg="#16213e")
        self.lbl_r1.pack(side="left", padx=(3, 15))
        tk.Label(radio_row, text="Radio 2:", font=("Consolas", 10),
                 fg="#888", bg="#16213e").pack(side="left")
        self.lbl_r2 = tk.Label(radio_row, text="--", font=("Consolas", 10, "bold"),
                                fg="#888", bg="#16213e")
        self.lbl_r2.pack(side="left", padx=(3, 0))

        # Telemetry row
        tele_row = tk.Frame(status_frame, bg="#16213e")
        tele_row.pack(fill="x", padx=10, pady=(2, 8))
        tk.Label(tele_row, text="ADV:", font=("Consolas", 10),
                 fg="#888", bg="#16213e").pack(side="left")
        self.lbl_adv = tk.Label(tele_row, text="-", font=("Consolas", 10, "bold"),
                                 fg="#4fc3f7", bg="#16213e")
        self.lbl_adv.pack(side="left", padx=(3, 12))
        tk.Label(tele_row, text="DATA:", font=("Consolas", 10),
                 fg="#888", bg="#16213e").pack(side="left")
        self.lbl_data = tk.Label(tele_row, text="-", font=("Consolas", 10, "bold"),
                                  fg="#4fc3f7", bg="#16213e")
        self.lbl_data.pack(side="left", padx=(3, 12))
        tk.Label(tele_row, text="SPEED:", font=("Consolas", 10),
                 fg="#888", bg="#16213e").pack(side="left")
        self.lbl_speed = tk.Label(tele_row, text="0 loops/s", font=("Consolas", 10, "bold"),
                                   fg="#4fc3f7", bg="#16213e")
        self.lbl_speed.pack(side="left", padx=(3, 0))

        # Log area
        tk.Label(self.root, text="Serial Log", font=("Consolas", 10),
                 fg="#888", bg="#1a1a2e").pack(anchor="w", padx=15, pady=(8, 2))
        self.log_text = tk.Text(self.root, height=10, font=("Consolas", 9),
                                 bg="#0f0f23", fg="#cccccc", bd=1, relief="solid",
                                 insertbackground="#cccccc", state="disabled")
        self.log_text.pack(fill="both", padx=15, pady=(0, 10), expand=True)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def log(self, text):
        """Add a line to the log area."""
        self.log_lines.append(text)
        if len(self.log_lines) > 200:
            self.log_lines = self.log_lines[-200:]
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def parse_line(self, line):
        """Parse a serial line and update state."""
        # Radio init results
        if "HSPI Radio 1: OK" in line:
            self.radio1_ok = True
        elif "HSPI Radio 1: FAILED" in line:
            self.radio1_ok = False
        if "VSPI Radio 2: OK" in line:
            self.radio2_ok = True
        elif "VSPI Radio 2: FAILED" in line:
            self.radio2_ok = False

        # Jamming active
        if "JAMMING ACTIVE" in line:
            self.jammer_live = True
            self.radio1_ok = True
            self.radio2_ok = True

        # Telemetry: "ADV Ch:2 | DATA Ch:42 | Speed: 51234 loops/s"
        m = re.search(r"ADV Ch:(\d+)\s*\|\s*DATA Ch:(\d+)\s*\|\s*Speed:\s*(\d+)", line)
        if m:
            self.adv_ch = m.group(1)
            self.data_ch = m.group(2)
            self.speed = m.group(3)
            self.last_telemetry = time.time()
            self.jammer_live = True
            # If we're getting telemetry, both radios must be working
            # (we may have missed the boot messages)
            self.radio1_ok = True
            self.radio2_ok = True

    def update_ui(self):
        """Update UI labels from current state."""
        # Check if telemetry is stale (no update in 5 seconds = probably dead)
        if self.last_telemetry > 0 and (time.time() - self.last_telemetry) > 5:
            self.jammer_live = False

        # Jammer status
        if not self.connected:
            self.lbl_status.configure(text="DISCONNECTED", fg="#ff4444")
        elif self.jammer_live:
            self.lbl_status.configure(text="● LIVE", fg="#00e676")
        else:
            self.lbl_status.configure(text="OFFLINE", fg="#ff4444")

        # Radio status
        self.lbl_r1.configure(text="OK" if self.radio1_ok else "FAIL",
                               fg="#00e676" if self.radio1_ok else "#ff4444")
        self.lbl_r2.configure(text="OK" if self.radio2_ok else "FAIL",
                               fg="#00e676" if self.radio2_ok else "#ff4444")

        # Telemetry
        self.lbl_adv.configure(text=str(self.adv_ch))
        self.lbl_data.configure(text=str(self.data_ch))
        self.lbl_speed.configure(text=f"{self.speed} loops/s")

        self.lbl_port.configure(text=self.port or "None",
                                 fg="#00e676" if self.connected else "#ff4444")

        if self.running:
            self.root.after(500, self.update_ui)

    def serial_thread(self):
        """Background thread: connect to serial and read lines."""
        while self.running:
            # Try to connect
            if not self.ser or not self.ser.is_open:
                self.connected = False
                self.jammer_live = False
                # Re-detect port if needed
                if not self.port:
                    self.port = find_esp32_port()
                if not self.port:
                    time.sleep(2)
                    continue
                try:
                    self.ser = serial.Serial(self.port, 115200, timeout=1)
                    self.connected = True
                    self.root.after(0, lambda: self.log(f"Connected to {self.port}"))
                except serial.SerialException:
                    self.ser = None
                    time.sleep(2)
                    continue

            # Read lines
            try:
                raw = self.ser.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        self.parse_line(line)
                        self.root.after(0, lambda l=line: self.log(l))
            except (serial.SerialException, OSError):
                self.connected = False
                self.jammer_live = False
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = None
                self.root.after(0, lambda: self.log("Serial disconnected — reconnecting..."))
                time.sleep(2)

    def start(self):
        self.running = True
        t = threading.Thread(target=self.serial_thread, daemon=True)
        t.start()
        self.update_ui()
        self.root.mainloop()

    def on_close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.root.destroy()


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else None
    app = JammerMonitor(port)
    app.start()
