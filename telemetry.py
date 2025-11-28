import socket
import time
import msgpack
import threading
import os
import psutil


def get_system_metrics():
    # CPU %
    cpu = psutil.cpu_percent(interval=None)

    # RAM %
    mem = psutil.virtual_memory().percent

    # CPU temp
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            temp = int(f.read()) / 1000.0
    except:
        temp = None

    # WiFi RSSI
    try:
        with open("/proc/net/wireless") as f:
            lines = f.readlines()
            if len(lines) >= 3:
                parts = lines[2].split()
                wifi = float(parts[3])  # dBm
            else:
                wifi = None
    except:
        wifi = None

    return {
        "cpu_percent": cpu,
        "mem_percent": mem,
        "cpu_temp_c": temp,
        "wifi_signal_dbm": wifi,
    }


class Telemetry:
    """
    Frame-based telemetry daemon.

    - You call .push(name, dict) or .push_raw(dict) from anywhere.
    - It accumulates data for the *next frame* only.
    - At its own rate (rate_hz), it builds one MsgPack packet and sends it.
    - After send, the internal frame dict is CLEARED.
    - So if you stop pushing some namespace (e.g. old controller), it disappears
      from telemetry automatically on the next frame.
    """

    def __init__(self, rate_hz=30, ip="255.255.255.255", port=9870):
        self.rate_hz = rate_hz
        self.period = 1.0 / rate_hz
        self.system_metrics_sample_period = 1.0 # seconds
        self.addr = (ip, port)

        self.enabled = False
        self.sock = None

        # Data for the *next* frame only
        self._frame = {}             # dict to be sent next
        self._lock = threading.Lock()
        self._running = True

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ----------------------------------------------------------------------
    def push(self, name: str, data: dict):
        """
        Push a dictionary under a namespace for the *next frame*.

        Example:
            telemetry.push("outputs", outputs_dict)
            telemetry.push("controller_manual", ctrl_state_dict)
        """
        if not isinstance(data, dict):
            return

        with self._lock:
            # Shallow copy to avoid caller mutating after push
            self._frame[name] = data.copy()

    # ----------------------------------------------------------------------
    def push_raw(self, root_dict: dict):
        """
        Merge a raw dict into the frame root.

        Example:
            telemetry.push_raw({"rpi": {...}, "debug": {...}})
        """
        if not isinstance(root_dict, dict):
            return

        with self._lock:
            # shallow merge into frame root
            for k, v in root_dict.items():
                self._frame[k] = v

    # ----------------------------------------------------------------------
    def _network_ok(self):
        # crude but cheap: wlan0 exists
        return os.path.isdir("/sys/class/net/wlan0")

    # ----------------------------------------------------------------------
    def _try_enable_socket(self):
        if not self._network_ok():
            return False

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.enabled = True
            return True
        except Exception:
            self.enabled = False
            return False

    # ----------------------------------------------------------------------
    def _loop(self):
        last_retry = 0.0
        last_system_metrics_sample = 0.0

        while self._running:
            now = time.time()

            # Try enabling socket if disabled
            if not self.enabled:
                if now - last_retry > 2.0:
                    last_retry = now
                    self._try_enable_socket()
                time.sleep(0.05)
                continue

            # Rate control
            time.sleep(self.period)
            now = time.time()

            # Take current frame and CLEAR it for the next period
            with self._lock:
                if not self._frame:
                    # nothing to send this frame
                    continue
                frame = self._frame
                self._frame = {}

            # Obtain system metrics once per some time
            if now - last_system_metrics_sample > self.system_metrics_sample_period:
                frame["system_metrics"] = get_system_metrics()
                last_system_metrics_sample = now

            # Build, pack, and send packet
            packet = {
                "timestamp": now,
                "brzanpi": frame,
            }
            try:
                bin_packet = msgpack.packb(packet, use_bin_type=True)
                self.sock.sendto(bin_packet, self.addr)
            except Exception:
                # network down → disable and retry later
                self.enabled = False
                try:
                    self.sock.close()
                except Exception:
                    pass

    # ----------------------------------------------------------------------
    def stop(self):
        self._running = False
        try:
            self.sock.close()
        except Exception:
            pass
