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

        # --- Controller metrics accumulators ---
        self._rt_loop_sum = 0.0
        self._rt_loop_count = 0
        self._rt_ctrl_sum = 0.0
        self._rt_ctrl_count = 0
        self._rt_overruns = 0
        self._rt_jitter_sum = 0.0
        self._rt_jitter_sq_sum = 0.0
        self._rt_jitter_count = 0
        self._rt_loop_max = 0.0
        self._rt_ctrl_max = 0.0
        self._rt_missed_cycles = 0
        self._rt_ctrl_iterations = 0
        # ---------------------------------------

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

            with self._lock:
                # Always create a frame, even if empty
                frame = self._frame if self._frame else {}
                self._frame = {}
            # # Take current frame and CLEAR it for the next period
            # with self._lock:
            #     if not self._frame:
            #         # nothing to send this frame
            #         continue
            #     frame = self._frame
            #     self._frame = {}

            # Obtain system metrics once per some time and put into frame
            if now - last_system_metrics_sample > self.system_metrics_sample_period:
                frame["system_metrics"] = get_system_metrics()
                last_system_metrics_sample = now

            # Extract and reset accumulated run_controller metrics
            controller_metrics = {}
            with self._lock:
                if self._rt_loop_count > 0:
                    controller_metrics["loop_time_avg_ms"] = (self._rt_loop_sum / self._rt_loop_count) * 1000
                if self._rt_ctrl_count > 0:
                    controller_metrics["ctrl_time_avg_us"] = (self._rt_ctrl_sum / self._rt_ctrl_count) * 1e6
                if self._rt_jitter_count > 0:
                    mean = self._rt_jitter_sum / self._rt_jitter_count
                    mean_sq = self._rt_jitter_sq_sum / self._rt_jitter_count
                    rms = (mean_sq - mean * mean) ** 0.5
                    controller_metrics["jitter_rms_us"] = rms * 1e6
                controller_metrics["max_loop_time_ms"] = self._rt_loop_max * 1000
                controller_metrics["max_ctrl_time_us"] = self._rt_ctrl_max * 1e6
                controller_metrics["missed_cycles"] = self._rt_missed_cycles
                controller_metrics["ctrl_iterations"] = self._rt_ctrl_iterations
                controller_metrics["loop_overruns"] = self._rt_overruns
                # Reset all accumulators
                self._rt_loop_sum = 0.0
                self._rt_loop_count = 0
                self._rt_ctrl_sum = 0.0
                self._rt_ctrl_count = 0
                self._rt_overruns = 0
                self._rt_jitter_sum = 0.0
                self._rt_jitter_sq_sum = 0.0
                self._rt_jitter_count = 0
                self._rt_loop_max = 0.0
                self._rt_ctrl_max = 0.0
                self._rt_missed_cycles = 0
                self._rt_ctrl_iterations = 0

            # Only add if something was measured
            if controller_metrics:
                frame["controller_metrics"] = controller_metrics

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

    # ----------------------------------------------------------------------    
    def accum_rt_loop_time(self, dt: float):
        """Accumulate loop execution time (seconds)."""
        with self._lock:
            self._rt_loop_sum += dt
            self._rt_loop_count += 1

    def accum_rt_ctrl_time(self, dt: float):
        """Accumulate controller execution time (seconds)."""
        with self._lock:
            self._rt_ctrl_sum += dt
            self._rt_ctrl_count += 1

    def flag_rt_overrun(self):
        """Increase count of controller loop overruns."""
        with self._lock:
            self._rt_overruns += 1

    def accum_rt_jitter(self, jitter: float):
        with self._lock:
            self._rt_jitter_sum += jitter
            self._rt_jitter_sq_sum += jitter * jitter
            self._rt_jitter_count += 1

    def accum_rt_loop_max(self, dt: float):
        with self._lock:
            if dt > self._rt_loop_max:
                self._rt_loop_max = dt

    def accum_rt_ctrl_max(self, dt: float):
        with self._lock:
            if dt > self._rt_ctrl_max:
                self._rt_ctrl_max = dt

    def flag_rt_missed_cycle(self, n: int = 1):
        with self._lock:
            self._rt_missed_cycles += n

    def inc_rt_ctrl_iterations(self):
        with self._lock:
            self._rt_ctrl_iterations += 1
