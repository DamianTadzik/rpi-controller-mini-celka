import threading
import queue
import os
import time
import pyarrow
import pyarrow.parquet


ALLOWED_SIGNALS = {
    # observer
    "x_hat.z_m",
    "x_hat.z_dot_mps",
    #"x_hat.phi_rad",
    #"x_hat.theta_rad",
    #"x_hat.psi_rad",
    #"x_hat.p_radps",
    #"x_hat.q_radps",
    #"x_hat.r_radps",

    # controller metrics
    #"controller_metrics.max_loop_time_ms",
    #"controller_metrics.max_ctrl_time_us",
    #"controller_metrics.missed_cycles",
    #"controller_metrics.ctrl_iterations",
    #"controller_metrics.loop_overruns",

    # observer metrics
    #"observer_metrics.obs_time_avg_us",
    #"observer_metrics.jitter_rms_us",
    #"observer_metrics.max_obs_time_us",
    #"observer_metrics.missed_cycles",
    #"observer_metrics.obs_iterations",

    # system
    #"system_metrics.cpu_percent",
    #"system_metrics.mem_percent",
    #"system_metrics.cpu_temp_c",
    #"system_metrics.wifi_signal_dbm",
}


class ParquetLogger:
    def __init__(self, path="logs/", flush_interval_s=60, flush_lines=1000):
        self.path = path
        self.flush_interval_s = flush_interval_s
        self.flush_lines = flush_lines
        self._writer = None

        # Check the logs directory exists at the initialization and create it if not
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        
        # check largest logXXX.parquet file index to avoid overwriting and create logXXX+1.parquet
        existing_logs = [f for f in os.listdir(self.path) if f.startswith("log") and f.endswith(".parquet")]
        max_index = -1
        for log_file in existing_logs:
            try:
                index = int(log_file[3:-8])  # Extract the number between 'log' and '.parquet'
                if index > max_index:
                    max_index = index
            except ValueError:
                continue

        self.log_file_path = os.path.join(self.path, f"log{max_index + 1}.parquet")
        self.q = queue.SimpleQueue()

        self._schema = pyarrow.schema(
            [("timestamp", pyarrow.float64())] +
            [(name, pyarrow.float64()) for name in sorted(ALLOWED_SIGNALS)]
        )

        # Start the logging thread
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # called by telemetry
    def push(self, packet: dict):
        self.q.put_nowait(packet)

    def _loop(self):
        last_flush_time = time.monotonic()
        lines = []

        while self.running:
            try:
                packet = self.q.get(timeout=1)
                lines.append(packet)
            except queue.Empty:
                pass

            if lines and (time.monotonic() - last_flush_time >= self.flush_interval_s or len(lines) >= self.flush_lines):
                self._flush(lines)
                lines = []
                last_flush_time = time.monotonic()
    
    def _flush(self, packets):
        if not packets:
            return

        rows = []

        for pkt in packets:
            row = {}

            # timestamp for PlotJuggler
            if "timestamp" in pkt:
                row["timestamp"] = pkt["timestamp"]

            for group, data in pkt.get("brzanpi", {}).items():
                if not isinstance(data, dict):
                    continue

                for k, v in data.items():
                    name = f"{group}.{k}"
                    if name in ALLOWED_SIGNALS and isinstance(v, (int, float)):
                        row[name] = v

            rows.append(row)

        if not rows:
            return
        
        table = pyarrow.Table.from_pylist(rows, schema=self._schema)

        if self._writer is None:
            self._writer = pyarrow.parquet.ParquetWriter(
                self.log_file_path,
                self._schema,
                compression="snappy"
            )

        self._writer.write_table(table)
    
    def stop(self):
        self.running = False
        self.thread.join()

        if self._writer is not None:
            self._writer.close()
            self._writer = None
