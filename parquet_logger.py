import threading
import queue
import os
import time
import pyarrow
import pyarrow.parquet


class ParquetLogger:
    def __init__(self, path="logs/", flush_interval_s=60, flush_lines=1000):
        self.path = path
        self.flush_interval_s = flush_interval_s
        self.flush_lines = flush_lines

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

        table = pyarrow.Table.from_pylist(packets)

        pyarrow.parquet.write_table(
            table,
            self.log_file_path,
            compression="snappy"
        )

    def stop(self):
        self.running = False
        self.thread.join()
        self._flush()
        
