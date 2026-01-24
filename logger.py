import os
import msgpack
import threading

class Logger:
    def __init__(self, path="logs/", max_bytes=100*1024*1024, max_seconds=60*60*4):
        os.makedirs(path, exist_ok=True)

        self.max_bytes = max_bytes
        self.max_seconds = max_seconds

        # find next log index
        existing = [
            f for f in os.listdir(path)
            if f.startswith("log") and f.endswith(".msgpack")
        ]

        max_index = -1
        for f in existing:
            try:
                max_index = max(max_index, int(f[3:-8]))
            except ValueError:
                pass

        self.log_file_path = os.path.join(path, f"log{max_index + 1}.msgpack")

        # open file once, append-only
        self._file = open(self.log_file_path, "ab")
        self._lock = threading.Lock()   

    def push(self, bin_packet: bytes):
        # atomic enough for that use-case
        with self._lock:
            self._file.write(bin_packet)
            self._file.flush() 

    def stop(self):
        with self._lock:
            self._file.flush()
            self._file.close()
