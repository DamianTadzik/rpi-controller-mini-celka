#!/usr/bin/env python3

import json
import os
import socket
import time
from pathlib import Path
from queue import Empty

import msgpack

import config

from log_format import build_header

class LogWriter:
    def __init__(self):
        self.log_dir = Path(config.LOG_DIR)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.control_socket_path = config.LOGGER_CONTROL_SOCKET
        self._control_socket = None

        self._file = None
        self._file_path = None
        self._last_flush_monotonic = 0.0

        # Per-file counters.
        self._records_written = 0
        self._bytes_written = 0
        self._pack_errors = 0
        # Statistics since the previous periodic report.
        self._stats_records_written = 0
        self._stats_bytes_written = 0
        self._stats_pack_errors = 0
        self._last_status_print = time.monotonic()


    # -------------------------------------------------------------------------
    # File handling
    # -------------------------------------------------------------------------
    def _find_next_log_index(self) -> int:
        max_index = -1

        for path in self.log_dir.glob("log*.msgpack"):
            name = path.stem  # e.g. "log12"

            if not name.startswith("log"):
                continue

            try:
                index = int(name[3:])
            except (ValueError, IndexError):
                continue

            max_index = max(max_index, index)

        return max_index + 1

    def _open_new_file(self):
        index = self._find_next_log_index()
        self._file_path = self.log_dir / f"log{index}.msgpack"
        # Large userspace buffer. Actual writes to the filesystem can therefore
        # be grouped instead of issuing tiny writes for each record.
        self._file = open(
            self._file_path,
            "ab",
            buffering=1024 * 1024,
        )
        self._last_flush_monotonic = time.monotonic()
        self._records_written = 0
        self._bytes_written = 0
        self._pack_errors = 0

        # Write the log_format header.
        packet = msgpack.packb(build_header(), use_bin_type=True)
        self._file.write(packet)
        self._bytes_written += len(packet)

        print(f"[log_writer] Opened {self._file_path}")

    def _flush(self):
        if self._file is None:
            return
        self._file.flush()
        self._last_flush_monotonic = time.monotonic()

    def _close_file(self):
        if self._file is None:
            return
        self._flush()
        self._file.close()
        print(f"[log_writer] Closed {self._file_path}")
        self._file = None
        self._file_path = None

    def _rotate(self):
        self._close_file()
        self._open_new_file()

    # -------------------------------------------------------------------------
    # Record writing
    # -------------------------------------------------------------------------
    def write_record(self, record):
        try:
            packet = msgpack.packb(record, use_bin_type=True)
        except Exception as exc:
            self._pack_errors += 1
            self._stats_pack_errors += 1
            print(f"[log_writer] WARN: MsgPack encode failed: {exc}")
            return

        self._file.write(packet)
        packet_size = len(packet)

        self._records_written += 1
        self._bytes_written += packet_size
        self._stats_records_written += 1
        self._stats_bytes_written += packet_size


    def _periodic_flush(self):
        now = time.monotonic()
        if now - self._last_flush_monotonic >= config.LOG_FLUSH_PERIOD_S:
            self._flush()

    # -------------------------------------------------------------------------
    # Local control socket
    # -------------------------------------------------------------------------
    def _setup_control_socket(self):
        try:
            os.unlink(self.control_socket_path)
        except FileNotFoundError:
            pass

        self._control_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._control_socket.bind(self.control_socket_path)
        self._control_socket.setblocking(False)
        print(f"[log_writer] Rotation socket: {self.control_socket_path}")

    def _handle_control_socket(self):
        try:
            command = self._control_socket.recv(64)
        except BlockingIOError:
            return

        if command.strip().lower() == b"rotate":
            self._rotate()

    def get_status(self) -> dict:
        elapsed_s = time.monotonic() - self._last_status_print

        records_per_s = (
            self._stats_records_written / elapsed_s
            if elapsed_s > 0 else 0.0
        )
        write_mib_per_s = (
            self._stats_bytes_written / elapsed_s / (1024 * 1024)
            if elapsed_s > 0 else 0.0
        )
        return {
            "module": "log_writer",
            "file": str(self._file_path) if self._file_path else None,
            "records_written": self._records_written,
            "bytes_written": self._bytes_written,
            "file_size_mib": self._bytes_written / (1024 * 1024),
            "pack_errors": self._pack_errors,
            "recent": {
                "period_s": elapsed_s,
                "records_written": self._stats_records_written,
                "records_per_s": records_per_s,
                "bytes_written": self._stats_bytes_written,
                "write_mib_per_s": write_mib_per_s,
                "pack_errors": self._stats_pack_errors,
            },
        }

    def _print_status_if_due(self):
        now = time.monotonic()
        if now - self._last_status_print < config.STATS_PRINT_PERIOD_S:
            return
        print(json.dumps(self.get_status(), indent=2))
        self._stats_records_written = 0
        self._stats_bytes_written = 0
        self._stats_pack_errors = 0
        self._last_status_print = now

    def start(self):
        self._open_new_file()
        self._setup_control_socket()


# =============================================================================
# Process entry point
# =============================================================================

def run_log_writer(log_queue):
    """
    Process entry point.

    The writer is the only process that accesses the log file. Producers only
    place Python records into log_queue.
    """
    writer = LogWriter()
    try:
        writer.start()
        while True:
            writer._handle_control_socket()
            try:
                record = log_queue.get(timeout=0.05)
                writer.write_record(record)
            except Empty:
                pass
            writer._periodic_flush()
            writer._print_status_if_due()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[log_writer] FATAL: {exc}")
        raise
