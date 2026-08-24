#!/usr/bin/env python3

import json
import os
import socket
import time
from pathlib import Path
from queue import Empty

import msgpack

import config

class LogWriter:
    def __init__(self):
        self.log_dir = Path(config.LOG_DIR)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.control_socket_path = config.LOGGER_CONTROL_SOCKET

        self._file = None
        self._file_path = None
        self._last_flush_monotonic = 0.0

        self._records_written = 0
        self._bytes_written = 0
        self._pack_errors = 0

        self._control_socket = None

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
            packet = msgpack.packb(
                record,
                use_bin_type=True,
            )
        except Exception as exc:
            self._pack_errors += 1
            print(f"[log_writer] WARN: MsgPack encode failed: {exc}")
            return

        self._file.write(packet)

        self._records_written += 1
        self._bytes_written += len(packet)


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

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.bind(self.control_socket_path)
        sock.listen(1)
        sock.setblocking(False)

        self._control_socket = sock

        print(
            f"[log_writer] Control socket: "
            f"{self.control_socket_path}"
        )

    def _handle_control_socket(self):
        if self._control_socket is None:
            return
        try:
            conn, _ = self._control_socket.accept()
        except BlockingIOError:
            return
        try:
            conn.settimeout(0.5)
            command = conn.recv(1024).decode("utf-8").strip().lower()
            if command == "rotate":
                self._rotate()
                response = {
                    "ok": True,
                    "command": "rotate",
                    "file": str(self._file_path),
                }
            elif command == "status":
                response = self.get_status()
            else:
                response = {
                    "ok": False,
                    "error": f"unknown command: {command}",
                }
            conn.sendall(
                (json.dumps(response) + "\n").encode("utf-8")
            )

        except Exception as exc:
            print(f"[log_writer] Control socket error: {exc}")

        finally:
            conn.close()


    def get_status(self):
        return {
            "ok": True,
            "file": str(self._file_path) if self._file_path else None,
            "records_written": self._records_written,
            "bytes_written": self._bytes_written,
            "pack_errors": self._pack_errors,
        }

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
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[log_writer] FATAL: {exc}")
        raise
