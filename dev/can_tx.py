#!/usr/bin/env python3

import time

import can
import cantools

import config


class CANTransmitter:
    def __init__(self):
        self.db = cantools.database.load_file(config.DBC_PATH)

        self.bus = can.interface.Bus(
            channel=config.CAN_CHANNEL,
            interface=config.CAN_INTERFACE,
        )

        # Lifetime statistics.
        self.sent_frames = 0
        self.send_errors = 0
        self.last_send_execution_ns = 0
        self.max_send_execution_ns = 0
        # Statistics since the previous periodic report.
        self._stats_sent_frames = 0
        self._stats_send_errors = 0
        self._stats_send_execution_ns_sum = 0
        self._stats_max_send_execution_ns = 0

    def send_outputs(self, outputs: dict):
        if not outputs:
            return

        for message_name, signals in outputs.items():
            try:
                dbc_msg = self.db.get_message_by_name(message_name)
                data = dbc_msg.encode(signals)

                frame = can.Message(
                    arbitration_id=dbc_msg.frame_id,
                    data=data,
                    is_extended_id=dbc_msg.is_extended_frame,
                )

                start_ns = time.monotonic_ns()
                self.bus.send(frame, timeout=config.CAN_TX_TIMEOUT_S)
                end_ns = time.monotonic_ns()

                self.sent_frames += 1

                self.last_send_execution_ns = end_ns - start_ns
                self.max_send_execution_ns = max(
                    self.max_send_execution_ns,
                    self.last_send_execution_ns,
                )

                self._stats_sent_frames += 1
                self._stats_send_execution_ns_sum += self.last_send_execution_ns
                self._stats_max_send_execution_ns = max(
                    self._stats_max_send_execution_ns,
                    self.last_send_execution_ns,
                )

            except Exception as exc:
                self.send_errors += 1
                self._stats_send_errors += 1
                print(f"[can_tx] Failed to send {message_name}: {exc}")

    def get_status(self) -> dict:
        avg_send_execution_ns = 0.0
        if self._stats_sent_frames:
            avg_send_execution_ns = (
                self._stats_send_execution_ns_sum / self._stats_sent_frames
            )
        return {
            "sent_frames": self.sent_frames,
            "send_errors": self.send_errors,
            "last_send_execution_us": self.last_send_execution_ns / 1e3,
            "max_send_execution_us": self.max_send_execution_ns / 1e3,
            "recent": {
                "sent_frames": self._stats_sent_frames,
                "send_errors": self._stats_send_errors,
                "avg_send_execution_us": avg_send_execution_ns / 1e3,
                "max_send_execution_us": self._stats_max_send_execution_ns / 1e3,
            },
        }

    def reset_status_window(self):
        self._stats_sent_frames = 0
        self._stats_send_errors = 0
        self._stats_send_execution_ns_sum = 0
        self._stats_max_send_execution_ns = 0
