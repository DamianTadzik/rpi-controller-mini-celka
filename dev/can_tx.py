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

        # Lightweight runtime statistics.
        self.sent_frames = 0
        self.send_errors = 0
        self.last_send_execution_ns = 0
        self.max_send_execution_ns = 0

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

                self.bus.send(frame)

                end_ns = time.monotonic_ns()

                self.sent_frames += 1

                self.last_send_execution_ns = end_ns - start_ns
                self.max_send_execution_ns = max(
                    self.max_send_execution_ns,
                    self.last_send_execution_ns,
                )

            except Exception as exc:
                self.send_errors += 1
                print(
                    f"[can_tx] Failed to send {message_name}: {exc}"
                )

    def get_stats(self) -> dict:
        return {
            "sent_frames": self.sent_frames,
            "send_errors": self.send_errors,
            "last_send_execution_ns": self.last_send_execution_ns,
            "max_send_execution_ns": self.max_send_execution_ns,
        }
        