#!/usr/bin/env python3

import struct
import time
from queue import Full

import can
import cantools

import config


class CANRx:
    def __init__(self, log_queue, latest_readout):
        self.log_queue = log_queue
        self.latest_readout = latest_readout

        self.db = cantools.database.load_file(config.DBC_PATH)

        # frame_id -> cantools message
        self.dbc_messages = {
            msg.frame_id: msg
            for msg in self.db.messages
        }

        # Preserve the FLOAT32 handling from the old CANBusIO.
        self.float32_signals = {
            msg.frame_id: {
                signal.name
                for signal in msg.signals
                if "FLOAT32_IEEE" in (signal.unit or "")
            }
            for msg in self.db.messages
        }

        self.bus = can.interface.Bus(
            channel=config.CAN_CHANNEL,
            interface=config.CAN_INTERFACE,
        )

        self.received_frames = 0
        self.decode_errors = 0
        self.log_queue_drops = 0

        print(
            f"[can_rx] Listening on {config.CAN_CHANNEL}, "
            f"DBC: {config.DBC_PATH}"
        )

    # -------------------------------------------------------------------------
    # Decode
    # -------------------------------------------------------------------------

    def _decode(self, msg):
        dbc_msg = self.dbc_messages.get(msg.arbitration_id)

        if dbc_msg is None:
            return None, None

        try:
            decoded = dbc_msg.decode(
                msg.data,
                decode_choices=False,
            )
        except Exception:
            self.decode_errors += 1
            return dbc_msg, None

        # Same custom FLOAT32 conversion as in the previous implementation.
        for name in self.float32_signals.get(msg.arbitration_id, ()):
            raw = decoded.get(name)

            if isinstance(raw, int):
                decoded[name] = struct.unpack(
                    "<f",
                    raw.to_bytes(4, "little"),
                )[0]

        return dbc_msg, decoded

    # -------------------------------------------------------------------------
    # Logging
    # -------------------------------------------------------------------------

    def _log_frame(
        self,
        msg,
        dbc_msg,
        decoded,
        rx_wall_time_ns,
        rx_monotonic_ns,
    ):
        record = {
            "type": "can",
            # SocketCAN/python-can receive timestamp.
            "timestamp": msg.timestamp,
            # Timestamp when our process actually handled the frame.
            "rx_wall_time_ns": rx_wall_time_ns,
            "rx_monotonic_ns": rx_monotonic_ns,
            "can_id": msg.arbitration_id,
            "is_extended_id": msg.is_extended_id,
            "is_remote_frame": msg.is_remote_frame,
            "is_error_frame": msg.is_error_frame,
            "dlc": msg.dlc,
            # Always preserve original CAN data.
            "data": bytes(msg.data),
            # None for messages not present in the DBC.
            "message": dbc_msg.name if dbc_msg is not None else None,
            # None when unknown or when decoding failed.
            "signals": decoded,
        }

        try:
            self.log_queue.put_nowait(record)

        except Full:
            self.log_queue_drops += 1

            # Do not spam stdout if something goes badly wrong.
            if self.log_queue_drops == 1 or self.log_queue_drops % 100 == 0:
                print(
                    f"[can_rx] WARNING: logger queue full, "
                    f"dropped {self.log_queue_drops} records"
                )

    # -------------------------------------------------------------------------
    # Main RX loop
    # -------------------------------------------------------------------------

    def run(self):
        while True:
            # Blocking receive. No polling and no arbitrary sleep.
            msg = self.bus.recv()

            if msg is None:
                continue

            # Capture these immediately after recv().
            rx_wall_time_ns = time.time_ns()
            rx_monotonic_ns = time.monotonic_ns()

            self.received_frames += 1

            # Decode exactly once.
            dbc_msg, decoded = self._decode(msg)

            # Update latest-known state for the controller.
            if dbc_msg is not None and decoded is not None:
                self.latest_readout.update(
                    dbc_msg.name,
                    decoded,
                    rx_monotonic_ns,
                )

            # Log every CAN frame independently.
            self._log_frame(
                msg,
                dbc_msg,
                decoded,
                rx_wall_time_ns,
                rx_monotonic_ns,
            )


# =============================================================================
# Process entry point
# =============================================================================

def run_can_rx(log_queue, latest_readout):
    can_rx = CANRx(
        log_queue=log_queue,
        latest_readout=latest_readout,
    )

    can_rx.run()
    