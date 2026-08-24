#!/usr/bin/env python3

import struct
import time
from queue import Full

import can
import cantools

import config


# -----------------------------------------------------------------------------
# Mapping from DBC signals to flat boat-state variables
# -----------------------------------------------------------------------------

SIGNAL_MAP = {
    ("RADIO_CONTROL", "THROTTLE"):    "RADIO_THROTTLE",
    ("RADIO_CONTROL", "STEERING"):    "RADIO_STEERING",
    ("RADIO_CONTROL", "FRONT_PITCH"): "RADIO_FRONT_PITCH",
    ("RADIO_CONTROL", "FRONT_ROLL"):  "RADIO_FRONT_ROLL",
    ("RADIO_CONTROL", "REAR_PITCH"):  "RADIO_REAR_PITCH",

    ("RADIO_CONTROL", "SYNC_SWITCH"): "RADIO_SYNC_SWITCH",
    ("RADIO_CONTROL", "ARM_SWITCH"):  "RADIO_ARM_SWITCH",
    ("RADIO_CONTROL", "MODE_SWITCH"): "RADIO_MODE_SWITCH",

    ("ACCELEROMETER", "AX"): "ACCELEROMETER_X",
    ("ACCELEROMETER", "AY"): "ACCELEROMETER_Y",
    ("ACCELEROMETER", "AZ"): "ACCELEROMETER_Z",

    ("GYROSCOPE", "GX"): "GYROSCOPE_X",
    ("GYROSCOPE", "GY"): "GYROSCOPE_Y",
    ("GYROSCOPE", "GZ"): "GYROSCOPE_Z",

    ("GPS_MOTION", "GroundSpeed"): "GPS_GROUND_SPEED",
    ("GPS_MOTION", "Heading"): "GPS_HEADING",
    ("GPS_MOTION", "SpeedAccuracyEstimate"): "GPS_GROUND_SPEED_ACCURACY",
    ("GPS_MOTION", "HeadingAccuracyEstimate"): "GPS_HEADING_ACCURACY",

    ("GPS_POSITION", "Latitude"): "GPS_LATITUDE",
    ("GPS_POSITION", "Longitude"): "GPS_LONGITUDE",

    ("GPS_TIME_STATUS", "Year"): "GPS_YEAR",
    ("GPS_TIME_STATUS", "Month"): "GPS_MONTH",
    ("GPS_TIME_STATUS", "Day"): "GPS_DAY",
    ("GPS_TIME_STATUS", "Hour"): "GPS_HOUR",
    ("GPS_TIME_STATUS", "Min"): "GPS_MIN",
    ("GPS_TIME_STATUS", "Sec"): "GPS_SEC",
    ("GPS_TIME_STATUS", "Valid"): "GPS_VALID",
    ("GPS_TIME_STATUS", "FixType"): "GPS_FIX_TYPE",
    ("GPS_TIME_STATUS", "NumSV"): "GPS_NUM_SV",

    ("DISTANCE_FORE_FEEDBACK", "RANGE_MM_L"): "DISTANCE_FORE_LEFT",
    ("DISTANCE_FORE_FEEDBACK", "ERROR_STATUS_L"): "DISTANCE_FORE_LEFT_STATUS",
    ("DISTANCE_FORE_FEEDBACK", "RANGE_MM_R"): "DISTANCE_FORE_RIGHT",
    ("DISTANCE_FORE_FEEDBACK", "ERROR_STATUS_R"): "DISTANCE_FORE_RIGHT_STATUS",

    ("DISTANCE_ACHTER_FEEDBACK", "RANGE_MM_L"): "DISTANCE_ACHTER_LEFT",
    ("DISTANCE_ACHTER_FEEDBACK", "ERROR_STATUS_L"): "DISTANCE_ACHTER_LEFT_STATUS",
    ("DISTANCE_ACHTER_FEEDBACK", "RANGE_MM_R"): "DISTANCE_ACHTER_RIGHT",
    ("DISTANCE_ACHTER_FEEDBACK", "ERROR_STATUS_R"): "DISTANCE_ACHTER_RIGHT_STATUS",

    ("ODRIVE_GET_BUS_VOLTAGE_CURRENT", "Bus_Voltage"): "ODRIVE_BUS_VOLTAGE",
    ("ODRIVE_GET_BUS_VOLTAGE_CURRENT", "Bus_Current"): "ODRIVE_BUS_CURRENT",

    ("ODRIVE_GET_IQ", "Iq_Setpoint"): "ODRIVE_IQ_SETPOINT",
    ("ODRIVE_GET_IQ", "Iq_Measured"): "ODRIVE_IQ_MEASURED",

    ("ODRIVE_SET_INPUT_VEL", "Input_Vel"): "ODRIVE_INPUT_VELOCITY",

    ("ODRIVE_GET_SENSORLESS_ESTIMATES", "Sensorless_Vel_Estimate"):
        "ODRIVE_VELOCITY_ESTIMATE",

    ("ACTUATOR_LEFT_FOIL_FEEDBACK", "POSITION_RAW"): "LEFT_FOIL_POSITION_RAW",
    ("ACTUATOR_LEFT_FOIL_FEEDBACK", "CURRENT"): "LEFT_FOIL_CURRENT",

    ("ACTUATOR_RIGHT_FOIL_FEEDBACK", "POSITION_RAW"): "RIGHT_FOIL_POSITION_RAW",
    ("ACTUATOR_RIGHT_FOIL_FEEDBACK", "CURRENT"): "RIGHT_FOIL_CURRENT",

    ("ACTUATOR_REAR_FOIL_FEEDBACK", "POSITION_RAW"): "REAR_FOIL_POSITION_RAW",
    ("ACTUATOR_REAR_FOIL_FEEDBACK", "CURRENT"): "REAR_FOIL_CURRENT",

    ("ACTUATOR_STEERING_FEEDBACK", "POSITION_RAW"): "STEERING_POSITION_RAW",
    ("ACTUATOR_STEERING_FEEDBACK", "CURRENT"): "STEERING_CURRENT",
}


class CANRx:
    def __init__(self, log_queue, boat_state=None):
        self.log_queue = log_queue

        # Can later be replaced by shared BoatState implementation.
        self.boat_state = boat_state if boat_state is not None else {}

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
    # Boat state
    # -------------------------------------------------------------------------

    def _update_boat_state(self, dbc_msg, decoded, can_timestamp):
        if dbc_msg is None or decoded is None:
            return

        message_name = dbc_msg.name

        for signal_name, value in decoded.items():
            state_key = SIGNAL_MAP.get(
                (message_name, signal_name)
            )

            if state_key is not None:
                self.boat_state[state_key] = value

        # Timestamp of the newest occurrence of this CAN message.
        self.boat_state[f"{message_name}_timestamp"] = can_timestamp

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
            #
            # Blocking receive. No polling and no arbitrary sleep.
            #
            msg = self.bus.recv()

            if msg is None:
                continue

            #
            # Capture these immediately after recv().
            #
            rx_wall_time_ns = time.time_ns()
            rx_monotonic_ns = time.monotonic_ns()

            self.received_frames += 1

            #
            # Decode exactly once.
            #
            dbc_msg, decoded = self._decode(msg)

            #
            # Update latest-known state for the controller.
            #
            self._update_boat_state(
                dbc_msg,
                decoded,
                msg.timestamp,
            )

            #
            # Log every CAN frame independently.
            #
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

def run_can_rx(log_queue, boat_state=None):
    can_rx = CANRx(
        log_queue=log_queue,
        boat_state=boat_state,
    )

    can_rx.run()
    