#!/usr/bin/env python3

SIGNAL_MAP = {
    ("RADIO_CONTROL", "THROTTLE"): "RADIO_THROTTLE",
    ("RADIO_CONTROL", "STEERING"): "RADIO_STEERING",
    ("RADIO_CONTROL", "FRONT_PITCH"): "RADIO_FRONT_PITCH",
    ("RADIO_CONTROL", "FRONT_ROLL"): "RADIO_FRONT_ROLL",
    ("RADIO_CONTROL", "REAR_PITCH"): "RADIO_REAR_PITCH",
    ("RADIO_CONTROL", "SYNC_SWITCH"): "RADIO_SYNC_SWITCH",
    ("RADIO_CONTROL", "ARM_SWITCH"): "RADIO_ARM_SWITCH",
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

    ("AUTO_CONTROL", "FRONT_LEFT_SETPOINT") : "AUTO_CONTROL_FRONT_LEFT_SETPOINT",
    ("AUTO_CONTROL", "FRONT_RIGHT_SETPOINT") : "AUTO_CONTROL_FRONT_RIGHT_SETPOINT",
    ("AUTO_CONTROL", "REAR_SETPOINT") : "AUTO_CONTROL_REAR_SETPOINT",

    ("ODRIVE_GET_BUS_VOLTAGE_CURRENT", "Bus_Voltage"): "ODRIVE_BUS_VOLTAGE",
    ("ODRIVE_GET_BUS_VOLTAGE_CURRENT", "Bus_Current"): "ODRIVE_BUS_CURRENT",
    ("ODRIVE_GET_IQ", "Iq_Setpoint"): "ODRIVE_IQ_SETPOINT",
    ("ODRIVE_GET_IQ", "Iq_Measured"): "ODRIVE_IQ_MEASURED",
    ("ODRIVE_SET_INPUT_VEL", "Input_Vel"): "ODRIVE_INPUT_VELOCITY",
    ("ODRIVE_GET_SENSORLESS_ESTIMATES", "Sensorless_Vel_Estimate"): "ODRIVE_VELOCITY_ESTIMATE",

    ("ACTUATOR_LEFT_FOIL_FEEDBACK", "POSITION_RAW"): "LEFT_FOIL_POSITION_RAW",
    ("ACTUATOR_LEFT_FOIL_FEEDBACK", "CURRENT"): "LEFT_FOIL_CURRENT",
    ("ACTUATOR_RIGHT_FOIL_FEEDBACK", "POSITION_RAW"): "RIGHT_FOIL_POSITION_RAW",
    ("ACTUATOR_RIGHT_FOIL_FEEDBACK", "CURRENT"): "RIGHT_FOIL_CURRENT",
    ("ACTUATOR_REAR_FOIL_FEEDBACK", "POSITION_RAW"): "REAR_FOIL_POSITION_RAW",
    ("ACTUATOR_REAR_FOIL_FEEDBACK", "CURRENT"): "REAR_FOIL_CURRENT",
    ("ACTUATOR_STEERING_FEEDBACK", "POSITION_RAW"): "STEERING_POSITION_RAW",
    ("ACTUATOR_STEERING_FEEDBACK", "CURRENT"): "STEERING_CURRENT",
}

SIGNAL_NAMES = tuple(dict.fromkeys(SIGNAL_MAP.values()))
SIGNAL_INDEX = {name: i for i, name in enumerate(SIGNAL_NAMES)}

MESSAGE_NAMES = tuple(dict.fromkeys(msg for msg, _ in SIGNAL_MAP))
MESSAGE_INDEX = {name: i for i, name in enumerate(MESSAGE_NAMES)}


class LatestReadout:
    def __init__(self, values, message_timestamps_ns, lock):
        self._values = values
        self._message_timestamps_ns = message_timestamps_ns
        self._lock = lock

    @classmethod
    def create(cls, mp_context):
        values = mp_context.RawArray("d", len(SIGNAL_NAMES))
        message_timestamps_ns = mp_context.RawArray("q", len(MESSAGE_NAMES))
        lock = mp_context.Lock()
        return cls(values, message_timestamps_ns, lock)

    def update(self, message_name: str, decoded: dict, rx_monotonic_ns: int):
        if not decoded:
            return

        with self._lock:
            for signal_name, value in decoded.items():
                readout_name = SIGNAL_MAP.get((message_name, signal_name))
                if readout_name is not None:
                    self._values[SIGNAL_INDEX[readout_name]] = float(value)

            idx = MESSAGE_INDEX.get(message_name)
            if idx is not None:
                self._message_timestamps_ns[idx] = int(rx_monotonic_ns)

    def snapshot(self) -> dict:
        with self._lock:
            out = {
                name: self._values[i]
                for i, name in enumerate(SIGNAL_NAMES)
            }

            for i, message_name in enumerate(MESSAGE_NAMES):
                ts_ns = self._message_timestamps_ns[i]
                out[f"{message_name}_timestamp"] = (
                    ts_ns * 1e-9 if ts_ns else None
                )

        return out
