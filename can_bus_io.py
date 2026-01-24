import struct
import time
import can
import cantools

class CANBusIO:
    def __init__(self, 
                 can_channel="can0", 
                 db_path="modules/can-messages-mini-celka/can_messages_mini_celka.dbc",
                 debug=False
                 ):

        self.db_path = db_path
        self.db = cantools.database.load_file(self.db_path) # Load DBC file

        # These are the messages we decode (names from the DBC)
        self.INPUT_MESSAGES = [
            "RADIO_CONTROL",
            "ACCELEROMETER",
            "GYROSCOPE",
            "DISTANCE_FORE_FEEDBACK",
            "DISTANCE_ACHTER_FEEDBACK",


            "ODRIVE_GET_BUS_VOLTAGE_CURRENT",
            "ODRIVE_GET_IQ",
            "ODRIVE_SET_INPUT_VEL",
            "ODRIVE_GET_SENSORLESS_ESTIMATES",
        ]

        # Mapping: (message_name, signal_name) -> boat_state_key
        self.SIGNAL_MAP = {
            ("RADIO_CONTROL", "THROTTLE")       : "RADIO_THROTTLE",
            ("RADIO_CONTROL", "STEERING")       : "RADIO_STEERING",
            ("RADIO_CONTROL", "FRONT_PITCH")    : "RADIO_FRONT_PITCH",
            ("RADIO_CONTROL", "FRONT_ROLL")     : "RADIO_FRONT_ROLL",
            ("RADIO_CONTROL", "REAR_PITCH")     : "RADIO_REAR_PITCH",
            ("RADIO_CONTROL", "ARM_SWITCH")     : "RADIO_ARM_SWITCH",
            ("RADIO_CONTROL", "MODE_SWITCH")    : "RADIO_MODE_SWITCH",

            ("ACCELEROMETER", "AX"): "ACCELEROMETER_X",
            ("ACCELEROMETER", "AY"): "ACCELEROMETER_Y",
            ("ACCELEROMETER", "AZ"): "ACCELEROMETER_Z",

            ("GYROSCOPE", "GX"): "GYROSCOPE_X",
            ("GYROSCOPE", "GY"): "GYROSCOPE_Y",
            ("GYROSCOPE", "GZ"): "GYROSCOPE_Z",

            ("DISTANCE_FORE_FEEDBACK", "RANGE_MM_L") : "DISTANCE_FORE_LEFT",
            ("DISTANCE_FORE_FEEDBACK", "ERROR_STATUS_L") : "DISTANCE_FORE_LEFT_STATUS",
            ("DISTANCE_FORE_FEEDBACK", "RANGE_MM_R") : "DISTANCE_FORE_RIGHT",
            ("DISTANCE_FORE_FEEDBACK", "ERROR_STATUS_R") : "DISTANCE_FORE_RIGHT_STATUS",

            ("DISTANCE_ACHTER_FEEDBACK", "RANGE_MM_L") : "DISTANCE_ACHTER_LEFT",
            ("DISTANCE_ACHTER_FEEDBACK", "ERROR_STATUS_L") : "DISTANCE_ACHTER_LEFT_STATUS",
            ("DISTANCE_ACHTER_FEEDBACK", "RANGE_MM_R") : "DISTANCE_ACHTER_RIGHT",
            ("DISTANCE_ACHTER_FEEDBACK", "ERROR_STATUS_R") : "DISTANCE_ACHTER_RIGHT_STATUS",


            ("ODRIVE_GET_BUS_VOLTAGE_CURRENT", "Bus_Voltage") : "ODRIVE_BUS_VOLTAGE",
            ("ODRIVE_GET_BUS_VOLTAGE_CURRENT", "Bus_Current") : "ODRIVE_BUS_CURRENT",

            ("ODRIVE_GET_IQ", "Iq_Setpoint") : "ODRIVE_IQ_SETPOINT",
            ("ODRIVE_GET_IQ", "Iq_Measured") : "ODRIVE_IQ_MEASURED",

            ("ODRIVE_SET_INPUT_VEL", "Input_Vel") : "ODRIVE_INPUT_VELOCITY",

            ("ODRIVE_GET_SENSORLESS_ESTIMATES", "Sensorless_Vel_Estimate") : "ODRIVE_VELOCITY_ESTIMATE",
        }

        self.TIMESTAMP_MAP = {
            "RADIO_CONTROL":            "RADIO_CONTROL",
            "ACCELEROMETER":            "ACCELEROMETER",
            "GYROSCOPE":                "GYROSCOPE",
            "DISTANCE_FORE_FEEDBACK":   "DISTANCE_FORE_FEEDBACK",
            "DISTANCE_ACHTER_FEEDBACK": "DISTANCE_ACHTER_FEEDBACK",

            "ODRIVE_GET_BUS_VOLTAGE_CURRENT":  "ODRIVE_GET_BUS_VOLTAGE_CURRENT",
            "ODRIVE_GET_IQ":                   "ODRIVE_GET_IQ",
            "ODRIVE_SET_INPUT_VEL":            "ODRIVE_SET_INPUT_VEL",
            "ODRIVE_GET_SENSORLESS_ESTIMATES": "ODRIVE_GET_SENSORLESS_ESTIMATES",
        }

        if not debug:
            # Prepare decoder: frame_id -> dbc message object
            self.in_messages = {}
            for name in self.INPUT_MESSAGES:
                msg = self.db.get_message_by_name(name)
                self.in_messages[msg.frame_id] = msg

            # Precompute FLOAT32 signals per frame_id
            self.float32_signals = {}
            for frame_id, msg in self.in_messages.items():
                self.float32_signals[frame_id] = {
                    s.name for s in msg.signals
                    if "FLOAT32_IEEE" in (s.unit or "")
                }

            # Unified boat state initialization
            self.boat_state = {key: 0 for key in self.SIGNAL_MAP.values()}
            # self.boat_state["timestamp"] = time.monotonic()

            for base in self.TIMESTAMP_MAP.values():
                self.boat_state[f"{base}_timestamp"] = None

            # CAN bus
            self.bus = can.interface.Bus(channel=can_channel, bustype="socketcan")

    # ----------------------------------------------------------------------

    def process_incoming(self, msg):
        """Decode incoming CAN frames and update boat_state."""
        if msg.arbitration_id not in self.in_messages:
            return

        dbc_msg = self.in_messages[msg.arbitration_id]

        try:
            decoded = dbc_msg.decode(msg.data)
        except Exception:
            return
        
        # FLOAT32_IEEE conversion
        for name in self.float32_signals.get(msg.arbitration_id, ()):
            raw = decoded.get(name)
            if isinstance(raw, int):
                decoded[name] = struct.unpack('<f', raw.to_bytes(4, 'little'))[0]

        msg_name = dbc_msg.name
        # Update timestamp for this message
        self.boat_state[f"{self.TIMESTAMP_MAP[msg_name]}_timestamp"] = time.monotonic()
        # Map decoded signals to boat_state
        for sig_name, val in decoded.items():
            key = self.SIGNAL_MAP.get((msg_name, sig_name))
            if key:
                self.boat_state[key] = val
        
    # ----------------------------------------------------------------------

    def get_state(self):
        """Return current boat_state."""
        return dict(self.boat_state)

    # ----------------------------------------------------------------------
    # GENERIC ENCODER/SENDER
    # ----------------------------------------------------------------------
    def send_message(self, message_name: str, **signals):
        """
        Generic CAN sender using DBC message name.
        Example:
            send_message("AUTO_CONTROL", FRONT_LEFT_SETPOINT=10, ...)
        """
        try:
            msg = self.db.get_message_by_name(message_name)
            data = msg.encode(signals)

            frame = can.Message(
                arbitration_id=msg.frame_id,
                data=data,
                is_extended_id=False
            )
            self.bus.send(frame)

        except Exception as e:
            print(f"[WARN] Failed to send {message_name}: {e}")

    # ----------------------------------------------------------------------
    # SPECIFIC API FOR MESSAGES
    # ----------------------------------------------------------------------
    def send_AUTO_CONTROL(self, 
                          FRONT_LEFT_SETPOINT, 
                          FRONT_RIGHT_SETPOINT, 
                          REAR_SETPOINT, 
                          PADDING=0):
        
        self.send_message(
            "AUTO_CONTROL",
            FRONT_LEFT_SETPOINT=FRONT_LEFT_SETPOINT,
            FRONT_RIGHT_SETPOINT=FRONT_RIGHT_SETPOINT,
            REAR_SETPOINT=REAR_SETPOINT,
            PADDING=PADDING,
        )


# ----------------------------------------------------------------------
# SIMPLE DBC INSPECTOR / DEBUG UTILITY
# ----------------------------------------------------------------------
if __name__ == "__main__":
    print("=== CANBusIO Simple DBC inspector / debug utility ===")

    try:
        # Use the class
        bus_io = CANBusIO(debug=True)
        db = bus_io.db
    except Exception as e:
        print(f"Failed to load DBC: {e}")
        exit(1)

    print("\nAvailable messages in DBC:")
    for msg in db.messages:
        print(f" - {msg.name}  (0x{msg.frame_id:X})")

    print("\nSignals in each message:")
    for msg in db.messages:
        print(f"\nMessage: {msg.name}  [0x{msg.frame_id:X}]")
        for sig in msg.signals:
            print(f"   - {sig.name}  ({sig.start} bit, {sig.length} bits)")

    print("\nDone.")
