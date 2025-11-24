# ============================================================================
#  MANUAL CONTROLLER
#  --------------------
#  This is a simple manual controller
#
#  Controllers define:
#       - DT:     update period (seconds)
#       - init_controller():   returns initial internal state dict
#       - step_controller():   runs one control step
#
#  The CANBusIO layer expects controllers to return:
#
#       outputs = {
#           "AUTO_CONTROL": {
#               "FRONT_LEFT_SETPOINT":  value,
#               "FRONT_RIGHT_SETPOINT": value,
#               "REAR_SETPOINT":        value,
#               "PADDING":              value (optional)
#           }
#       }
#
#  AND the internal state must be returned so it can be preserved
#  between control steps.
#
# ============================================================================


# Controller update rate (seconds)
# Example: 0.01 → 100 Hz controller
DT = 0.01


def init_controller():
    """
    Initialize internal controller state here.
    This function is called once whenever:
        - system starts,
        - controller mode changes,
        - or controller is reset.

    The state dictionary may store:
        - integrator terms
        - filtered sensor values
        - previous control outputs
        - previous timestamp
        - user-defined variables
    """

    state = {
        # manual controller does not have internal state
    }

    return state


def step_controller(state, inputs):
    """
    Perform **one control step**.

    Parameters
    ----------
    state : dict
        Controller-internal state (persistent between steps)

    inputs : dict
        Dictionary containing **all decoded CAN signals** from CANBusIO.
        Example keys (depending on your SIGNAL_MAP):
            inputs["ACCELEROMETER_AX"]
            inputs["GYROSCOPE_AY"]
            inputs["DISTANCE_FORE_LEFT"]
            inputs["RADIO_THROTTLE"]
            inputs["RADIO_STEERING"]

    Returns
    -------
    state : dict
        Updated internal state

    outputs : dict
        CAN messages to be transmitted by CANBusIO.
        The outer dictionary key must match the **DBC message name**.

        Example:
            outputs = {
                "AUTO_CONTROL": {
                    "FRONT_LEFT_SETPOINT":  1,
                    "FRONT_RIGHT_SETPOINT": 2,
                    "REAR_SETPOINT":        -2,
                    "PADDING":              0,
                }
            }

        If no outputs are produced:
            return state, {}
    """

    def map(x, in_min, in_max, out_min, out_max):
        # Linear mapping from one range to another
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def saturate(x, x_min, x_max):
        # Saturate x to [x_min, x_max]
        return max(min(x, x_max), x_min)


    # Read inputs from controller
    radio_front_pitch = inputs.get("RADIO_FRONT_PITCH", 0)
    radio_front_roll = inputs.get("RADIO_STEERING", 0)
    radio_rear_pitch = inputs.get("RADIO_REAR_PITCH", 0)

    # Compute control signals for front foils
    gain_pitch = 1000
    gain_roll = 1000
    # left = pitch + roll
	# right = pitch - roll
    left_setpoint = (gain_pitch * radio_front_pitch + gain_roll * radio_front_roll) / 1000
    right_setpoint = (gain_pitch * radio_front_pitch - gain_roll * radio_front_roll) / 1000
    rear_setpoint = radio_rear_pitch

    # Saturate control signals to [-1000, 1000]
    left_setpoint = saturate(left_setpoint, -1000, 1000)
    right_setpoint = saturate(right_setpoint, -1000, 1000)
    rear_setpoint = saturate(rear_setpoint, -1000, 1000)

    # Scale to -6 +12 degrees
    left_setpoint = map(left_setpoint, -1000, 1000, -6, 12)
    right_setpoint = map(right_setpoint, -1000, 1000, -6, 12)
    rear_setpoint = map(rear_setpoint, -1000, 1000, -6, 12)

    # Write outputs to CAN message structure
    outputs = {
        "AUTO_CONTROL": {
            "FRONT_LEFT_SETPOINT":  left_setpoint,
            "FRONT_RIGHT_SETPOINT": right_setpoint,
            "REAR_SETPOINT":        rear_setpoint,
            "PADDING":              0,
        }
    }
    return state, outputs
