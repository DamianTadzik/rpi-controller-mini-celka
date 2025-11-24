# ============================================================================
#  TEMPLATE CONTROLLER
#  --------------------
#  This file serves as a reference for building new controllers.
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
        "last_error": 0.0,
        "integrator": 0.0,
        "timestamp": 0.0,
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
                    "FRONT_LEFT_SETPOINT":  300,
                    "FRONT_RIGHT_SETPOINT": 300,
                    "REAR_SETPOINT":        512,
                    "PADDING":              0,
                }
            }

        If no outputs are produced:
            return state, {}
    """

    # ==============================================================
    # Example: simple pass-through (copy radio steering to servos)
    # ==============================================================

    # Read some input values safely
    radio_pitch = inputs.get("RADIO_FRONT_PITCH", 0)
    radio_steer = inputs.get("RADIO_STEERING", 0)

    # Example control logic:
    fl = radio_pitch + radio_steer
    fr = radio_pitch - radio_steer
    rear = 0  # Could be throttle-based

    outputs = {
        "AUTO_CONTROL": {
            "FRONT_LEFT_SETPOINT":  fl,
            "FRONT_RIGHT_SETPOINT": fr,
            "REAR_SETPOINT":        rear,
            "PADDING":              0,
        }
    }

    return state, outputs
