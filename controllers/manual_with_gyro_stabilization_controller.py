# ============================================================================
#  FIRST PID CONTROLLER
#  --------------------
#  This is a simple controller
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
    def saturate(x, x_min, x_max):
        # Saturate x to [x_min, x_max]
        return max(min(x, x_max), x_min)

    # Read input values
    radio_pitch = inputs["RADIO_FRONT_PITCH"]

    FL_distance = inputs["DISTANCE_FORE_LEFT"]
    FR_distance = inputs["DISTANCE_FORE_RIGHT"]

    AL_distance = inputs["DISTANCE_ACHTER_LEFT"]
    AR_distance = inputs["DISTANCE_ACHTER_RIGHT"]
    ## ?????? how do i know if reading is correct XD

   
    # Setpoint
    heave_setpoint = 100 # mm  


    # Alghoritm 
    #   Rear wing control
    rear_distance = (AL_distance + AR_distance) / 2.0
    rear_error = heave_setpoint - rear_distance

    rear_control = 0.1 * rear_error

    #   Front left wing control
    front_left_error = heave_setpoint - FL_distance
    front_left_control = 0.1 * front_left_error + radio_pitch 

    #   Front right wing control
    front_right_error = heave_setpoint - FR_distance    
    front_right_control = 0.1 * front_right_error + radio_pitch


    # Output
    outputs = {
        "AUTO_CONTROL": {
            "FRONT_LEFT_SETPOINT":  saturate(front_left_control, -6, +12),
            "FRONT_RIGHT_SETPOINT": saturate(front_right_control, -6, +12),
            "REAR_SETPOINT":        saturate(rear_control, -6, +12),
            "PADDING":              0,
        }
    }
    return state, outputs
