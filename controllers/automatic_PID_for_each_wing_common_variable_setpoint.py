# ============================================================================
#  Controller define:
#       - DT:     update period (seconds)
#       - NAME:   controller name displayed in telemetry
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
# ============================================================================

from controllers.control_helpers import map, saturate, lowpass_filter
from controllers.control_helpers import PID_initialize_state, PID_controller_step

DT = 0.01 # Controller update rate (seconds)
NAME = "PID_each" # idk if name should be short if that's transmitted over UDP with msgpack
# automatic_PID_for_each_wing_common_variable_setpoint

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
        "setpoint_heave_mm": 100.0, # desired heave in mm

        "rear_distance": 0.0,
        "front_left_distance": 0.0,
        "front_right_distance": 0.0,

        "rear_error": 0.0,
        "front_left_error": 0.0, 
        "front_right_error": 0.0,

        # Rear wing PID controller state
        "rear_controller" : PID_initialize_state(
            Kp=0.2, Ki=0.01, Kd=0.004, integral_absolute_limit=10.0
        ),

        "front_left_controller" : PID_initialize_state(
            Kp=0.2, Ki=0.01, Kd=0.004, integral_absolute_limit=10.0
        ),

        "front_right_controller" : PID_initialize_state(
            Kp=0.3, Ki=0.01, Kd=0.004, integral_absolute_limit=10.0
        ),
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

    Returns
    -------
    state : dict
        Updated internal state

    outputs : dict
        CAN messages to be transmitted by CANBusIO.
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
    # Read input values
    radio_rear_pitch = inputs["RADIO_REAR_PITCH"]

    FL_distance = inputs["DISTANCE_FORE_LEFT"]
    FR_distance = inputs["DISTANCE_FORE_RIGHT"]

    AL_distance = inputs["DISTANCE_ACHTER_LEFT"]
    AR_distance = inputs["DISTANCE_ACHTER_RIGHT"]


    # Setpoint
    state["setpoint_heave_mm"] = map(radio_rear_pitch, -1000, +1000, 50.0, 150.0) 


    # Alghoritm, separate PID for each control surface
    #   Rear wing control
    state["rear_distance"] = lowpass_filter(state["rear_distance"], (AL_distance + AR_distance)/2.0, 0.1)
    state["rear_error"] = state["setpoint_heave_mm"] - state["rear_distance"]
    rear_control = PID_controller_step(state["rear_error"], state["rear_controller"], DT)

    #   Front left wing control
    state["front_left_distance"] = lowpass_filter(state["front_left_distance"], FL_distance, 0.1)
    state["front_left_error"] = state["setpoint_heave_mm"] - state["front_left_distance"]
    front_left_control = PID_controller_step(state["front_left_error"], state["front_left_controller"], DT)
    
    #   Front right wing control
    state["front_right_distance"] = lowpass_filter(state["front_right_distance"], FR_distance, 0.1)
    state["front_right_error"] = state["setpoint_heave_mm"] - state["front_right_distance"]    
    front_right_control = PID_controller_step(state["front_right_error"], state["front_right_controller"], DT)


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
