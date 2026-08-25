"""
Example controller implementation for the Mini Celka control runtime.
Every controller module should expose a class named ``Controller`` with the
following interface:

    controller = Controller()
    outputs = controller.step(
        readout,
        estimated_state,
    )

The controller object is created once when the control-loop process starts.
Any persistent internal controller state should therefore be stored directly
as instance attributes (``self.*``).

Inputs
------
readout : dict
    Latest decoded CAN readout. These values come directly from the
    LatestReadout shared-memory object.
    Example:
        readout["RADIO_FRONT_PITCH"]
        readout["RADIO_FRONT_ROLL"]
        readout["RADIO_REAR_PITCH"]
estimated_state : dict
    Current output of the observer.
    Example:
        estimated_state["z_m"]
        estimated_state["z_dot_mps"]
        estimated_state["phi_rad"]
        estimated_state["theta_rad"]
    A controller does not have to use the estimated state. The manual
    controller intentionally ignores it.

Outputs
-------
dict
    Tuple of setpoints to transmit, in degrees.
    (left_setpoint, right_setpoint, rear_setpoint)

Persistent state
----------------
Any controller state that must survive between calls to ``step()`` should be
stored as instance attributes.
Example:
    self.integral_error = 0.0
    self.previous_error = 0.0
    self.filtered_value = 0.0
"""

from controllers.control_helpers import map, saturate


class Controller:
    """
    Manual hydrofoil controller.

    Maps RC pitch/roll commands directly to front foil incidence angles and
    the rear pitch command to the rear foil incidence angle.
    """

    def __init__(self):
        # Controller parameters
        self.pitch_gain = 1.0
        self.roll_gain = 1.0

        self.command_min = -1000.0
        self.command_max = 1000.0

        self.foil_angle_min_deg = -6.0
        self.foil_angle_max_deg = 12.0

        # Persistent controller state
        # The manual controller currently has no dynamic internal state.
        # More complex controllers may store for example:
        # self.integral_error = 0.0
        # self.previous_error = 0.0
        # self.previous_output = 0.0

    def step(self, readout: dict, estimated_state: dict) -> dict:
        """
        Execute one controller step.

        Parameters
        ----------
        readout : dict
            Latest decoded CAN inputs.

        estimated_state : dict
            Current observer output. Not used by the manual controller.

        Returns
        -------
        tuple
            (front_left_setpoint, front_right_setpoint, rear_setpoint)
        """

        # Read controller inputs
        radio_front_pitch = readout.get("RADIO_FRONT_PITCH", 0.0)
        radio_front_roll = readout.get("RADIO_FRONT_ROLL", 0.0)
        radio_rear_pitch = readout.get("RADIO_REAR_PITCH", 0.0)

        # Control law
        left_command = (self.pitch_gain * radio_front_pitch + self.roll_gain * radio_front_roll)
        right_command = (self.pitch_gain * radio_front_pitch - self.roll_gain * radio_front_roll)
        rear_command = radio_rear_pitch

        # Input-command saturation
        left_command = saturate(left_command, self.command_min, self.command_max)
        right_command = saturate(right_command, self.command_min, self.command_max)
        rear_command = saturate(rear_command, self.command_min, self.command_max)

        # RC command -> foil incidence angle
        left_setpoint = map(left_command, self.command_min, self.command_max, self.foil_angle_min_deg, self.foil_angle_max_deg)
        right_setpoint = map(right_command, self.command_min, self.command_max, self.foil_angle_min_deg, self.foil_angle_max_deg)
        rear_setpoint = map(rear_command, self.command_min, self.command_max, self.foil_angle_min_deg, self.foil_angle_max_deg)

        # CAN outputs
        return (left_setpoint, right_setpoint, rear_setpoint)
        