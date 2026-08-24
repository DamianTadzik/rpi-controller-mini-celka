def map(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    # Linear mapping from one range to another
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def saturate(x: float, x_min: float, x_max: float) -> float:
    # Saturate x to [x_min, x_max]
    return max(min(x, x_max), x_min)

def lowpass_filter(prev_value: float, new_value: float, alpha: float) -> float:
    # Simple low pass filter
    return prev_value + alpha * (new_value - prev_value)



def PID_initialize_state(Kp: float, Ki: float, Kd: float, integral_absolute_limit: float) -> dict:
    """
    Returns initial state dictionary for a PID controller:
    """
    return {
        # Gains
        "Kp": Kp,
        "Ki": Ki,
        "Kd": Kd,
        # Integral windup limit
        "integral_absolute_limit": integral_absolute_limit,
        "integral": 0.0,
        # Previous error for derivative calculation
        "previous_error": 0.0,
        "derivative": 0.0,
        # PID terms
        "P_term": 0.0,
        "I_term": 0.0,
        "D_term": 0.0,
    }

def PID_controller_step(error: float, controller_state: dict, DT: float) -> float:
    """
    Perform one step of PID control for given controller state and error.
    """
    # Integral calculation with windup guarding
    controller_state["integral"] += error * DT
    controller_state["integral"] = saturate(
        controller_state["integral"],
        -controller_state["integral_absolute_limit"],
        +controller_state["integral_absolute_limit"]
    )

    # Derivative calculation
    controller_state["derivative"] = (error - controller_state["previous_error"]) / DT
    controller_state["previous_error"] = error

    # PID terms
    controller_state["P_term"] = controller_state["Kp"] * error
    controller_state["I_term"] = controller_state["Ki"] * controller_state["integral"]
    controller_state["D_term"] = controller_state["Kd"] * controller_state["derivative"]

    # Return total control output
    return controller_state["P_term"] + controller_state["I_term"] + controller_state["D_term"]
