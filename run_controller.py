#!/usr/bin/env python3
import time
import signal

from can_bus_io import CANBusIO

# MANUAL controller (MODE = 1)
from controllers import manual_controller as manual_controller

# AUTO controller (MODE = 2)
from controllers import auto_controller as auto_controller

running = True

def stop(sig, frame):
    global running
    running = False

def main():
    global running
    signal.signal(signal.SIGINT, stop)

    can_io = CANBusIO()

    # Default mode = MANUAL
    ctrl = manual_controller
    ctrl_state = ctrl.init_controller()
    last_mode = None

    # Initialize controller timing
    last_ctrl_run = 0.0

    while running:
        # -------------------------------------------------------
        # Drain CAN buffer completely
        # -------------------------------------------------------
        while True:
            msg = can_io.bus.recv(timeout=0.0)
            if msg is None:
                break
            can_io.process_incoming(msg)
        
        # -------------------------------------------------------
        # Read state
        # -------------------------------------------------------
        state = can_io.get_state()
        ARM = state["RADIO_ARM_SWITCH"]
        MODE = state["RADIO_MODE_SWITCH"]

        # -------------------------------------------------------
        # ARMED?
        # -------------------------------------------------------
        if ARM not in (1, 2):
            # DISARMED → no control output
            time.sleep(0.001)
            continue

        # -------------------------------------------------------
        # Select controller based on MODE
        # -------------------------------------------------------
        if MODE != last_mode:
            if MODE == 1:
                ctrl = manual_controller
            elif MODE == 2:
                ctrl = auto_controller
            else:
                ctrl = manual_controller  # default fallback

            ctrl_state = ctrl.init_controller()
            last_mode = MODE
            last_ctrl_run = time.monotonic()  # reset timing

        # -------------------------------------------------------
        # Periodic controller execution (controller-defined DT)
        # -------------------------------------------------------
        now = time.monotonic()
        if now - last_ctrl_run >= ctrl.DT:
            last_ctrl_run = now

            ctrl_state, outputs = ctrl.step_controller(ctrl_state, state)

            # -------------------------------------------------------
            # Send actuator data (if controller produced any)
            # -------------------------------------------------------
            if outputs and "AUTO_CONTROL" in outputs:
                o = outputs["AUTO_CONTROL"]
                can_io.send_AUTO_CONTROL(
                    FRONT_LEFT_SETPOINT=o.get("FRONT_LEFT_SETPOINT", 0),
                    FRONT_RIGHT_SETPOINT=o.get("FRONT_RIGHT_SETPOINT", 0),
                    REAR_SETPOINT=o.get("REAR_SETPOINT", 0),
                    PADDING=o.get("PADDING", 0),
                )

        # Small sleep to reduce CPU load (optional)
        time.sleep(0.0001)
        
if __name__ == "__main__":
    main()
