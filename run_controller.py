#!/usr/bin/env python3
import time
import signal

from can_bus_io import CANBusIO
from telemetry import Telemetry

# OBSERVER: Madgwick + Kalman Filter 
from observers import observer_Mahony_KF as observer

# MANUAL controller (MODE = 1)
from controllers import manual_controller as manual_controller

# AUTO controller (MODE = 2)
from controllers import automatic_PID_for_each_wing_common_variable_setpoint as auto_controller

running = True

def stop(sig, frame):
    global running
    running = False

def main():
    global running
    signal.signal(signal.SIGINT, stop)

    can_io = CANBusIO()
    telemetry = Telemetry(ip="255.255.255.255", port=9870, rate_hz=50)
    
    # Init observer state
    observer_state = observer.init_observer()

    # Default mode = MANUAL
    controller = manual_controller
    controller_state = controller.init_controller()
    last_mode = None

    # Initialize timing
    last_observer_run = time.monotonic()
    last_controller_run = time.monotonic()

    while running:
        loop_start = time.monotonic() # TELEMETRY: measure loop time
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
        inputs = can_io.get_state()
        telemetry.push("inputs", inputs)
        ARM = inputs["RADIO_ARM_SWITCH"]
        MODE = inputs["RADIO_MODE_SWITCH"]

        # -------------------------------------------------------
        # Periodic observer execution (observer-defined DT)
        # ------------------------------------------------------
        now = time.monotonic()
        elapsed = now - last_observer_run
        if elapsed >= observer.DT:
            last_observer_run += observer.DT

            telemetry.accum_rt_obs_jitter(elapsed - observer.DT) # TELEMETRY: accumulate jitter
            cycles = int(elapsed // observer.DT)
            if cycles > 1:
                telemetry.flag_rt_obs_missed_cycle(cycles - 1)
            observer_start = time.monotonic() # TELEMETRY: measure observer time

            # Step observer
            observer_state, x_hat = observer.step_observer(observer_state, inputs)
            # Update inputs with estimated state  
            #inputs.update(x_hat) # or inputs["ESTIMATED_STATE"] = estimated_state
            telemetry.push("x_hat", x_hat)
            
            observer_execution_time = time.monotonic() - observer_start # TELEMETRY: measure observer time
            telemetry.accum_rt_obs_time(observer_execution_time) # TELEMETRY: accumulate observer time
            telemetry.accum_rt_obs_max(observer_execution_time)
            telemetry.inc_rt_obs_iterations()

        # -------------------------------------------------------
        # Armed check
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
                controller = manual_controller
            elif MODE == 2:
                controller = auto_controller
            else:
                controller = manual_controller  # default fallback

            last_mode = MODE
            last_controller_run = time.monotonic()  # reset timing

            # Init controller state
            controller_state = controller.init_controller()


        # -------------------------------------------------------
        # Periodic controller execution (controller-defined DT)
        # -------------------------------------------------------
        now = time.monotonic()
        elapsed = now - last_controller_run
        if elapsed >= controller.DT:
            last_controller_run += controller.DT

            telemetry.accum_rt_jitter(elapsed - controller.DT) # TELEMETRY: accumulate jitter
            cycles = int(elapsed // controller.DT)
            if cycles > 1:
                telemetry.flag_rt_missed_cycle(cycles - 1)
            controller_start = time.monotonic()  # TELEMETRY: measure controller time

            # Step controller
            controller_state, outputs = controller.step_controller(controller_state, inputs)

            controller_execution_time = time.monotonic() - controller_start # TELEMETRY: measure controller time
            telemetry.accum_rt_ctrl_time(controller_execution_time) # TELEMETRY: accumulate controller time
            telemetry.accum_rt_ctrl_max(controller_execution_time)
            telemetry.inc_rt_ctrl_iterations()

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

            # -------------------------------------------------------
            # Telemetry
            # -------------------------------------------------------
            telemetry.push("outputs", outputs) 
            telemetry.push(f"{controller.NAME}", controller_state)

        loop_time = time.monotonic() - loop_start # TELEMETRY: measure loop time
        telemetry.accum_rt_loop_time(loop_time) # TELEMETRY: accumulate loop time
        telemetry.accum_rt_loop_max(loop_time)
        if loop_time > controller.DT:
            telemetry.flag_rt_overrun() # TELEMETRY: flag overrun

        # Small sleep to reduce CPU load
        time.sleep(0.0001)
    
    # Cleanup after exiting main loop
    telemetry.stop() 
        
if __name__ == "__main__":
    main()
