#!/usr/bin/env python3

import importlib
import time
from queue import Full

from can_tx import CANTransmitter
import config


class ControlLoop:
    def __init__(self, log_queue, latest_readout):
        self.log_queue = log_queue
        self.latest_readout = latest_readout

        self.observer = (
            importlib.import_module(config.OBSERVER_MODULE).Observer()
        )
        self.manual_controller = (
            importlib.import_module(config.MANUAL_CONTROLLER_MODULE).Controller()
        )
        self.auto_controller = (
            importlib.import_module(config.AUTO_CONTROLLER_MODULE).Controller()
        )
        
        # CAN transmitter object for sending control outputs to the boat's CAN bus.
        self.can_tx = CANTransmitter()

        self.period_ns = int(config.CONTROL_PERIOD_S * 1e9)

    def _log(self, record):
        try:
            self.log_queue.put_nowait(record)
        except Full:
            pass

    def _step_controller(self, mode, readout, estimated_state):
        if mode == 1:
            return "manual", self.manual_controller.step(
                readout,
                estimated_state,
            )
        if mode == 2:
            return "auto", self.auto_controller.step(
                readout,
                estimated_state,
            )
        return "none", {}


    def run(self):
        next_tick_ns = time.monotonic_ns()

        while True:
            now_ns = time.monotonic_ns()

            if now_ns < next_tick_ns:
                time.sleep((next_tick_ns - now_ns) * 1e-9)
                continue

            scheduled_ns = next_tick_ns
            cycle_start_wall_time = time.time()
            cycle_start_ns = time.monotonic_ns()
            lateness_ns = max(0, cycle_start_ns - scheduled_ns)

            readout = self.latest_readout.snapshot()
            arm = int(readout.get("RADIO_ARM_SWITCH", 0))
            mode = int(readout.get("RADIO_MODE_SWITCH", 0))

            observer_start_ns = time.monotonic_ns()
            estimated_state = self.observer.step(readout)
            observer_end_ns = time.monotonic_ns()

            controller_name = "none"
            outputs = {}
            controller_execution_ns = 0
            if arm in (1, 2):
                controller_start_ns = time.monotonic_ns()
                controller_name, outputs = self._step_controller(
                    mode,
                    readout,
                    estimated_state,
                )
                controller_end_ns = time.monotonic_ns()
                controller_execution_ns = controller_end_ns - controller_start_ns

                self.can_tx.send_outputs(outputs)

            cycle_end_ns = time.monotonic_ns()

            self._log({
                "type": "control_cycle",
                "timestamp": cycle_start_wall_time,
                "timestamp_monotonic_ns": cycle_start_ns,
                "lateness_ns": lateness_ns,
                "execution_ns": cycle_end_ns - cycle_start_ns,
                "observer_execution_ns": observer_end_ns - observer_start_ns,
                "controller_execution_ns": controller_execution_ns,
                "controller": controller_name,
                "arm": arm,
                "mode": mode,
                "estimated_state": estimated_state,
                "outputs": outputs,
            })

            next_tick_ns += self.period_ns
            now_ns = time.monotonic_ns()

            if now_ns >= next_tick_ns:
                missed = (now_ns - next_tick_ns) // self.period_ns + 1
                next_tick_ns += missed * self.period_ns


def run_control_loop(log_queue, latest_readout):
    ControlLoop(log_queue, latest_readout).run()
