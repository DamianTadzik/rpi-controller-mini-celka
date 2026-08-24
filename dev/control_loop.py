#!/usr/bin/env python3

import json
import importlib
import time
from queue import Full

from can_tx import CANTransmitter
import config

from log_format import SCHEMA_CONTROL_CYCLE_LOG


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

        # Lifetime statistics.
        self.cycles = 0
        self.missed_cycles = 0
        self.log_queue_drops = 0
        # Statistics since the previous periodic report.
        self._stats_cycles = 0
        self._stats_missed_cycles = 0
        self._stats_execution_ns_sum = 0
        self._stats_max_execution_ns = 0
        self._stats_lateness_ns_sum = 0
        self._stats_max_lateness_ns = 0
        self._stats_observer_execution_ns_sum = 0
        self._stats_max_observer_execution_ns = 0
        self._stats_controller_execution_ns_sum = 0
        self._stats_max_controller_execution_ns = 0
        self._stats_controller_cycles = 0
        self._last_status_print = time.monotonic()

    def _log(self, record):
        try:
            self.log_queue.put_nowait(record)
        except Full:
            self.log_queue_drops += 1

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


    def get_status(self) -> dict:
        elapsed_s = time.monotonic() - self._last_status_print
        cycle_rate_hz = self._stats_cycles / elapsed_s if elapsed_s > 0 else 0.0
        avg_execution_ns = (
            self._stats_execution_ns_sum / self._stats_cycles
            if self._stats_cycles else 0.0
        )
        avg_lateness_ns = (
            self._stats_lateness_ns_sum / self._stats_cycles
            if self._stats_cycles else 0.0
        )
        avg_observer_execution_ns = (
            self._stats_observer_execution_ns_sum / self._stats_cycles
            if self._stats_cycles else 0.0
        )
        avg_controller_execution_ns = (
            self._stats_controller_execution_ns_sum / self._stats_controller_cycles
            if self._stats_controller_cycles else 0.0
        )
        return {
            "module": "control_loop",
            "cycles": self.cycles,
            "missed_cycles": self.missed_cycles,
            "log_queue_drops": self.log_queue_drops,
            "recent": {
                "period_s": elapsed_s,
                "cycles": self._stats_cycles,
                "cycle_rate_hz": cycle_rate_hz,
                "missed_cycles": self._stats_missed_cycles,
                "avg_execution_us": avg_execution_ns / 1e3,
                "max_execution_us": self._stats_max_execution_ns / 1e3,
                "avg_lateness_us": avg_lateness_ns / 1e3,
                "max_lateness_us": self._stats_max_lateness_ns / 1e3,
                "avg_observer_execution_us": avg_observer_execution_ns / 1e3,
                "max_observer_execution_us": self._stats_max_observer_execution_ns / 1e3,
                "controller_cycles": self._stats_controller_cycles,
                "avg_controller_execution_us": avg_controller_execution_ns / 1e3,
                "max_controller_execution_us": self._stats_max_controller_execution_ns / 1e3,
            },
            "can_tx": self.can_tx.get_status(),
        }
    def _reset_status_window(self):
        self._stats_cycles = 0
        self._stats_missed_cycles = 0
        self._stats_execution_ns_sum = 0
        self._stats_max_execution_ns = 0
        self._stats_lateness_ns_sum = 0
        self._stats_max_lateness_ns = 0
        self._stats_observer_execution_ns_sum = 0
        self._stats_max_observer_execution_ns = 0
        self._stats_controller_execution_ns_sum = 0
        self._stats_max_controller_execution_ns = 0
        self._stats_controller_cycles = 0
        self.can_tx.reset_status_window()
    def _print_status_if_due(self):
        now = time.monotonic()
        if now - self._last_status_print < config.STATS_PRINT_PERIOD_S:
            return
        print(json.dumps(self.get_status(), indent=2))
        self._reset_status_window()
        self._last_status_print = now


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

            arm = mode = 1

            observer_start_ns = time.monotonic_ns()
            estimated_state = self.observer.step(readout)
            observer_end_ns = time.monotonic_ns()
            observer_execution_ns = observer_end_ns - observer_start_ns

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

            execution_ns = cycle_end_ns - cycle_start_ns

            self.cycles += 1
            self._stats_cycles += 1
            self._stats_execution_ns_sum += execution_ns
            self._stats_max_execution_ns = max(
                self._stats_max_execution_ns,
                execution_ns,
            )
            self._stats_lateness_ns_sum += lateness_ns
            self._stats_max_lateness_ns = max(
                self._stats_max_lateness_ns,
                lateness_ns,
            )
            self._stats_observer_execution_ns_sum += observer_execution_ns
            self._stats_max_observer_execution_ns = max(
                self._stats_max_observer_execution_ns,
                observer_execution_ns,
            )
            if controller_name != "none":
                self._stats_controller_cycles += 1
                self._stats_controller_execution_ns_sum += controller_execution_ns
                self._stats_max_controller_execution_ns = max(
                    self._stats_max_controller_execution_ns,
                    controller_execution_ns,
                )

            # record = {
            #     "type": "control_cycle",
            #     "timestamp": cycle_start_wall_time,
            #     "timestamp_monotonic_ns": cycle_start_ns,
            #     "lateness_ns": lateness_ns,
            #     "execution_ns": execution_ns,
            #     "observer_execution_ns": observer_execution_ns,
            #     "controller_execution_ns": controller_execution_ns,
            #     # "controller": controller_name,
            #     # "arm": arm,
            #     # "mode": mode,
            #     "estimated_state": estimated_state,
            #     "outputs": outputs,
            # }
            record = [
                SCHEMA_CONTROL_CYCLE_LOG,
                cycle_start_wall_time,
                cycle_start_ns,
                lateness_ns,
                execution_ns,
                observer_execution_ns,
                controller_execution_ns,
                estimated_state,
                outputs,
            ]

            self._log(record)

            next_tick_ns += self.period_ns
            now_ns = time.monotonic_ns()

            if now_ns >= next_tick_ns:
                missed = (now_ns - next_tick_ns) // self.period_ns + 1
                self.missed_cycles += missed
                self._stats_missed_cycles += missed
                next_tick_ns += missed * self.period_ns

            self._print_status_if_due()


def run_control_loop(log_queue, latest_readout):
    ControlLoop(log_queue, latest_readout).run()
