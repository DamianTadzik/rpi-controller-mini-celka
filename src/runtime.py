#!/usr/bin/env python3

import multiprocessing as mp
import time

import config
from can_rx import run_can_rx
from log_writer import run_log_writer
from latest_readout import LatestReadout
from control_loop import run_control_loop


def main() -> int:
    # "spawn" daje nam czyste, niezależne procesy i nie dziedziczy przypadkowo
    # otwartych socketów/file descriptorów z procesu rodzica.
    ctx = mp.get_context("spawn")

    # Queue for logging, any process can put log records, and the log_writer will consume them and write to disk.
    log_queue = ctx.Queue(
        maxsize=config.LOG_QUEUE_SIZE
    )

    # Shared state for the latest readout of the recieved CAN signals. Read by control_loop, written by can_rx.
    latest_readout = LatestReadout.create(ctx)

    can_rx_process = ctx.Process(
        name="can_rx",
        target=run_can_rx,
        args=(
            log_queue,
            latest_readout,
        ),
    )
    control_loop_process = ctx.Process(
        name="control_loop",
        target=run_control_loop,
        args=(
            log_queue,
            latest_readout,
        ),
    )
    log_writer_process = ctx.Process(
        name="log_writer",
        target=run_log_writer,
        args=(
            log_queue,
        ),
    )

    processes = [
        log_writer_process,
        can_rx_process,
        control_loop_process,
    ]

    print("[runtime] Starting processes...")

    # Start the logger first so it is ready before CAN RX starts producing data.
    log_writer_process.start()
    can_rx_process.start()
    control_loop_process.start()

    print(f"[runtime] log_writer   PID: {log_writer_process.pid}")
    print(f"[runtime] can_rx       PID: {can_rx_process.pid}")
    print(f"[runtime] control_loop PID: {control_loop_process.pid}")
    print("[runtime] Running.")

    # For now runtime only owns the processes and shared IPC objects.
    while True:
        time.sleep(8.0)


if __name__ == "__main__":
    main()
