RPi Hydrofoil Control Runtime
## Overview

(CAN bus + DBC decoding + modular controllers)

This project implements a runtime controller engine for a hydrofoil robotic platform running on a Raspberry Pi.
The system receives sensor, radio, and feedback signals via CAN bus, decodes them using a DBC specification, executes the selected controller, and sends actuator commands back over CAN.

The focus of the project is modularity and clarity:

- **CANBusIO** handles all CAN traffic and DBC decoding/encoding
- **run_controller.py** is a small scheduler that selects and runs controllers
- **controllers/** contains hot-swappable controllers (manual, auto, template, etc.)

The architecture is deterministic, testable, and mirrors the controller logic used on your embedded STM32 systems.

## Architecture

### 1. `can_bus_io.py`

Handles all low-level I/O:

- Opens SocketCAN (`can0`)
- Loads the DBC
- Decodes incoming messages → updates a unified `boat_state` dictionary
- Provides semantic send functions (e.g., `send_AUTO_CONTROL()`)
- Supports a standalone DBC inspector (run directly)

Incoming and outgoing CAN configuration is fully contained inside this module.

### 2. `run_controller.py`

The main runtime loop:

- Drains CAN buffer continuously
- Reads state from CANBusIO
- Uses `RADIO_ARM_SWITCH` and `RADIO_MODE_SWITCH` to select controller mode
- Runs controller at its own timing (`controller.DT`)
- Sends actuator commands when controller produces output

Completely independent of CAN configuration.

### 3. `controllers/`

Each controller module implements:

- `DT` – controller update interval (seconds)
- `init_controller()` – initializes controller-internal state
- `step_controller(state, inputs)` – computes outputs based on incoming signals

Included:

- `manual_controller.py` – radio passthrough + mixing logic (STM32-compatible)
- `template_controller.py` – reference for building new controllers

## Installation

Activate your virtual environment on the Pi:

```bash
cd rpi_controller_repo
python3 -m venv venv
source venv/bin/activate
pip install python-can cantools
```


## Running the Controller on Raspberry Pi

### 1. Enable CAN

```bash
sudo ip link set can0 up type can bitrate 500000

```

### 2. Activate your Python environment

```bash
cd rpi_controller_repo
source venv/bin/activate
```

### 3. Start the runtime loop

```bash
python3 run_controller.py
```

This will:

- Begin listening on CAN
- Decode all sensor + radio + feedback signals
- Automatically select manual or auto controller based on:
  - `RADIO_ARM_SWITCH`
  - `RADIO_MODE_SWITCH`
- Run controller logic at the controller's own `DT`
- Send `AUTO_CONTROL` CAN messages continuously

### 4. To stop

Press `Ctrl + C`.

## Debug / DBC Inspector

To inspect available messages and signals in the DBC, run:

```bash
python3 can_bus_io.py
```

This will print all messages and signals defined in the DBC, helping you build or verify mapping definitions.
