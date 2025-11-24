# RPi Controller Runtime (CAN + DBC + Modular Controllers)

This workspace contains:

- A `run_controller.py` runtime that:
  - Listens on SocketCAN (`can0`)
  - Decodes CAN frames using a DBC file
  - Maintains a global `world_state` dictionary with all incoming signals
  - Calls a pluggable controller from `controllers/`
  - Encodes and sends controller outputs using the same DBC
- A `controllers/` folder containing modular controllers
  - Each controller implements two functions:
    - `init_controller()`
    - `step_controller(state, inputs)`

## Requirements

Activate your virtual environment:

```bash
cd rpi_controller_repo
source venv/bin/activate
pip install cantools python-can
```
