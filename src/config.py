from pathlib import Path

# Paths
PROJECT_ROOT = Path(__file__).resolve().parent
DBC_PATH = (
    PROJECT_ROOT
    / ".."
    / "modules"
    / "can-messages-mini-celka"
    / "can_messages_mini_celka.dbc"
)
# Na razie osobny katalog, żeby nie mieszać ze starym loggerem.
LOG_DIR = PROJECT_ROOT / "logs_runtime"

# CAN
CAN_CHANNEL = "can0"
CAN_INTERFACE = "socketcan"
CAN_TX_TIMEOUT_S = 0.004

# Logger
# Maximum number of records waiting to be written to disk.
LOG_QUEUE_SIZE = 10_000

# Flush the Python userspace file buffer periodically.
# This is intentionally flush(), not fsync().
LOG_FLUSH_PERIOD_S = 2.0

# Unix socket used by logctl.py.
LOGGER_CONTROL_SOCKET = "/tmp/minicelka_logger.sock"

# Controller, observer
OBSERVER_MODULE = "observers.observer_mahony_kf"
# OBSERVER_MODULE = "observers.observer_test_inputs"
MANUAL_CONTROLLER_MODULE = "controllers.manual_controller"
AUTO_CONTROLLER_MODULE = (
    "controllers.LQ_controller"
)
# Fixed observer/controller loop rate: 100 Hz
CONTROL_PERIOD_S = 0.01

# Runitme stats printout period for all modules that support stats reporting
STATS_PRINT_PERIOD_S = 16.0
