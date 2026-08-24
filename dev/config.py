from pathlib import Path

# -----------------------------------------------------------------------------
# Paths
# -----------------------------------------------------------------------------

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

# -----------------------------------------------------------------------------
# CAN
# -----------------------------------------------------------------------------

CAN_CHANNEL = "can0"
CAN_INTERFACE = "socketcan"

# -----------------------------------------------------------------------------
# Logger
# -----------------------------------------------------------------------------

# Maximum number of records waiting to be written to disk.
LOG_QUEUE_SIZE = 10_000

# Flush the Python userspace file buffer periodically.
# This is intentionally flush(), not fsync().
LOG_FLUSH_PERIOD_S = 2.0

# Unix socket used by logctl.py.
LOGGER_CONTROL_SOCKET = "/tmp/minicelka_logger.sock"
