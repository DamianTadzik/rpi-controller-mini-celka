"""
Definition of the runtime log file format.

The log file consists of consecutive MsgPack objects:

    header
    record
    record
    record
    ...

Each data record is encoded as:

    [schema_id, primary_timestamp, field_0, field_1, ...]

The meaning and order of fields is described by the schemas
stored in the file header.
"""

LOG_FORMAT_VERSION = 1
# Schema IDs
SCHEMA_CAN_LOG = 1
SCHEMA_CONTROL_CYCLE_LOG = 2

SCHEMAS = {
    SCHEMA_CAN_LOG: {
        "type": "can",
        "fields": [
            "msg_timestamp_s",
            "timestamp_s",
            "timestamp_monotonic_ns",
            "can_id",
            "dlc",
            "data",
            "message",
            "signals",
        ],
    },
    SCHEMA_CONTROL_CYCLE_LOG: {
        "type": "control_cycle",
        "fields": [
            "timestamp_s",
            "timestamp_monotonic_ns",
            "lateness_ns",
            "execution_ns",
            "observer_execution_ns",
            "controller_execution_ns",

            "estimated_state/velocity_mps",
            "estimated_state/z_m",
            "estimated_state/z_dot_mps",
            "estimated_state/phi_rad",
            "estimated_state/theta_rad",
            "estimated_state/psi_rad",
            "estimated_state/p_radps",
            "estimated_state/q_radps",
            "estimated_state/r_radps",
            "estimated_state/delta_FL_deg",
            "estimated_state/delta_FR_deg",
            "estimated_state/delta_R_deg",
            
            "outputs/front_left_setpoint",
            "outputs/front_right_setpoint",
            "outputs/rear_setpoint",
        ],
    }
}

def build_header():
    """
    Build a self-describing log file header.

    The header contains everything required to interpret subsequent
    array-encoded records.
    """
    return {
        "type": "header",
        "version": LOG_FORMAT_VERSION,
        "schemas": SCHEMAS,
    }
