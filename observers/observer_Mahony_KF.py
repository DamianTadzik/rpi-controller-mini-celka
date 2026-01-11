# observer_Madgwick_KF.py
"""
Madgwick + Kalman Filter observer for state estimation.
This observer processes IMU and TOF senosrs data to estimate
the vehicle's orientation and position. 
It combines the Madgwick filter for attitude estimation
with a Kalman filter for height estimation.

This kalman filter is based on matlab implementation.

Inputs:
- IMU gyro [deg/s] and accel [g] in BODY frame, NED convention (+Z down)
- ToF distances [mm] for 4 sensors: FL, FR, RL, RR

Outputs:
- Estimated state x_hat:
    z       [m]     (NED +down)
    z_dot   [m/s]
    phi     [rad]   roll
    theta   [rad]   pitch
    psi     [rad]   
    yaw     (unobservable here, kept for completeness)
    p,q,r   [rad/s] corrected angular rates (bias removed)

"""

from math import sin, cos, atan2, sqrt, asin
import numpy as np

# =========================
# Quaternion helpers
# =========================
def quat_conj(q):
    # q = [qw,qx,qy,qz]
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)

def quat_mul(a, b):
    # Hamilton product
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw
    ], dtype=float)

def quat_rotate(q, v):
    # Rotate 3D vector v by quaternion q: v' = q * [0,v] * q_conj
    vq = np.array([0.0, v[0], v[1], v[2]], dtype=float)
    return quat_mul(quat_mul(q, vq), quat_conj(q))[1:4]

def quat_to_euler_BW(q):
    # matches your MATLAB quat_to_euler_BW
    qw, qx, qy, qz = q
    phi = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    theta = asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
    psi = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return phi, theta, psi

def eulZYX_to_quat(psi, theta, phi):
    # ZYX (yaw, pitch, roll) -> quaternion [qw,qx,qy,qz]
    # same convention as MATLAB eul2quat([psi theta phi]) with ZYX
    cy = cos(psi * 0.5)
    sy = sin(psi * 0.5)
    cp = cos(theta * 0.5)
    sp = sin(theta * 0.5)
    cr = cos(phi * 0.5)
    sr = sin(phi * 0.5)

    qw = cy*cp*cr + sy*sp*sr
    qx = cy*cp*sr - sy*sp*cr
    qy = cy*sp*cr + sy*cp*sr
    qz = sy*cp*cr - cy*sp*sr
    return np.array([qw, qx, qy, qz], dtype=float)

def safe_norm(v, eps=1e-12):
    n = float(np.linalg.norm(v))
    return max(n, eps)


# =========================
# Mahony update (from your MATLAB)
# =========================
def mahony_update(q, b, gyro_rad_s, accel_g, params):
    """
    q: quaternion [qw,qx,qy,qz] body->world
    b: gyro bias [rad/s] (3,)
    gyro_rad_s: gyro [rad/s] (3,)
    accel_g: accel [g] (3,)
    params: dict with:
        params["Ts"]
        params["observer"]["attitude"]["Kp"], ["Ki"], ["acc_norm_min"], ["acc_norm_max"]
    returns: (q, b, w) where w = gyro_rad_s - b  (bias removed, like MATLAB)
    """
    att = params["observer"]["attitude"]
    Kp = float(att["Kp"])
    Ki = float(att["Ki"])
    Ts = float(params["Ts"])

    a_norm = float(np.linalg.norm(accel_g))
    use_acc = (a_norm > float(att["acc_norm_min"])) and (a_norm < float(att["acc_norm_max"]))

    if use_acc:
        a = accel_g / max(a_norm, 1e-12)

        # Estimated gravity direction in BODY frame (NED: +Z down)
        g_est = quat_rotate(quat_conj(q), np.array([0.0, 0.0, 1.0], dtype=float))

        # error = cross(measured, estimated)
        e = np.cross(a, g_est)
    else:
        e = np.array([0.0, 0.0, 0.0], dtype=float)

    # bias update (integral)
    b = b - Ki * e * Ts

    # corrected gyro for quaternion integration (P + I)
    omega = gyro_rad_s - b + Kp * e

    # quaternion integration
    q_dot = 0.5 * quat_mul(q, np.array([0.0, omega[0], omega[1], omega[2]], dtype=float))
    q = q + q_dot * Ts
    q = q / max(float(np.linalg.norm(q)), 1e-12)

    # return corrected omega WITHOUT proportional term (exactly like MATLAB)
    w = gyro_rad_s - b
    return q, b, w


# =========================
# ToF FRONT (FL, FR) -> z
# =========================
def tof_front_to_z(tof_mm_front, phi, theta, params):
    """
    tof_mm_front: [FL, FR] in mm
    returns z_front [m], NED +down
    """
    d = np.array(tof_mm_front, dtype=float) * 1e-3  # mm -> m

    cphi = cos(phi);  sphi = sin(phi)
    cth  = cos(theta); sth = sin(theta)

    R_x = np.array([
        [1,    0,     0],
        [0,  cphi, -sphi],
        [0,  sphi,  cphi],
    ], dtype=float)

    R_y = np.array([
        [ cth, 0,  sth],
        [  0,  1,   0 ],
        [-sth, 0,  cth],
    ], dtype=float)

    R_BW = R_y @ R_x

    e_W = R_BW @ np.array([0.0, 0.0, 1.0])
    ez = float(e_W[2])

    tof_pos = params["tof"]
    rB = np.column_stack([
        np.array(tof_pos["pos_FL_B"], dtype=float),
        np.array(tof_pos["pos_FR_B"], dtype=float),
    ])
    rW = R_BW @ rB

    z_i = -d * ez - rW[2, :]
    return float(np.mean(z_i))


# =========================
# ToF REAR (RL, RR) -> z
# =========================
def tof_rear_to_z(tof_mm_rear, phi, theta, params):
    """
    tof_mm_rear: [RL, RR] in mm
    returns z_rear [m], NED +down
    """
    d = np.array(tof_mm_rear, dtype=float) * 1e-3  # mm -> m

    cphi = cos(phi);  sphi = sin(phi)
    cth  = cos(theta); sth = sin(theta)

    R_x = np.array([
        [1,    0,     0],
        [0,  cphi, -sphi],
        [0,  sphi,  cphi],
    ], dtype=float)

    R_y = np.array([
        [ cth, 0,  sth],
        [  0,  1,   0 ],
        [-sth, 0,  cth],
    ], dtype=float)

    R_BW = R_y @ R_x

    e_W = R_BW @ np.array([0.0, 0.0, 1.0])
    ez = float(e_W[2])

    tof_pos = params["tof"]
    rB = np.column_stack([
        np.array(tof_pos["pos_RL_B"], dtype=float),
        np.array(tof_pos["pos_RR_B"], dtype=float),
    ])
    rW = R_BW @ rB

    z_i = -d * ez - rW[2, :]
    return float(np.mean(z_i))


def kf_predict(state, accel_g, quat, params):
    Ts = params["Ts"]
    g  = params["g"]

    xh = state["xh"]
    Pz = state["Pz"]

    # vertical accel in world frame (NED +down)
    aB = accel_g * g
    aW = quat_rotate(quat, aB)
    a_z = aW[2] - g - xh[2]

    A = np.array([
        [1.0, Ts, -0.5*Ts*Ts],
        [0.0, 1.0, -Ts],
        [0.0, 0.0, 1.0],
    ])

    B = np.array([0.5*Ts*Ts, Ts, 0.0])

    Q = params["observer"]["heave_KF"]["Q"]

    state["xh"] = A @ xh + B * a_z
    state["Pz"] = A @ Pz @ A.T + Q


def kf_update_z(state, z_meas, params):
    xh = state["xh"]
    Pz = state["Pz"]

    R = params["observer"]["heave_KF"]["R"]

    H = np.array([1.0, 0.0, 0.0])

    S = H @ Pz @ H.T + R
    K = (Pz @ H.T) / S

    state["xh"] = xh + K * (z_meas - H @ xh)
    state["Pz"] = (np.eye(3) - np.outer(K, H)) @ Pz


# Update rate (seconds)
# Example: 0.01 → 100 Hz 
DT = 0.01


def init_observer():
    """
    Initialize internal  state here.
    This function is called once whenever:
        - system starts,
        - reset.

    The state dictionary may store:
        - integrator terms
        - filtered sensor values
        - previous outputs
        - previous timestamp
        - user-defined variables
    """

    # Load the parameters from a MATLAB file or define them here
    params = {
        # fixed observer rate
        "Ts": DT,
        "g": 9.80665,

        # -------------------------
        # ToF sensor geometry (BODY frame, meters)
        # -------------------------
        "tof": {
            "pos_FL_B": np.array([+225, -182, -37], dtype=float) * 1e-3,
            "pos_FR_B": np.array([+225, +182, -37], dtype=float) * 1e-3,
            "pos_RL_B": np.array([-616, -182, -37], dtype=float) * 1e-3,
            "pos_RR_B": np.array([-616, +182, -37], dtype=float) * 1e-3,
        },

        # -------------------------
        # Observer parameters
        # -------------------------
        "observer": {
            # Heave Kalman Filter
            "heave_KF": {
                "R": 4.864859165974720e-05,
                "Q": np.diag([
                    1e-6,  # z
                    1e-4,  # z_dot
                    1e-3,  # accel bias
                ]),
            },

            # Mahony attitude filter
            "attitude": {
                "Kp": 1.2,
                "Ki": 0.01,
                "acc_norm_min": 0.1,
                "acc_norm_max": 1.1,
            },
        },
    }
  

    # Initialize persistent state variables
    state = {
        "params": params,

        # ToF timing
        "t_tof_front_prev": None,
        "t_tof_rear_prev": None,

        # Mahony
        "quat": np.array([1.0, 0.0, 0.0, 0.0]),  # identity
        "gyro_bias": np.zeros(3, dtype=float),

        # Heave KF
        "xh": np.array([0.0, 0.0, 0.0]),         # [z, z_dot, a_bias]
        "Pz": np.diag([10.0, 10.0, 10.0]),       # covariance 3x3

        # Optional cached outputs
        "x_hat": np.zeros(8, dtype=float),
    }

    return state


def step_observer(state, inputs):
    """
    Perform **one step**.

    Parameters
    ----------
    state : dict
        Controller-internal state (persistent between steps)

    inputs : dict
        Dictionary containing **all decoded CAN signals** from CANBusIO.
        Example keys (depending on your SIGNAL_MAP):
            inputs["ACCELEROMETER_AX"]
            inputs["GYROSCOPE_AY"]
            inputs["DISTANCE_FORE_LEFT"]
            inputs["RADIO_THROTTLE"]
            inputs["RADIO_STEERING"]

    Returns
    -------
    state : dict
        Updated internal state

    outputs : dict
        CAN messages to be transmitted by CANBusIO.
        The outer dictionary key must match the **DBC message name**.

        Example:
            outputs = {
                "AUTO_CONTROL": {
                    "FRONT_LEFT_SETPOINT":  300,
                    "FRONT_RIGHT_SETPOINT": 300,
                    "REAR_SETPOINT":        512,
                    "PADDING":              0,
                }
            }

        If no outputs are produced:
            return state, {}
    """

    # ==============================================================
    # Inputs readout, and vector forming
    # ==============================================================
    
    ax = float(inputs.get("ACCELEROMETER_X", 0.0))
    ay = float(inputs.get("ACCELEROMETER_Y", 0.0))
    az = float(inputs.get("ACCELEROMETER_Z", 0.0))
    accel_g = np.array([ax, ay, az], dtype=float) # Those three are coming in at the same time

    gx = float(inputs.get("GYROSCOPE_X", 0.0))
    gy = float(inputs.get("GYROSCOPE_Y", 0.0))
    gz = float(inputs.get("GYROSCOPE_Z", 0.0))
    gyro_dps = np.array([gx, gy, gz], dtype=float) # Those three are coming in at the same time
    gyro_rads = np.deg2rad(gyro_dps)

    FL = float(inputs.get("DISTANCE_FORE_LEFT", 0.0))
    FR = float(inputs.get("DISTANCE_FORE_RIGHT", 0.0))
    tof_F_mm = [FL, FR] # These two are coming in at the same time
    RL = float(inputs.get("DISTANCE_REAR_LEFT", 0.0))
    RR = float(inputs.get("DISTANCE_REAR_RIGHT", 0.0))
    tof_R_mm = [RL, RR] # These two are coming in at the same time


    params = state.get("params")

    params["Ts"] = DT

    # ==============================================================
    # Observer algorithm
    # ==============================================================

    # Mahony

    state["quat"], state["gyro_bias"], gyro_corr = mahony_update(state["quat"], state["gyro_bias"], gyro_rads, accel_g, params)

    phi, theta, psi = quat_to_euler_BW(state["quat"])   # [rad]
    p, q, r = gyro_corr                     # [rad/s]


    # Kalman

    kf_predict(state, accel_g, state["quat"], params)

    # KF update (only when ToF arrives)
    tF = inputs.get("DISTANCE_FORE_FEEDBACK_timestamp", None)
    if tF is not None and tF != state["t_tof_front_prev"]:
        zF = tof_front_to_z(tof_F_mm, phi, theta, params)
        kf_update_z(state, zF, params)
        state["t_tof_front_prev"] = tF

    tR = inputs.get("DISTANCE_ACHTER_FEEDBACK_timestamp")
    if tR is not None and tR != state["t_tof_rear_prev"]:
        zR = tof_rear_to_z(tof_R_mm, phi, theta, params)
        kf_update_z(state, zR, params)
        state["t_tof_rear_prev"] = tR

    # ==============================================================
    # Outputs packing
    # ==============================================================
    z     = state["xh"][0]
    z_dot = state["xh"][1]

    x_hat = {
        "z":      float(z),
        "z_dot":  float(z_dot),
        "phi":    float(phi),
        "theta":  float(theta),
        "psi":    float(psi),
        "p":      float(p),
        "q":      float(q),
        "r":      float(r),
    }

    return state, x_hat
