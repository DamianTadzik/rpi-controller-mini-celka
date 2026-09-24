#!/usr/bin/env python3
"""
Mahony + Kalman Filter observer for state estimation.

This module estimates the boat motion from IMU, ToF, GPS, and actuator inputs.
It combines a quaternion-based Mahony attitude observer with two Kalman filters:

- a heave (vertical position) state estimator using the four ToF sensors,
- a forward velocity estimator using body acceleration and GPS speed,
- and a simple actuator-state estimator for delayed actuator commands.

Coordinate and unit conventions
------------------------------
- IMU accelerometer readings are treated as g units in the body frame.
- IMU gyroscope readings are provided in deg/s and converted internally to rad/s.
- NED convention is used: +Z points down, so vertical displacement is measured as a
  positive downwards distance.
- ToF distances are in mm, converted to metres internally before use.
- Sensor ordering in the implementation is [FL, FR, RL, RR].

High-level output tuple
----------------------
The `step()` method returns a flattened state tuple:

    velocity      [m/s]    forward world velocity
    z             [m]      heave position (NED +down)
    z_dot         [m/s]    heave velocity
    phi           [rad]    roll
    theta         [rad]    pitch
    psi           [rad]    yaw
    p, q, r       [rad/s]  corrected body angular rates
    delta_hat     [var]    estimated actuator states [FL, FR, RR]
    delay_states  [var]    actuator transport-delay state vector

The returned tuple is intentionally compatible with the rest of the runtime and is
not a single structured state vector object.

Implementation notes
--------------------
- The quaternion uses the convention [qw, qx, qy, qz].
- The yaw angle is kept for completeness even though it is not directly observable
  from the current sensor set.
- ToF updates are accepted only when the corresponding CAN frame timestamp changes
  and the status flag indicates a valid reading (status == 0).
- The observer is based on the MATLAB implementation that this project originally
  used, while keeping the Python code and data flow consistent with the runtime.
"""

from math import sin, cos, atan2, sqrt, asin
import numpy as np

from pathlib import Path
from scipy.io import loadmat

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

from scipy.io import loadmat
class Observer:
    """Observer wrapper for the Mahony attitude estimator and the runtime KFs.

    The class stores all calibration and model parameters loaded from the MATLAB
    parameter file, including the fixed sampling time, heave-KF tuning, velocity-KF
    tuning, and actuator model parameters. It keeps internal state for the current
    quaternion estimate, vertical-position Kalman filter, forward-velocity filter,
    and delayed actuator estimates.
    """

    def __init__(self, params_file="/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/src/observers/boat_controller_parameters.mat"):
        try:
            data = loadmat(params_file, simplify_cells=True)
            self.params = data["ctrl_params"]
            print(f"[observer] Parameters loaded successfully from: {params_file}")
        except Exception as e:
            print(f"[observer] Failed to load parameters: {e}")
            raise
        self.DT = float(self.params["Ts"])

        replay_path = Path(params_file).parent / "observer_controller_simulation_60s.mat"
        try:
            replay_data = loadmat(replay_path, simplify_cells=True)
            self.replay = replay_data["s"]
            self.replay_sample = 0
            self.replay_length = len(np.asarray(self.replay["time"]).reshape(-1))
            print(f"[observer] Replay data loaded successfully from: {replay_path}")
            print(f"[observer] Replay samples: {self.replay_length}")
        except Exception as e:
            print(f"[observer] Failed to load replay data: {e}")
            raise

        # ToF timing
        self.t_tof_front_prev = None
        self.t_tof_rear_prev = None

        # Mahony
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])  # identity
        self.gyro_bias = np.zeros(3, dtype=float)

        # Heave KF
        self.xh = np.array([0.0, 0.0, 0.0])         # [z, z_dot, a_bias]
        self.Pz = np.diag([10.0, 10.0, 10.0])       # covariance 3x3

        # GPS timing
        self.t_gps_prev = None

        # Velocity KF
        self.xv = None   # [velocity, accel_bias]
        self.Pv = None

        # Actuator estimator
        act = self.params["actuator_model"]
        self.actuator_Td = np.asarray(act["Td"], dtype=float)
        self.actuator_Ld = int(act["Ld"])
        self.actuator_alpha_min = np.asarray(act["alpha_min"], dtype=float)
        self.actuator_alpha_max = np.asarray(act["alpha_max"], dtype=float)
        self.delta_hat = None
        self.delay_buffer = None

    # =========================
    # Mahony update (from your MATLAB)
    # =========================
    def mahony_update(self, gyro_rad_s, accel_g):
        """
        gyro_rad_s: gyro [rad/s] (3,)
        accel_g: accel [g] (3,)
        returns: w = gyro_rad_s - bias (bias removed, like MATLAB)
        """
        att = self.params["observer"]["attitude"]
        Kp = float(att["Kp"])
        Ki = float(att["Ki"])
        Ts = float(self.params["Ts"])

        a_norm = float(np.linalg.norm(accel_g))
        acc_norm_error = abs(a_norm - 1.0)
        use_acc = acc_norm_error < float(att["acc_norm_tolerance"])
        if use_acc:
            a = accel_g / max(a_norm, 1e-12)

            # Estimated gravity direction in BODY frame (NED: +Z down)
            g_est = quat_rotate(quat_conj(self.quat), np.array([0.0, 0.0, 1.0], dtype=float))

            # error = cross(measured, estimated)
            e = np.cross(a, g_est)
        else:
            e = np.array([0.0, 0.0, 0.0], dtype=float)

        # bias update (integral)
        self.gyro_bias = self.gyro_bias - Ki * e * Ts

        # corrected gyro for quaternion integration (P + I)
        omega = gyro_rad_s - self.gyro_bias + Kp * e

        # quaternion integration
        q_dot = 0.5 * quat_mul(self.quat, np.array([0.0, omega[0], omega[1], omega[2]], dtype=float))
        self.quat = self.quat + q_dot * Ts
        self.quat = self.quat / max(float(np.linalg.norm(self.quat)), 1e-12)

        # return corrected omega WITHOUT proportional term (exactly like MATLAB)
        w = gyro_rad_s - self.gyro_bias
        return w

    def tof_to_z_i(self, i, tof_mm, phi, theta):
        """Convert a single ToF distance reading into a heave measurement.

        Parameters
        ----------
        i : int
            Sensor index in the implementation order [FL, FR, RL, RR].
        tof_mm : float
            Raw ToF distance in millimetres.
        phi, theta : float
            Roll and pitch estimates used to transform the sensor offsets into the
            world frame.

        Returns
        -------
        float
            Vertical displacement z in the NED +down convention, expressed as a
            measurement for the heave Kalman filter.
        """
        d = float(tof_mm) * 1e-3  # mm -> m

        cphi = cos(phi); sphi = sin(phi)
        cth = cos(theta); sth = sin(theta)

        R_x = np.array([[1,0,0],[0,cphi,-sphi],[0,sphi,cphi]], dtype=float)
        R_y = np.array([[cth,0,sth],[0,1,0],[-sth,0,cth]], dtype=float)
        R_BW = R_y @ R_x

        e_W = R_BW @ np.array([0.0, 0.0, 1.0], dtype=float)
        ez = float(e_W[2])

        tof_pos = self.params["tof"]
        if i == 0:
            rB = np.array(tof_pos["pos_FL_B"], dtype=float)
        elif i == 1:
            rB = np.array(tof_pos["pos_FR_B"], dtype=float)
        elif i == 2:
            rB = np.array(tof_pos["pos_AL_B"], dtype=float)
        elif i == 3:
            rB = np.array(tof_pos["pos_AR_B"], dtype=float)
        else:
            raise ValueError("Invalid ToF index")

        rW = R_BW @ rB
        return -d * ez - float(rW[2])  # NED +down

    def kf_predict(self, accel_g):
        """Predict the heave-state estimate from the current acceleration.

        State vector: [z, z_dot, a_bias], with z using the NED +down convention.
        The transform body->world is performed through the current quaternion, and
        the model uses a vertical acceleration term a_z = a_Wz - g.
        """
        Ts = self.params["Ts"]
        g = self.params["g"]

        xh = self.xh
        Pz = self.Pz

        # vertical accel in world frame (NED +down)
        aB = accel_g * g
        aW = quat_rotate(self.quat, aB)
        a_z = aW[2] - g

        A = np.array([
            [1.0, Ts, -0.5*Ts*Ts],
            [0.0, 1.0, -Ts],
            [0.0, 0.0, 1.0],
        ])

        B = np.array([0.5*Ts*Ts, Ts, 0.0])

        Q = self.params["observer"]["heave_KF"]["Q"]

        self.xh = A @ xh + B * a_z
        self.Pz = A @ Pz @ A.T + Q

    def kf_update_z(self, z_meas, sensor_idx):
        """Apply a scalar measurement update to the heave Kalman filter.

        sensor_idx is the ToF sensor index in [FL, FR, RL, RR] order and selects the
        corresponding measurement noise value `R_i[sensor_idx]`.
        """
        xh = self.xh
        Pz = self.Pz

        R_i = self.params["observer"]["heave_KF"]["R_i"]
        R = float(R_i[sensor_idx])

        H = np.array([1.0, 0.0, 0.0], dtype=float)

        S = float(H @ Pz @ H.T + R)
        K = (Pz @ H.T) / S  # (3,)

        self.xh = xh + K * (float(z_meas) - float(H @ xh))
        self.Pz = (np.eye(3) - np.outer(K, H)) @ Pz

    def velocity_kf_update(self, accel_g, phi, theta, gps_speed, gps_new):
        """Update the forward world-velocity estimate.

        State vector: [velocity, accel_bias].
        The acceleration is transformed from body coordinates to world coordinates
        using the roll and pitch rotation, then integrated with a constant-bias model.
        GPS speed is used as a measurement update when a new packet is detected.
        """

        Ts = float(self.params["Ts"])
        g = float(self.params["g"])

        # Same initialization as MATLAB
        if self.xv is None:
            self.xv = np.array([gps_speed, 0.0], dtype=float)
            self.Pv = np.eye(2, dtype=float)

        # BODY -> WORLD acceleration
        cphi = cos(phi)
        sphi = sin(phi)
        cth = cos(theta)
        sth = sin(theta)

        R_x = np.array([
            [1.0, 0.0,  0.0],
            [0.0, cphi, -sphi],
            [0.0, sphi,  cphi],
        ])

        R_y = np.array([
            [ cth, 0.0, sth],
            [ 0.0, 1.0, 0.0],
            [-sth, 0.0, cth],
        ])

        R_BW = R_y @ R_x

        aB = accel_g * g
        aW = R_BW @ aB

        ax = float(aW[0])

        # Prediction
        A = np.array([
            [1.0, -Ts],
            [0.0,  1.0],
        ])

        B = np.array([
            Ts,
            0.0,
        ])

        Q = np.asarray(
            self.params["observer"]["velocity_KF"]["Q"],
            dtype=float
        )

        self.xv = A @ self.xv + B * ax
        self.Pv = A @ self.Pv @ A.T + Q

        # GPS correction
        if gps_new:
            H = np.array([1.0, 0.0])

            R = float(
                self.params["observer"]["velocity_KF"]["R"]
            )

            S = float(H @ self.Pv @ H.T + R)
            K = (self.Pv @ H.T) / S

            self.xv = self.xv + K * (
                float(gps_speed) - float(H @ self.xv)
            )

            self.Pv = (
                np.eye(2) - np.outer(K, H)
            ) @ self.Pv

        return float(self.xv[0])

    def actuators_estimator_update(self, u):
        """Estimate the actuator states with delayed command handling.

        The model keeps a transport-delay buffer and a first-order actuator dynamics
        update. The returned `delta_hat` uses the same ordering as the runtime input
        commands: [FL, FR, RR].
        """
        u = np.asarray(u, dtype=float).reshape(3)
        # Initialization
        if self.delta_hat is None:
            self.delta_hat = u.copy()
            self.delay_buffer = np.tile(
                u.reshape(3, 1),
                (1, self.actuator_Ld)
            )
        # Saturation
        u = np.clip(
            u,
            self.actuator_alpha_min,
            self.actuator_alpha_max
        )
        # Pure transport delay
        u_delayed = self.delay_buffer[:, -1].copy()
        if self.actuator_Ld > 1:
            self.delay_buffer[:, 1:] = self.delay_buffer[:, :-1].copy()

        self.delay_buffer[:, 0] = u
        # First-order actuator dynamics
        self.delta_hat = (
            self.actuator_Td * self.delta_hat
            + (1.0 - self.actuator_Td) * u_delayed
        )
        # Same ordering as MATLAB delay_buffer(:)
        delay_states = self.delay_buffer.reshape(
            -1,
            order="F"
        ).copy()
        return self.delta_hat.copy(), delay_states

    def step(self, inputs):
        """Perform one observer step and return the runtime state tuple.

        Parameters
        ----------
        inputs : dict
            Dictionary of CAN/IMU values. Expected keys include accelerometer and
            gyroscope readings, ToF distances and statuses, GPS speed, and actuator
            setpoints.

        Returns
        -------
        tuple
            (velocity, z, z_dot, phi, theta, psi, p, q, r, delta_hat..., delay_states...)
        """

        # # ==============================================================
        # # Inputs readout, and vector forming
        # # ==============================================================
        # ax = float(inputs.get("ACCELEROMETER_X", 0.0))
        # ay = float(inputs.get("ACCELEROMETER_Y", 0.0))
        # az = float(inputs.get("ACCELEROMETER_Z", 0.0))
        # accel_g = np.array([ax, ay, az], dtype=float)  # Those three are coming in at the same time

        # gx = float(inputs.get("GYROSCOPE_X", 0.0))
        # gy = float(inputs.get("GYROSCOPE_Y", 0.0))
        # gz = float(inputs.get("GYROSCOPE_Z", 0.0))
        # gyro_dps = np.array([gx, gy, gz], dtype=float)  # Those three are coming in at the same time
        # gyro_rads = np.deg2rad(gyro_dps)

        # FL = float(inputs.get("DISTANCE_FORE_LEFT", 0.0))
        # status_FL = inputs.get("DISTANCE_FORE_LEFT_STATUS", -1)  # 0=OK anything else=error
        # FR = float(inputs.get("DISTANCE_FORE_RIGHT", 0.0))
        # status_FR = inputs.get("DISTANCE_FORE_RIGHT_STATUS", -1)
        # # KF heave update (only when ToF arrives)
        # tF = inputs.get("DISTANCE_FORE_FEEDBACK_timestamp", None)
        # new_front = (tF is not None) and (tF != self.t_tof_front_prev)
        # if new_front:
        #     self.t_tof_front_prev = tF

        # RL = float(inputs.get("DISTANCE_ACHTER_LEFT", 0.0))
        # status_RL = inputs.get("DISTANCE_ACHTER_LEFT_STATUS", -1)
        # RR = float(inputs.get("DISTANCE_ACHTER_RIGHT", 0.0))
        # status_RR = inputs.get("DISTANCE_ACHTER_RIGHT_STATUS", -1)
        # # KF heave update (only when ToF arrives)
        # tR = inputs.get("DISTANCE_ACHTER_FEEDBACK_timestamp", None)
        # new_rear = (tR is not None) and (tR != self.t_tof_rear_prev)
        # if new_rear:
        #     self.t_tof_rear_prev = tR

        # gps_speed = float(inputs.get("GPS_GROUND_SPEED", 0.0))
        # tGPS = inputs.get("GPS_MOTION_timestamp", None)
        # gps_new = (tGPS is not None) and (tGPS != self.t_gps_prev)
        # if gps_new:
        #     self.t_gps_prev = tGPS

        u_actuator = np.array([
            float(inputs.get("AUTO_CONTROL_FRONT_LEFT_SETPOINT", 0.0)),
            float(inputs.get("AUTO_CONTROL_FRONT_RIGHT_SETPOINT", 0.0)),
            float(inputs.get("AUTO_CONTROL_REAR_SETPOINT", 0.0)),
        ], dtype=float)

        # ==============================================================
        # Replay inputs from MATLAB
        # ==============================================================

        k = min(
            self.replay_sample,
            self.replay_length - 1
        )
        self.replay_sample += 1
        if (k+1) % 100 == 0:
            print(f"{k=}")

        gyro_dps = np.asarray(
            self.replay["gyro"][k, :],
            dtype=float
        ).reshape(3)
        gyro_rads = np.deg2rad(gyro_dps)

        accel_g = np.asarray(
            self.replay["accel"][k, :],
            dtype=float
        ).reshape(3)

        tof_mm = np.asarray(
            self.replay["tof"][k, :],
            dtype=float
        ).reshape(4)
        use = np.asarray(
            self.replay["tof_new"][k, :],
            dtype=int
        ).reshape(4)

        gps_speed = float(np.asarray(self.replay["gps"]).reshape(-1)[k])
        gps_new = bool(np.asarray(self.replay["gps_new"]).reshape(-1)[k])

        # ==============================================================
        # Observer algorithm
        # ==============================================================

        # Mahony attitude
        gyro_corr = self.mahony_update(gyro_rads, accel_g)
        phi, theta, psi = quat_to_euler_BW(self.quat)  # [rad]
        p, q, r = gyro_corr                            # [rad/s]

        # Kalman heave
        self.kf_predict(accel_g)
        # # Build full vectors in MATLAB order: [FL, FR, RL, RR]
        # tof_mm = np.array([FL, FR, RL, RR], dtype=float)
        # # Convert CAN status: 0=OK -> 1=good (simulation convention)
        # tof_status = np.array([
        #     1 if status_FL == 0 else 0,
        #     1 if status_FR == 0 else 0,
        #     1 if status_RL == 0 else 0,
        #     1 if status_RR == 0 else 0,
        # ], dtype=int)
        # # Only update when a NEW packet arrived for that pair
        # can_update = np.array([
        #     1 if new_front else 0,  # FL
        #     1 if new_front else 0,  # FR
        #     1 if new_rear else 0,   # RL
        #     1 if new_rear else 0,   # RR
        # ], dtype=int)
        # use = (can_update == 1) & (tof_status == 1)
        for i in range(4):
            if use[i]:
                z_meas = self.tof_to_z_i(i, tof_mm[i], phi, theta)  # i = 0..3
                self.kf_update_z(z_meas, i)

        # Kalman velocity
        velocity = self.velocity_kf_update(
            accel_g,
            phi,
            theta,
            gps_speed,
            gps_new
        )

        # Actuator estimator
        delta_hat, delay_states = self.actuators_estimator_update(u_actuator)

        # ==============================================================
        # Outputs packing
        # ==============================================================
        z = self.xh[0]
        z_dot = self.xh[1]

        return (
            float(velocity),  # forward WORLD velocity [m/s]
            float(z),       # heave position [m] (NED +down)
            float(z_dot),   # heave velocity [m/s]
            float(phi),     # roll  [rad]
            float(theta),   # pitch [rad]
            float(psi),     # yaw   [rad]
            float(p),       # roll rate  [rad/s]
            float(q),       # pitch rate [rad/s]
            float(r),       # yaw rate   [rad/s]

            # actuator states
            float(delta_hat[0]),   # delta_FL
            float(delta_hat[1]),   # delta_FR
            float(delta_hat[2]),   # delta_R

            # transport-delay states
            *[float(v) for v in delay_states],
        )
