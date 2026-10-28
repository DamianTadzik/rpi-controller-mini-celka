"""Velocity-scheduled hydrofoil MPC with heave integral action, using OSQP.

Uses the same Controller.step(readout, estimated_state) interface as LQ_controller.py.
The model and operating point are interpolated from ctrl_params.controller.MPC.

The QP uses absolute foil commands [deg]. It is a condensed linear MPC with
hard MV limits and a predicted integral of heave tracking error. This is
an experimental MPCI variant; test against the baseline MPC in PIL before
using it to control the real boat.
"""

from time import perf_counter_ns
import numpy as np
import osqp
from scipy import sparse
from scipy.io import loadmat


class Controller:
    def __init__(self, params_file=None):
        if params_file is None:
            params_file = (
                "/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/"
                "src/controllers/boat_controller_parameters_MPC.mat"
            )

        self.params = loadmat(params_file, simplify_cells=True)["ctrl_params"]
        mpc = self.params["controller"]["MPC"]

        self.velocity_grid = np.asarray(mpc["velocity_grid"], dtype=float).reshape(-1)
        self.A_grid = np.asarray(mpc["Aaug_grid"], dtype=float)
        self.B_grid = np.asarray(mpc["Baug_grid"], dtype=float)
        self.x0_grid = np.asarray(mpc["x0aug_grid"], dtype=float)
        self.u0_grid = np.asarray(mpc["u0_grid"], dtype=float)
        self.C = np.asarray(mpc["C"], dtype=float)
        self.D = np.asarray(mpc["D"], dtype=float)

        self.Ts = float(mpc["Ts"])
        self.Ld = int(mpc["Ld"])
        self.Np = int(mpc["Np"])
        self.Nc = int(mpc["Nc"])
        self.nx, self.nu, self.ny = self.C.shape[1], self.B_grid.shape[1], self.C.shape[0]

        # Tune weights locally, independently of the MAT file.
        # Weight vectors are squared in the QP cost.
        self.Wy = np.array([10.0, 10.0, 10.0])
        self.Wu = np.array([0.01, 0.01, 0.01])
        self.Wdu = np.array([0.25, 0.25, 0.25])
        self.ov_scale = np.ones(self.ny)
        self.mv_scale = np.ones(self.nu)

        # Use physical actuator limits if they are not duplicated in MPC params.
        actuator = self.params["actuator_model"]
        umin = mpc.get("u_min", actuator["alpha_min"])
        umax = mpc.get("u_max", actuator["alpha_max"])
        self.umin = np.broadcast_to(np.asarray(umin, dtype=float).reshape(-1), (self.nu,)).copy()
        self.umax = np.broadcast_to(np.asarray(umax, dtype=float).reshape(-1), (self.nu,)).copy()

        assert self.nx == 21 and self.nu == 3 and self.ny == 3
        assert self.Ld == 4 and 1 <= self.Nc <= self.Np
        assert self.A_grid.shape == (self.nx, self.nx, self.velocity_grid.size)
        assert self.B_grid.shape == (self.nx, self.nu, self.velocity_grid.size)
        assert self.x0_grid.shape == (self.nx, self.velocity_grid.size)
        assert self.u0_grid.shape == (self.nu, self.velocity_grid.size)
        assert self.D.shape == (self.ny, self.nu) and np.allclose(self.D, 0.0)
        assert self.velocity_grid.size >= 2 and np.all(np.diff(self.velocity_grid) > 0)
        assert np.all(self.umin < self.umax)
        assert np.all(self.ov_scale > 0) and np.all(self.mv_scale > 0)
        assert all(v.size == n for v, n in ((self.Wy, self.ny), (self.Wu, self.nu), (self.Wdu, self.nu)))

        # Decision variables are Nc consecutive ABSOLUTE commands [FL, FR, R].
        nv = self.nu * self.Nc
        self.nv = nv
        self.lb = np.tile(self.umin, self.Nc)
        self.ub = np.tile(self.umax, self.Nc)

        # Differences: [u(k)-u(k-1), u(k+1)-u(k), ...].
        self.Du = np.eye(nv)
        for j in range(1, self.Nc):
            self.Du[j*self.nu:(j+1)*self.nu, (j-1)*self.nu:j*self.nu] = -np.eye(self.nu)

        # Over the prediction horizon, commands are held after the Nc-th move.
        self.Hu = np.zeros((self.Np * self.nu, nv))
        for k in range(self.Np):
            j = min(k, self.Nc - 1)
            self.Hu[k*self.nu:(k+1)*self.nu, j*self.nu:(j+1)*self.nu] = np.eye(self.nu)

        self.Qy_diag = np.tile((self.Wy / self.ov_scale)**2, self.Np)
        self.Qu_diag = np.tile((self.Wu / self.mv_scale)**2, self.Np)
        self.Rdu_diag = np.tile((self.Wdu / self.mv_scale)**2, self.Nc)

        # MPCI: accumulate and predict heave tracking error [m*s].
        # This is a COST term, not an artificial shift of the heave reference.
        self.Wi_heave = 10.0
        self.xi_limit = 0.1           # [m*s], simple integrator clamping
        self.xi_heave = 0.0

        # Ei maps stacked predicted [z, phi, theta] errors to the predicted
        # integral values at all Np steps: xi_j = xi_now + Ts*sum(e_z,1..j).
        self.Ei = self.Ts * np.kron(
            np.tril(np.ones((self.Np, self.Np))),
            np.array([[1.0, 0.0, 0.0]]),
        )
        self.Qi_diag = np.full(self.Np, self.Wi_heave**2)

        self.P_fixed = self.Du.T @ (self.Rdu_diag[:, None] * self.Du)
        self.P_fixed += self.Hu.T @ (self.Qu_diag[:, None] * self.Hu)

        # Keep the COMPLETE upper-triangular sparsity pattern across all speeds.
        # OSQP.update(Px=...) then updates a QP without creating a new solver.
        self._p_cols = np.repeat(np.arange(nv), np.arange(1, nv + 1))
        self._p_rows = np.concatenate([np.arange(j + 1) for j in range(nv)])

        A0, B0, _, _ = self._scheduled_model(float(self.velocity_grid[0]))
        _, G0 = self._prediction_matrices(A0, B0)
        P0 = self._hessian(G0)
        P_sparse = sparse.csc_matrix(
            (P0[self._p_rows, self._p_cols], (self._p_rows, self._p_cols)),
            shape=(nv, nv),
        )

        self.solver = osqp.OSQP()
        self.solver.setup(
            P=P_sparse,
            q=np.zeros(nv),
            A=sparse.eye(nv, format="csc"),
            l=self.lb,
            u=self.ub,
            verbose=False,
            warm_starting=True,
            eps_abs=1e-4,
            eps_rel=1e-4,
            max_iter=400,
        )

        self.last_solver_status = None
        self.last_step_ms = 0.0
        # Same hysteresis as Simulink mpc_enable_logic.
        self.mpc_enabled = False
        self.last_command = None

    def _scheduled_model(self, velocity):
        """Linear interpolation with clamping at the grid endpoints."""
        i = int(np.clip(np.searchsorted(self.velocity_grid, velocity, side="right") - 1,
                        0, self.velocity_grid.size - 2))
        v1, v2 = self.velocity_grid[i:i+2]
        mu = float(np.clip((velocity - v1) / (v2 - v1), 0.0, 1.0))

        A = (1 - mu) * self.A_grid[:, :, i] + mu * self.A_grid[:, :, i + 1]
        B = (1 - mu) * self.B_grid[:, :, i] + mu * self.B_grid[:, :, i + 1]
        x0 = (1 - mu) * self.x0_grid[:, i] + mu * self.x0_grid[:, i + 1]
        u0 = (1 - mu) * self.u0_grid[:, i] + mu * self.u0_grid[:, i + 1]
        return A, B, x0, u0

    def _prediction_matrices(self, A, B):
        """y_pred - y_nom = F @ (x - x0) + G @ (U - U0)."""
        F = np.zeros((self.Np * self.ny, self.nx))
        G = np.zeros((self.Np * self.ny, self.nv))
        Ap = np.eye(self.nx)
        S = np.zeros((self.nx, self.nv))

        for k in range(self.Np):
            Ap = A @ Ap
            S = A @ S
            j = min(k, self.Nc - 1)
            S[:, j*self.nu:(j+1)*self.nu] += B
            F[k*self.ny:(k+1)*self.ny, :] = self.C @ Ap
            G[k*self.ny:(k+1)*self.ny, :] = self.C @ S
        return F, G

    def _hessian(self, G):
        Gi = self.Ei @ G
        H = G.T @ (self.Qy_diag[:, None] * G) + self.P_fixed
        H += Gi.T @ (self.Qi_diag[:, None] * Gi)
        return H + H.T  # OSQP's 0.5 * U.T @ P @ U convention

    @staticmethod
    def _reference(readout):
        # Exactly the radio-to-roll-reference mapping used by LQ_controller.py.
        steering = float(readout.get("RADIO_STEERING", 0.0))
        if steering >= 10.0:
            roll_deg = (steering - 10.0) * 3.0 / 990.0
        elif steering <= -10.0:
            roll_deg = (steering + 10.0) * 3.0 / 990.0
        else:
            roll_deg = 0.0

        return np.array([
            float(readout.get("REFERENCE_Z", -0.1)),
            np.deg2rad(roll_deg),
            float(readout.get("REFERENCE_THETA", 0.0)),  # radians, as in LQ
        ], dtype=float)

    def step(self, readout: dict, estimated_state: tuple):
        """Return (FL, FR, R) commanded angles [deg] for one 100 Hz tick."""
        if estimated_state is None:
            return (0.0, 0.0, 0.0)

        t_start = perf_counter_ns()
        state = np.asarray(estimated_state, dtype=float).reshape(-1)
        if state.size != 12 + self.nu * self.Ld or not np.all(np.isfinite(state)):
            raise ValueError(f"Expected {12 + self.nu*self.Ld} finite observer values, got {state.size}")

        velocity = float(state[0])
        reference = self._reference(readout)
        if not np.all(np.isfinite(reference)):
            raise ValueError("Invalid MPC reference")

        # Observer output:
        # [velocity, z, zdot, phi, theta, psi, p, q, r,
        #  delta_FL, delta_FR, delta_R,  delay_states (newest -> oldest)]
        physical = state[[1, 2, 3, 4, 6, 7, 9, 10, 11]]

        # Hysteresis: switch MPC on at >2.4 m/s, off at <1.9 m/s.
        if velocity < 1.9:
            self.mpc_enabled = False
        elif velocity > 2.4:
            self.mpc_enabled = True

        if not self.mpc_enabled:
            # Clear integral while MPC is disabled (takeoff mode).
            self.xi_heave = 0.0
            # Simple takeoff controller, as in Simulink.
            collective = 3.0
            phi = state[3]  # estimated roll [rad]
            differential = -1.2 * np.rad2deg(phi)
            u = np.clip(
                np.array([collective + differential,
                          collective - differential,
                          collective]),
                self.umin, self.umax,
            )
            self.last_command = u.copy()
            self.last_solver_status = "disabled"
            self.last_step_ms = (perf_counter_ns() - t_start) / 1e6
            return tuple(float(v) for v in u)

        newest_first = state[12:].reshape(self.Ld, self.nu)
        delayed = newest_first[::-1].reshape(-1)  # oldest -> newest in Aaug
        x = np.concatenate((physical, delayed))

        # Most recent absolute command: needed for first MV-rate penalty.
        u_previous = (
            self.last_command.copy()
            if self.last_command is not None
            else newest_first[0].copy()
        )

        A, B, x0, u0 = self._scheduled_model(velocity)
        F, G = self._prediction_matrices(A, B)

        # Prediction in deviation coordinates; optimization in absolute MV.
        # Absolute references and absolute command limits are preserved.
        # Update measured heave-error integral at the current sample.
        # NED convention: positive error means boat is too low in the water.
        self.xi_heave += self.Ts * (state[1] - reference[0])
        self.xi_heave = float(np.clip(self.xi_heave, -self.xi_limit, self.xi_limit))

        x_dev = x - x0
        r_dev = reference - self.C @ x0
        e0 = F @ x_dev - G @ np.tile(u0, self.Nc) - np.tile(r_dev, self.Np)

        # Predicted integral over the MPC horizon: xi_pred = bi + Gi @ U.
        Gi = self.Ei @ G
        bi = self.xi_heave + self.Ei @ e0

        # First move is relative to most recent commanded foil angles.
        previous = np.zeros(self.nv)
        previous[:self.nu] = u_previous

        q = 2.0 * (
            G.T @ (self.Qy_diag * e0)
            - self.Du.T @ (self.Rdu_diag * previous)
            - self.Hu.T @ (self.Qu_diag * np.tile(u0, self.Np))
            + Gi.T @ (self.Qi_diag * bi)
        )
        P = self._hessian(G)

        self.solver.update(Px=P[self._p_rows, self._p_cols], q=q)
        solution = self.solver.solve()
        self.last_solver_status = solution.info.status

        if solution.info.status_val != 1:
            # Never silently accept infeasible or inaccurately solved QPs.
            # Runtime should catch the failure and fall back to safe control.
            raise RuntimeError(f"MPC/OSQP failed: {solution.info.status}")

        u = np.clip(solution.x[:self.nu], self.umin, self.umax)
        self.last_command = u.copy()
        self.last_step_ms = (perf_counter_ns() - t_start) / 1e6
        return tuple(float(v) for v in u)
