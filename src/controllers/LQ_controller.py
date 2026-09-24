from math import radians
from controllers.control_helpers import saturate
import numpy as np
from scipy.io import loadmat


class Controller:
    """Discrete augmented LQ hydrofoil controller.

    This is a direct port of the MATLAB implementation.
    The controller loads the same parameter file used by the observer,
    performs gain scheduling over the current velocity, and applies the
    augmented state feedback with an integral state on the selected outputs.
    """

    def __init__(self):
        params_file = (
            "/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/"
            "src/observers/boat_controller_parameters.mat"
        )

        data = loadmat(params_file, simplify_cells=True)
        self.params = data["ctrl_params"]

        self.Ts = float(self.params["Ts"])

        # Gain scheduling data
        self.velocity_grid = np.asarray(
            self.params["controller"]["LQ"]["velocity_grid"],
            dtype=float,
        )
        self.K_grid = np.asarray(
            self.params["controller"]["LQ"]["K_aug_lqi_grid"],
            dtype=float,
        )
        self.x0_grid = np.asarray(
            self.params["controller"]["LQ"]["x0aug_grid"],
            dtype=float,
        )
        self.u0_grid = np.asarray(
            self.params["controller"]["LQ"]["u0_grid"],
            dtype=float,
        )
        self.Nv = int(self.velocity_grid.size)

        # Persistent controller state
        self.xi = np.zeros(3, dtype=float)
        self.integral_enabled = False

    def _select_lq_schedule(self, velocity):
        """Return scheduled K, x0, and u0 for the current velocity."""
        # Clamp below scheduling range
        if velocity <= self.velocity_grid[0]:
            K = self.K_grid[:, :, 0]
            x0 = self.x0_grid[:, 0].copy()
            u0 = self.u0_grid[:, 0].copy()
            return K, x0, u0

        # Clamp above scheduling range
        if velocity >= self.velocity_grid[self.Nv - 1]:
            K = self.K_grid[:, :, self.Nv - 1]
            x0 = self.x0_grid[:, self.Nv - 1].copy()
            u0 = self.u0_grid[:, self.Nv - 1].copy()
            return K, x0, u0

        # Find V_i <= V < V_{i+1}
        idx = 0
        for i in range(self.Nv - 1):
            if velocity >= self.velocity_grid[i] and velocity < self.velocity_grid[i + 1]:
                idx = i
                break

        V1 = self.velocity_grid[idx]
        V2 = self.velocity_grid[idx + 1]

        # Linear interpolation coefficient
        mu = (velocity - V1) / (V2 - V1)

        # Interpolate controller and trim point
        K = (1.0 - mu) * self.K_grid[:, :, idx] + mu * self.K_grid[:, :, idx + 1]
        x0 = (1.0 - mu) * self.x0_grid[:, idx] + mu * self.x0_grid[:, idx + 1]
        u0 = (1.0 - mu) * self.u0_grid[:, idx] + mu * self.u0_grid[:, idx + 1]
        return K, x0, u0

    def step(self, readout: dict, estimated_state: tuple):
        """Execute one controller step.

        Parameters
        ----------
        readout : dict
            Latest decoded CAN readout. The controller expects the reference and
            current scheduling values to be present in the readout if used in the
            runtime. A fallback to the observer estimate is used when the exact
            MATLAB-style reference is not available.

        estimated_state : tuple
            Current observer output.

        Returns
        -------
        tuple
            (front_left_setpoint, front_right_setpoint, rear_setpoint)
        """
        # Apply commanded references
        # reference:
        # [z_ref;
        #  phi_ref;
        #  theta_ref]
        reference = np.array([
            float(readout.get("REFERENCE_Z", -0.1)),
            float(readout.get("REFERENCE_PHI", 0.0)),
            float(readout.get("REFERENCE_THETA", 0.0)),
        ], dtype=float)

        # Observer output:
        # [velocity,
        #  z, zdot, phi, theta, psi, p, q, r,
        #  delta_FL, delta_FR, delta_R,
        #  delay_states...]

        if estimated_state is None:
            return (0.0, 0.0, 0.0)

        state = np.asarray(estimated_state, dtype=float)

        # Scheduling variable
        velocity = float(state[0])

        # Controller state:
        # [z zdot phi theta p q delta_FL delta_FR delta_R delay_states...]
        x = np.concatenate([
            state[[1, 2, 3, 4, 6, 7, 9, 10, 11]],
            state[12:]
        ])

        # Gain scheduling
        K, x0, u0 = self._select_lq_schedule(velocity)

        # reference values override trim point entries
        x0[0] = reference[0]   # zW / heave
        x0[2] = reference[1]   # phi / roll
        x0[3] = reference[2]   # theta / pitch

        # Integrate the roll pitch and heave error numerically
        # persistent xi
        # if isempty(xi)
        #     xi = zeros(3,1);
        # end

        # Integrator on off switch with hysteresis
        V_DISABLE = 1.9
        V_ENABLE = 2.4
        if velocity < V_DISABLE:
            self.xi[:] = 0.0
            self.integral_enabled = False
        elif velocity > V_ENABLE:
            self.integral_enabled = True

        if self.integral_enabled:
            # Integrate all
            self.xi = self.xi + self.Ts * np.array([
                x[0] - reference[0],   # zW / heave
                x[2] - reference[1],   # phi / roll
                x[3] - reference[2],   # theta / pitch
            ], dtype=float)
        else:
            # Dont integrate heave and pitch
            # xi = xi + params.Ts * [ ...
            #     0; ... x(1) - reference(1); ... % zW / heave
            #     0; ... x(3) - reference(2); ... % phi / roll
            #     0 ... x(4) - reference(3) ...  % theta / pitch
            # ];

            # Experiment with zeroing the zdot movement so controller does not
            # try to cancel when boat is lifting again
            # x(2) = 0.0; % zdot - heave rate
            # x(6) = 0.0; % q - pitch rate

            # Experiment with adding positive reference when not foiling? to
            # encourage foiling again XD
            # x0(4) = x0(4) + deg2rad(2); % nose up reference
            # x0(6) = x0(6) + deg2rad(5); % nose up rate reference xd
            pass

        # State error
        dx = x - x0
        # integral part extension
        dx = np.concatenate([dx, self.xi])
        # LQ control
        du = -K @ dx
        # Full actuator command
        u = u0 + du

        # Experiment, try to take off by placing rear hydrofoil flat.. and
        # using simple feedback controller
        if not self.integral_enabled:
            lambda_ = (velocity - V_DISABLE) / (V_ENABLE - V_DISABLE)
            lambda_ = min(max(lambda_, 0.0), 1.0)
            # u(3) = u(3) - (1-lambda) * u0(3); % deg bias during take off
            u[2] = lambda_ * u[2]

            collective = +3.0
            differential = -1.2 * np.rad2deg(x[2]) - 0.1 * np.rad2deg(x[3])
            u[0] = collective + differential
            u[1] = collective - differential
            # u(1) = (collective + differential) * (1 - lambda) + (lambda) * u(1);
            # u(2) = (collective - differential) * (1 - lambda) + (lambda) * u(2);

        # Return actuator command in degrees
        return (saturate(float(u[0]), -6.0, 12.0), saturate(float(u[1]), -6.0, 12.0), saturate(float(u[2]), 6.0, 12.0))
