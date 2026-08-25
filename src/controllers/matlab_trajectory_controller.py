from pathlib import Path
from scipy.io import loadmat

"""
To move the trajectory file to the Raspberry Pi, use the following command in PS:
scp "D:\Dane\workspace\dynamics-model-mini-celka\ACTUATORS-CHARACTERIZATION\hydrofoil_actuators_identification_trajectory.mat" brzanpi@192.168.1.49:/home/brzanpi/ws_minicelka/rpi-controller-mini-celka/src/controllers/matlab_trajectory_controller/test_trajectory.mat
"""

class Controller:
    """
    Pre-recorded trajectory controller.

    Each call to step() advances the trajectory by exactly one sample.
    The trajectory is assumed to be generated for a 100 Hz control loop.
    """
    def __init__(self):
        trajectory_path = Path(__file__).parent / "matlab_trajectory_controller/test_trajectory.mat"

        data = loadmat(trajectory_path)
        self.trajectory = data["trajectory"]

        if self.trajectory.shape[0] != 3:
            raise ValueError(
                f"Expected trajectory shape (3, N), "
                f"got {self.trajectory.shape}"
            )

        self.sample = 0
        self.previous_mode = None
        self.active = False

    def step(self, readout: dict, estimated_state: tuple) -> tuple:
        mode = readout.get("RADIO_SYNC_SWITCH", 0)

        # Detect controller activation
        if mode == 2 and self.previous_mode != 2:
            self.sample = 0
            self.active = True

        # Detect controller deactivation
        if mode != 2:
            self.active = False

        self.previous_mode = mode

        if not self.active:
            return (0.0, 0.0, 0.0)

        # End of trajectory -> hold last sample
        if self.sample >= self.trajectory.shape[1]:
            i = self.trajectory.shape[1] - 1
        else:
            i = self.sample
            self.sample += 1

        return (
            float(self.trajectory[0, i]),
            float(self.trajectory[1, i]),
            float(self.trajectory[2, i]),
        )
