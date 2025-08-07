import numpy as np

class KalmanFilter3DProjectile:
    def __init__(self, dt, process_noise_std, measurement_noise_std, g=-9.81):
        self.dt = dt
        self.g = g
        self.initialized = False

        self.F = np.array([
            [1, 0, 0, dt, 0,  0 ],
            [0, 1, 0, 0,  dt, 0 ],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0 ],
            [0, 0, 0, 0,  1,  0 ],
            [0, 0, 0, 0,  0,  1 ]
        ])

        self.B = np.array([[0], [0], [0.5 * dt**2], [0], [0], [dt]])
        self.u = np.array([[g]])

        self.H = np.eye(3, 6)
        self.R = measurement_noise_std**2 * np.eye(3)
        q = process_noise_std**2
        self.Q = q * np.block([
            [np.eye(3) * (dt**4)/4, np.eye(3) * (dt**3)/2],
            [np.eye(3) * (dt**3)/2, np.eye(3) * (dt**2)]
        ])
        self.P = np.eye(6)
        self.x = np.zeros((6, 1))

    def initialize_from_positions(self, positions):
        positions = np.array(positions)
        # Estimate initial velocity via finite differences
        vels = (positions[1:] - positions[:-1]) / self.dt
        vel_mean = np.mean(vels, axis=0)

        self.x = np.array([
            [positions[-1][0]],
            [positions[-1][1]],
            [positions[-1][2]],
            [vel_mean[0]],
            [vel_mean[1]],
            [vel_mean[2]]
        ])
        self.initialized = True

    def predict(self):
        self.x = self.F @ self.x + self.B @ self.u
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:3].flatten()  # Return predicted position

    def update(self, z):
        z = np.reshape(z, (3, 1))
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

def predict_to_height(positions_3d, z_final=0.0, dt=0.033, max_steps=300):
    """
    Takes last n 3D positions of a ball, predicts future trajectory
    until it reaches z_final.
    """
    kf = KalmanFilter3DProjectile(dt=dt, process_noise_std=0.1, measurement_noise_std=0.5)
    kf.initialize_from_positions(positions_3d)

    # Optional: update on all given measurements
    for pos in positions_3d:
        kf.predict()
        kf.update(pos)

    trajectory = []
    for _ in range(max_steps):
        pos = kf.predict()
        trajectory.append(pos)
        if pos[1] <= z_final:
            break

    return np.array(trajectory)
