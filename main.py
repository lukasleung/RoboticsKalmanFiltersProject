import numpy as np
from KalmanFilter import *
from Simulation import *

if __name__ == "__main__":
    # initial_estimation = np.array([0, 0, 0, 0]).reshape((4, 1))
    # acceleration = np.array([1, 1]).reshape((2, 1))
    # number_of_iters = 10
    # delta_t = 1
    # sig_acceleration = np.array([1, 1])  # std dev of noise of a_x, a_y, assume that they are independently distributed
    # sig_obs = np.matrix([[1, 0], [0, 1]])  # std dev of noise of measurement (observation) of p_x and p_y
    kalman = KalmanFilter(
        true_initial_state=np.array([100, 3000, 100, 30]).reshape(4, 1),
        number_of_iters=50,
        acceleration=np.array([5, 5]).reshape((2, 1)),
        sig_acceleration=np.array([2.5, 2.5])
    )
    sim = Simulation(
        true_trajectory=kalman.get_true_trajectory(),
        estimation=kalman.get_estimation(),
        difference=kalman.get_difference(),
        delta_t=kalman.get_delta_t()
    )
