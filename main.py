import numpy as np
from KalmanFilter import *

if __name__ == "__main__":
    # initial_estimation = np.array([0, 0, 0, 0]).reshape((4, 1))
    # acceleration = np.array([1, 1]).reshape((2, 1))
    # number_of_iters = 10
    # delta_t = 1
    # sig_acceleration = np.array([1, 1])  # std dev of noise of a_x, a_y, assume that they are independently distributed
    # sig_obs = np.matrix([[1, 0], [0, 1]])  # std dev of noise of measurement (observation) of p_x and p_y
    kalman = KalmanFilter()
    print kalman.get_difference()
