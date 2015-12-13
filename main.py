import numpy as np
from KalmanFilter import *
from Simulation import *
from sympy import *
x, y = symbols('x y')

if __name__ == "__main__":
    kalman = KalmanFilter(
        true_initial_state=np.array([100, 3000, 100, 30]).reshape(4, 1),
        number_of_iters=50,
        acceleration_function_x=0*x + 5,
        acceleration_function_y=0*y + 5,
        sig_acceleration=np.array([2.5, 2.5])
    )
    sim = Simulation(
        true_trajectory=kalman.get_true_trajectory(),
        estimation=kalman.get_estimation(),
        difference=kalman.get_difference(),
        delta_t=kalman.get_delta_t()
    )
