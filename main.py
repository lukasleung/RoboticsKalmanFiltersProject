import numpy as np
from KalmanFilter import *
from Simulation import *
from sympy import *
x, y = symbols('x y')


def functions():
    f_x = 100 * sin(x)
    f_y = -100 * cos(y)
    return (f_x, f_y)

if __name__ == "__main__":
    fncs = functions()
    print "\nAcceleration functions:\nf(x) = " + str(fncs[0])
    print "f(y) = " + str(fncs[1])
    print "\n"
    kalman = KalmanFilter(
        true_initial_state=np.array([10, 10, 2, 2]).reshape(4, 1),
        number_of_iters=50,
        acceleration_function_x=fncs[0],
        acceleration_function_y=fncs[1],
        sig_acceleration=np.array([2.5, 2.5])
    )
    sim = Simulation(
        true_trajectory=kalman.get_true_trajectory(),
        estimation=kalman.get_estimation(),
        difference=kalman.get_difference(),
        delta_t=kalman.get_delta_t()
    )
