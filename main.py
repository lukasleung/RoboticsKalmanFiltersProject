import numpy as np
from numpy.linalg import inv


# estimation transition matrix
def A(t):
    return np.matrix([
        [1, 0, t, 0],
        [0, 1, 0, t],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


# control matrix
def B(t):
    return np.matrix([
        [0.5*t*t, 0],
        [0, 0.5*t*t],
        [t, 0],
        [0, t]
    ])


# system noise covariance matrix
def E_X(t, sig_acceleration):
    sigma1 = 0.5 * t * t * sig_acceleration[0]
    sigma2 = 0.5 * t * t * sig_acceleration[1]
    sigma3 = t * sig_acceleration[0]
    sigma4 = t * sig_acceleration[1]
    return np.matrix([
        [sigma1 * sigma1, 0, sigma1 * sigma3, 0],
        [0, sigma2 * sigma2, 0, sigma2 * sigma4],
        [sigma1 * sigma3, 0, sigma3 * sigma3, 0],
        [0, sigma2 * sigma4, 0, sigma4 * sigma4]
    ])

def generate_trajectory(initial_state, acceleration, number_of_iters, delta_t):
    # tested
    states = []
    states.append(initial_state)
    for i in range(1, number_of_iters):
        # state update
        new_state = A(delta_t).dot(states[i - 1]) + B(delta_t).dot(acceleration)
        states.append(new_state)

    return states


def kalman(initial_estimation, initial_obs, acceleration, number_of_iters, delta_t, sig_acceleration, sig_obs):
    C = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0]])  # measurement matrix
    C_transpose = C.transpose()
    P = np.zeros((4, 4), np.float_)  # a measure of the estimated accuracy of the state estimate
    estimations = []
    obs = []
    estimations.append(initial_estimation)
    obs.append(initial_obs)
    for i in range(1, number_of_iters):
        t = delta_t * i
        # estimation update
        new_estimation = np.dot(A(t), estimations[i - 1]) + np.dot(B(t), acceleration)
        estimations.append(new_estimation)
        # update P
        A_transpose = A(t).transpose()
        P = (A(t).dot(P)).dot(A_transpose) + E_X(t, sig_acceleration)
        # compute the Kalman gain
        inverse = inv(C.dot(P).dot(C_transpose) + sig_obs)
        K = P.dot(C_transpose).dot(inverse)


if __name__ == "__main__":
    # initial true state and estimation
    initial_estimation = np.matrix([0, 0, 0, 0]).reshape((4, 1))
    # initial_obs = np.array([0, 0]).reshape((2, 1))
    acceleration = np.array([1, 1]).reshape((2, 1))
    number_of_iters = 10
    delta_t = 1
    # sig_acceleration = np.array([0.05, 0.05])  # std dev of noise of a_x, a_y, assume that they are independently distributed
    # sig_obs = np.matrix([[1, 0], [0, 1]])  # std dev of noise of measurement (observation) of p_x and p_y
    # kalman(initial_estimation, initial_obs, acceleration, number_of_iters, delta_t, sig_acceleration, sig_obs)
