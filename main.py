import numpy as np
from numpy.linalg import inv
from numpy.random import normal


def A(t):
    """ Compute the state transition matrix.

    Keyword arguments:
    t -- the time interval/step (NOT the current time)
    """
    return np.matrix([
        [1, 0, t, 0],
        [0, 1, 0, t],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


def B(t):
    """ Compute the control matrix.
    Express effects of acceleration on position and velocity.

    Keyword arguments:
    t -- the time interval/step (NOT the current time)
    """
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


# tested
def generate_trajectory(initial_state, acceleration, number_of_iters, delta_t):
    """ Generate the true trajectory of the object.

    Keyword arguments:
    initial_state -- [px_0, py_0, vx_0, vy_0]'
    acceleration -- [a_x, a_y]

    """
    states = []
    states.append(initial_state)
    for i in range(1, number_of_iters):
        # state update
        new_state = A(delta_t).dot(states[i - 1]) + B(delta_t).dot(acceleration)
        states.append(new_state)
    return states


# tested
def generate_observations(true_trajectory, sig_obs):
    """ Generate the simulated noisy observations by adding random Gaussian noise
    to the true trajectory.

    Keyword arguments:
    true_trajectory -- a list of true states of the object in motion,
        each a vector in R^4 = [p_x, p_y, v_x, v_y]
    sig_obs -- the covariance matrix of observation noise = [[sigma_x^2, 0], [0, sigma_y^2]]

    """
    obs = []
    n = len(true_trajectory)
    std_dev_px = sig_obs[0, 0]
    std_dev_py = sig_obs[1, 1]
    for i in range(n):
        noise = np.array([normal(0.0, std_dev_px), normal(0.0, std_dev_py)]).reshape(2, 1)
        new_obs = true_trajectory[i][:2] + noise
        obs.append(new_obs)
    return obs


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
    initial_estimation = np.array([0, 0, 0, 0]).reshape((4, 1))
    acceleration = np.array([1, 1]).reshape((2, 1))
    number_of_iters = 10
    delta_t = 1
    sig_acceleration = np.array([0.05, 0.05])  # std dev of noise of a_x, a_y, assume that they are independently distributed
    sig_obs = np.matrix([[1, 0], [0, 1]])  # std dev of noise of measurement (observation) of p_x and p_y
    # kalman(initial_estimation, initial_obs, acceleration, number_of_iters, delta_t, sig_acceleration, sig_obs)
    # true_trajectory = generate_trajectory(initial_estimation, acceleration, number_of_iters, delta_t)
    # obs = generate_observations(true_trajectory, sig_obs)
    # print true_trajectory
    # print obs
