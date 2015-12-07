import numpy as np


# state transition matrix
def A(t):
    return np.matrix([[1, 0, t, 0], [0, 1, 0, t], [0, 0, 1, 0], [0, 0, 0, 1]])


# control matrix
def B(t):
    return np.matrix([[0.5 * t * t, 0], [0, 0.5 * t * t], [t, 0], [0, t]])


def kalman(initial_state, initial_obs, acceleration, number_of_iters, delta_t, sig_acceleration, sig_obs):
    C = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0]])  # measurement matrix
    P = np.zeros((4, 4), np.float_)
    states = np.array()
    obs = np.array()
    states.append(initial_state)
    obs.append(initial_obs)
    for i in range(1, number_of_iters):
        t = delta_t * i
        # state update
        new_state = np.dot(A(t), states[i - 1]) + np.dot(B(t), acceleration)
        states.append(new_state)

    



if __name__ == "__main__":
    initial_state = np.array([0, 0, 0, 0])
    initial_obs = np.array([0, 0])
    acceleration = np.array([1, 1])
    number_of_iters = 50
    delta_t = 0.1
    sig_acceleration = np.array([0.05, 0.05])  # std dev of noise of a_x, a_y, assume that they are independently distributed
    sig_obs = np.array([1, 1])  # std dev of noise of measurement (observation) of p_x and p_y
    kalman(initial_state, initial_obs, acceleration, number_of_iters, delta_t, sig_acceleration, sig_obs)
    # print control_signal(5)
