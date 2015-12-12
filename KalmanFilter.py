import numpy as np
from numpy.linalg import inv
from numpy.random import normal


class KalmanFilter(object):

    def __init__(self, true_initial_state=np.array([10, 10, 0, 0]).reshape(4, 1), initial_estimation=np.array([0, 0, 0, 0]).reshape((4, 1)), acceleration=np.array([1, 1]).reshape((2, 1)), number_of_iters=10, delta_t=1, sig_acceleration=np.array([0.1, 0.1]), var_obs=np.matrix([[1, 0], [0, 1]])):
        self.true_trajectory = self.generate_trajectory(true_initial_state, acceleration, number_of_iters, delta_t)
        self.obs = self.generate_observations(self.true_trajectory, var_obs)
        self.estimations = self.kalman(initial_estimation, self.obs, acceleration, number_of_iters, delta_t, sig_acceleration, var_obs)

    def get_true_trajectory(self):
        return self.true_trajectory

    def get_estimation(self):
        return self.estimations

    def get_difference(self):
        n = len(self.true_trajectory)
        diff = []
        for i in range(n):
            diff.append(self.true_trajectory[i] - self.estimations[i])
        return diff

    def state_transition_matrix(self, t):
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

    def control_matrix(self, t):
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

    def state_noise_covariance_matrix(self, t, sig_acceleration):
        """ System noise covariance matrix
        """
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
    def generate_trajectory(self, initial_state, acceleration, number_of_iters, delta_t):
        """ Generate the true trajectory of the object.

        Keyword arguments:
        initial_state -- [px_0, py_0, vx_0, vy_0]'
        acceleration -- [a_x, a_y]
        """
        states = []
        states.append(initial_state)
        A = self.state_transition_matrix(delta_t)
        B = self.control_matrix(delta_t)
        for i in range(1, number_of_iters):
            # state update
            new_state = A.dot(states[i - 1]) + B.dot(acceleration)
            states.append(new_state)
        return states

    # tested
    def generate_observations(self, true_trajectory, var_obs):
        """ Generate the simulated noisy observations by adding random Gaussian noise
        to the true trajectory.

        Keyword arguments:
        true_trajectory -- a list of true states of the object in motion,
            each a vector in R^4 = [p_x, p_y, v_x, v_y]
        var_obs -- the covariance matrix of observation noise = [[sigma_x^2, 0], [0, sigma_y^2]]
        """
        obs = []
        n = len(true_trajectory)
        std_dev_px = var_obs[0, 0]
        std_dev_py = var_obs[1, 1]
        for i in range(n):
            noise = np.array([normal(0.0, std_dev_px), normal(0.0, std_dev_py)]).reshape(2, 1)
            new_obs = true_trajectory[i][:2] + noise
            obs.append(new_obs)
        return obs

    def kalman(self, initial_estimation, obs, acceleration, number_of_iters, delta_t, sig_acceleration, var_obs):
        """
        """
        estimations = []
        estimations.append(initial_estimation)

        A = self.state_transition_matrix(delta_t)
        A_transpose = A.transpose()
        B = self.control_matrix(delta_t)
        C = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0]])  # measurement matrix
        C_transpose = C.transpose()
        E_X = self.state_noise_covariance_matrix(delta_t, sig_acceleration)
        P = np.zeros((4, 4), np.float_)  # a measure of the estimated accuracy of the state estimate
        I = np.identity(4)

        for i in range(1, number_of_iters):
            # estimation update by prediction
            predicted_estimation = np.dot(A, estimations[i - 1]) + np.dot(B, acceleration)
            # update P
            P = (A.dot(P)).dot(A_transpose) + E_X
            # compute the Kalman gain (4 x 2 matrix)
            inverse = inv(C.dot(P).dot(C_transpose) + var_obs)
            K = P.dot(C_transpose).dot(inverse)
            # compute true estimation based on predicted value and new observation
            true_estimation = predicted_estimation + K.dot(obs[i] - C.dot(predicted_estimation))
            estimations.append(true_estimation)
            # update P again
            P = (I - K.dot(C)).dot(P)

        return estimations

if __name__ == "__main__":
    kalman = KalmanFilter()
    print kalman.get_difference()
