

if __name__ == "__main__":
    initial_estimation = np.array([0, 0, 0, 0]).reshape((4, 1))
    acceleration = np.array([1, 1]).reshape((2, 1))
    number_of_iters = 10
    delta_t = 1
    sig_acceleration = np.array([1, 1])  # std dev of noise of a_x, a_y, assume that they are independently distributed
    sig_obs = np.matrix([[1, 0], [0, 1]])  # std dev of noise of measurement (observation) of p_x and p_y
    true_trajectory = generate_trajectory(initial_estimation, acceleration, number_of_iters, delta_t)
    obs = generate_observations(true_trajectory, sig_obs)
    estimations = kalman(initial_estimation, obs, acceleration, number_of_iters, delta_t, sig_acceleration, sig_obs)
    print estimations
    print true_trajectory
    # print obs
