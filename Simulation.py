import matplotlib.pyplot as plt


class Simulation(object):

    def __init__(self, true_trajectory, estimation):
        self.plot_true_position_and_estimation(true_trajectory, estimation)

    def plot_true_position_and_estimation(self, true_trajectory, estimation):
        n = len(true_trajectory)
        true_pos_x = []
        true_pos_y = []
        estimated_pos_x = []
        estimated_pos_y = []
        for i in range(n):
            true_pos_x.append(true_trajectory[i][0, 0])
            true_pos_y.append(true_trajectory[i][1, 0])
            estimated_pos_x.append(estimation[i][0, 0])
            estimated_pos_y.append(estimation[i][1, 0])

        max_x = max([max(true_pos_x), max(estimated_pos_x)]) * 1.25
        max_y = max([max(true_pos_y), max(estimated_pos_y)]) * 1.25
        axes = plt.gca()
        axes.set_xlim([0, max_x])
        axes.set_ylim([0, max_y])

        # fig_size = [int(max_x + 1), int(max_y + 1)]
        # plt.rcParams["figure.figsize"] = fig_size

        for i in range(n):
            if i >= 1:
                plt.scatter(true_pos_x[i - 1], true_pos_y[i - 1], s=40, c='w')
                plt.scatter(estimated_pos_x[i - 1], estimated_pos_y[i - 1], s=40, c='0.5')
            plt.scatter(true_pos_x[i], true_pos_y[i], s=40, c='r')
            plt.scatter(estimated_pos_x[i], estimated_pos_y[i], s=40, c='g')
            plt.pause(0.1)
       


        # print true_pos_x
        # print true_pos_y

        plt.show()

        # plt.plot(true_pos_x, true_pos_y, 'ro')
        # plt.plot(estimated_pos_x, estimated_pos_y, 'go')
        # plt.show()
