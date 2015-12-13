import matplotlib.pyplot as plt

class DifferenceType(object):
    Px, Py, Vx, Vy = range(4)

class Simulation(object):

    def __init__(self, true_trajectory, estimation, difference, delta_t):
        self.plot_true_position_and_estimation(true_trajectory, estimation)
        self.plot_difference_over_time(difference, delta_t, difference_type=DifferenceType.Px)
        self.plot_difference_over_time(difference, delta_t, difference_type=DifferenceType.Py)
        self.plot_difference_over_time(difference, delta_t, difference_type=DifferenceType.Vx)
        self.plot_difference_over_time(difference, delta_t, difference_type=DifferenceType.Vy)

    def plot_true_position_and_estimation(self, true_trajectory, estimation, pause_time=0.1):
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

        plt.title("Trajectory vs Estimation")
        plt.xlabel("Position-x")
        plt.ylabel("Position-y")
        colors = ['w', '0.75', 'r', 'g']
        prev_true_pos_legend = plt.scatter(-1, -1, s=40, c=colors[0])
        prev_est_legend = plt.scatter(-1, -1, s=40, c=colors[1])
        true_pos_legend = plt.scatter(-1, -1, s=40, c=colors[2])
        est_legend = plt.scatter(-1, -1, s=40, c=colors[3])
        plt.legend(
            [prev_true_pos_legend, prev_est_legend, true_pos_legend, est_legend],
            ['Previous True Positions', 'Previous Estimations', 'Current True Position', 'Current Estimation'],
            loc=2
        )
        max_x = max([max(true_pos_x), max(estimated_pos_x)]) * 1.25
        max_y = max([max(true_pos_y), max(estimated_pos_y)]) * 1.25
        axes = plt.gca()
        axes.set_xlim([0, max_x])
        axes.set_ylim([0, max_y])

        for i in range(n):
            if i >= 1:
                plt.scatter(true_pos_x[i - 1], true_pos_y[i - 1], s=40, c=colors[0])
                plt.scatter(estimated_pos_x[i - 1], estimated_pos_y[i - 1], s=40, c=colors[1])
            plt.scatter(true_pos_x[i], true_pos_y[i], s=40, c=colors[2])
            plt.scatter(estimated_pos_x[i], estimated_pos_y[i], s=40, c=colors[3])
            plt.pause(pause_time)

        plt.show()

    def plot_difference_over_time(self, diff, delta_t, difference_type):
        n = len(diff)
        time = [delta_t * i for i in range(n)]
        max_time = max(time)
        if difference_type == DifferenceType.Px:
            diff_px = [diff[i][0, 0] for i in range(n)]
            plt.title("Difference in Position-x Between Trajectory and Estimation over Time")
            plt.xlabel("Time")
            plt.ylabel("Difference in Position-x")
            axes = plt.gca()
            axes.set_ylim([min(diff_px) * 1.25, max(diff_px) * 1.25])
            axes.set_xlim([-1, max_time])
            plt.scatter(time, diff_px)
            plt.plot(time, diff_px)
            plt.show()
        elif difference_type == DifferenceType.Py:
            diff_py = [diff[i][1, 0] for i in range(n)]
            plt.title("Difference in Position-y Between Trajectory and Estimation over Time")
            plt.xlabel("Time")
            plt.ylabel("Difference in Position-y")
            axes = plt.gca()
            axes.set_ylim([min(diff_py) * 1.25, max(diff_py) * 1.25])
            axes.set_xlim([-1, max_time])
            plt.scatter(time, diff_py)
            plt.plot(time, diff_py)
            plt.show()
        elif difference_type == DifferenceType.Vx:
            diff_vx = [diff[i][2, 0] for i in range(n)]
            plt.title("Difference in Velocity-x Between Trajectory and Estimation over Time")
            plt.xlabel("Time")
            plt.ylabel("Difference in Velocity-x")
            axes = plt.gca()
            axes.set_ylim([min(diff_vx) * 1.25, max(diff_vx) * 1.25])
            axes.set_xlim([-1, max_time])
            plt.scatter(time, diff_vx)
            plt.plot(time, diff_vx)
            plt.show()
        elif difference_type == DifferenceType.Vy:
            diff_vy = [diff[i][3, 0] for i in range(n)]
            plt.title("Difference in Velocity-y Between Trajectory and Estimation over Time")
            plt.xlabel("Time")
            plt.ylabel("Difference in Velocity-y")
            axes = plt.gca()
            axes.set_ylim([min(diff_vy) * 1.25, max(diff_vy) * 1.25])
            axes.set_xlim([-1, max_time])
            plt.scatter(time, diff_vy)
            plt.plot(time, diff_vy)
            plt.show()
