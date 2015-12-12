import matplotlib.pyplot as plt


class Simulation(object):

    def __init__(self, true_trajectory, estimation):
        
        self.true_trajectory = true_trajectory
        self.estimation = estimation

