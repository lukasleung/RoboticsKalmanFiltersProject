import numpy as np
from enum import Enum


class Distribution(Enum):
    Gaussian = 1


class NoiseGenerator:

    ''' Initialize a NoiseGenerator object
        dimension: the dimension of the noise (can be 1 for accelerator or 2 for position in this case)
        mean: a list of mean for each dimension
        std_dev: a list of standard deviation for each dimension
        Here we assume the 2-d distribution is independent
        i.e Cov(X, Y) = 0
    '''
    def __init__(self, dimension, mean, std_dev, type=Distribution.Gaussian):
        self.dimension = dimension
        self.mean = mean
        self.std_dev = std_dev

    def get_mean(self):
        return self.mean

    def get_std_dev(self):
        return self.std_dev

    def get_error(self):
        if self.dimension == 1:
            return self.std_dev[0]**2
        elif self.dimension == 2:
            return np.matrix([self.std_dev[0]**2, self.std_dev[1]**2])


if __name__ == "__main__":
    pass
