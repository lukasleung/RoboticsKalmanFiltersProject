import numpy as np

from sympy import *
x, y = symbols('x y')

# define functions, can be just passed in
def fncs():
    f_x = 10*sin(x-10)
    f_y = y**2+1 #10*cos(y+11)
    return (f_x, f_y)

# returns an array of matrices which are what the acceleration is at each step
def get_acceleration(func_x=1, func_y=1, number_iters=10, delta_t=1):
    acceleration = []
    for i in range(number_iters):
        t = delta_t * i
        print func_x.evalf(subs={x: t})
        print func_y.evalf(subs={y: t})
        print 
        accel_t = np.array([func_x.evalf(subs={x: t}), func_y.evalf(subs={y: t})]).reshape((2, 1))
        acceleration.append(accel_t)
    return acceleration


def test():
    functions = fncs()
    func_x = functions[0]
    func_y = functions[1]
    print functions[0]
    
    acceleration = get_acceleration(func_x, func_y, 10, 1)
    n = len(acceleration)
    for i in range(n):
        print acceleration[i]
    
test()
