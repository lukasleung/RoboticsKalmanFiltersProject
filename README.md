# RoboticsKalmanFiltersProject
Use a Kalman Filter to predict the movement of an object over space through time.

## Objectives
1. Choose an interesting problem to estimate
2. Establish data set to be used and manipulated
3. Create a data generator
  - P_t = P_(t-1) + V_t * t + A_t * (t^2)/2
  - V_t = V_(t-1) + A_t * t
   => Need to generate A_t and *given* P_0, V_0 and the correspoinding *Noise sigma*
4. Code functions for Kalman Filter
⋅⋅1. State Prediction X-bar
  - [[P_t] [V_t]] = [[1 t] [0 1]] * [[P_(t-1)] [V_(t-1)]] + [[(t^2)/2] [t]] * a + E_x
  - E_x = [[(sig_p)^2 sig_p*sig_v] [sig_v*sig_p (sig_v)^2]]
⋅⋅2. Measurment Prediction Z-bar
  - [P_t] = [1 0] * [[P_(t-1)] [V_(t-1)]] + E_z
  - E_z = (sig_z)^2
⋅⋅3. For Noise (E's)
  - Need to get the standard deviation of the error of the acceleration
5. Create visualizer

---

## Requirements
1. Linear Distribution 
2. Data set and error noise must be Gaussian distributed

---

## Resources 
1. What are Kalman Filters?
 - [For Dummies](http://bilgin.esme.org/BitsBytes/KalmanFilterforDummies.aspx)
 - [Georgia Tech Course](https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373)
 
2. Examples
 - [Air Plane](http://www.mathworks.com/help/dsp/examples/estimating-position-of-an-aircraft-using-kalman-filter.html#zmw57dd0e5587)
 - [Pictures](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
 - [MATLAB part1 (2 and 3 linked through video)](https://www.youtube.com/watch?v=FkCT_LV9Syk)
 - [^Code for above^](http://studentdavestutorials.weebly.com/kalman-filter-with-matlab-code.html)
 
---

## Tool Kit
- Coding in Python 3.4.3
