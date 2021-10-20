'''
####################
EN605.613 - Introduction to Robotics
Assignment 5
Linear Quadratice Regulator
-------
For this assignment you must implement the DifferentialDrive class, get_R, get_Q, and dLQR function.

Use the vehicle kinematics defined in Lecture 3. 

==================================
Copyright 2020,
The Johns Hopkins University Applied Physics Laboratory LLC (JHU/APL).
All Rights Reserved.
####################
'''

import sys
import numpy as np
import scipy.linalg as la
import control

"""
Implementation of Differential Drive kinematics.
This represents a two-wheeled vehicle defined by the following states
state = [x,y,theta]
and accepts the following control inputs
input = [left_wheel_rotation_rate,right_wheel_rotation_rate]
"""
class DifferentialDrive(object):

    """
    Initializes the class
    Input
      :param L: The distance between the wheels
      :param R: The radius of the wheels
    """
    def __init__(self, L=1, R=1):
        self.L = L
        self.R = R
        self.u_limit = 1 / R
        self.V = None

    def get_state_size(self):
        return 3

    def get_input_size(self):
        return 2

    '''
    This function provides the covariance matrix V which describes the noise that can be applied to the forward kinematics.

    Feel free to experiment with different levels of noise.

      Output
        :return: V: input cost matrix
    '''
    def get_V(self):
        if self.V is None:
            self.V = np.eye(3)
            self.V[0, 0] = 0
            self.V[1, 1] = 0
            self.V[2, 2] = 0.075
        return self.V

    """
    Computes the forward kinematics for the system.
    
    Input
      :param x0: The starting state (position) of the system (units:[m,m,rad]) -> np.array with shape (3,)
      :param u: The control input to the system (e.g. wheel rotation rates) (units: [rad/s,rad/s]) -> np.array with shape (2,)
      :param v: The noise applied to the system (units:[m/s, m/s, rad/s^2]) -> np.array with shape (3,)
    
    Output
      :return: x1: The new state of the system
    """
    # 20 points: Implement this function
    def forward(self, x, u, v, dt):

        A, B = self.linearize(x, u)

        # scaling A based on dt
        A[0, 2] = A[0, 2] * dt
        A[1, 2] = A[1, 2] * dt
        A[2, 2] = dt

        # scaling B based on dt
        B = B * dt

        Ax = np.matmul(A, x) # A * x
        Bu = np.matmul(B, u) # B * u

        x1 = np.add(Ax, Bu) # x1 = Ax + Bu

        return x1

    """
    Computes the first order jacobian for A and B around the state x and input u
    
    Input
      :param x: The state of the system  -> np.array with shape (3,)
      :param u: The control input to the system (e.g. wheel rotation rates) -> np.array with shape (2,)
    
    Output
      :return: A: The Jacobian of the kinematics with respect to x 
      :return: B: The Jacobian of the kinematics with respect to u
    """
    # 20 points: Implement this function
    def linearize(self, x, u):

        theta = x[2, 0] # theta as taken from state vector
        sin_theta = np.sin(theta)
        cos_theta = np.sin(theta)
        R_div_2 = self.R / 2
        R_div_L = self.R / self.L
        v_left = u[0, 0]
        v_right = [1, 0]

        # calculate A matrix
        A = np.eye(3)
        A[0, 2] = -R_div_2 * sin_theta * (v_left + v_right)
        A[1, 2] = R_div_2 * cos_theta * (v_left + v_right)

        # calculate B matrix
        B = np.zeros((3, 2))
        B[0, 0] = R_div_2 * cos_theta
        B[0, 1] = R_div_2 * cos_theta
        B[1, 0] = R_div_2 * sin_theta
        B[1, 1] = R_div_2 * sin_theta
        B[2, 0] = -R_div_L
        B[2, 1] = R_div_L

        return A, B

'''
discrete-time linear quadratic regulator for a non-linear system.

Compute the optimal control given a nonlinear system, cost matrices, a current state, and a final state.

Solve for P using the dynamic programming method.

Assume that Qf = Q

If you are encountering errors with the inverse function you may use the pseudoinverse instead.
Hint: Making additional helper functions may be useful

  Input
    :param F: The dynamics class object (has forward and linearize functions implemented)
    :param Q: The cost-to-go matrix Q -> np.array with shape (3,3)
    :param R: The input cost matrix R -> np.array with shape (2,2)
    :param x: The current state of the system x -> np.array with shape (3,)
    :param xf: The desired state of the system xf -> np.array with shape (3,)
    :param dt: The size of the timestep -> float

  Output
    :return: u: Optimal action u for the current state
'''
# 50 points: Implement this function
def dLQR(F, Q, R, x, xf, dt):

    N = 100 # number of steps
    x_dist = np.subtract(x, xf)
    A, B = F.linearize(x, ????)

    P = [] * (N + 1)




    raise NotImplementedError

'''
This function provides the R matrix to the lqr_steer_control and lqr_ekf_control simulators.

Returnss the input cost matrix R.

Experiment with different gains to see their effect on the vehicle's behavior.

  Output
    :return: R: input cost matrix
'''


# 5 points: Implement this function
def get_R():
    return np.eye(2)

'''
This function provides the Q matrix to the lqr_steer_control and lqr_ekf_control simulators.

Returns the input cost matrix R.

Experiment with different gains to see their effect on the vehicle's behavior.

  Output
    :return: Q: State cost matrix
'''
# 5 points: Implement this function
def get_Q():
    return np.eye(3)
