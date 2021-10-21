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

        A, B = self.linearize(x, u, dt)

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
    def linearize(self, x, u, dt):

        theta = x[2] # theta as taken from state vector
        sin_theta = np.sin(theta)
        cos_theta = np.sin(theta)
        R_div_2 = self.R / 2
        R_div_L = self.R / self.L
        v_left = u[0]
        v_right = u[1]

        # calculate A matrix
        A = np.eye(3)
        A[0, 2] = -R_div_2 * sin_theta * (v_left + v_right) * dt
        A[1, 2] = R_div_2 * cos_theta * (v_left + v_right) * dt
        A[2, 2] = dt

        # calculate B matrix
        B = np.zeros((3, 2))
        B[0, 0] = R_div_2 * cos_theta
        B[0, 1] = R_div_2 * cos_theta
        B[1, 0] = R_div_2 * sin_theta
        B[1, 1] = R_div_2 * sin_theta
        B[2, 0] = -R_div_L
        B[2, 1] = R_div_L
        B = np.multiply(B, dt)

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

    u0 = np.array([0, 0])
    A, B = F.linearize(x, u0, dt)
    AT = np.transpose(A)
    BT = np.transpose(B)

    x_dist = np.subtract(x, xf)
    N = 5 # number of steps
    P = [0] * (N + 1)
    Qf = Q
    P[N] = Qf

    for i in range(N, 0, -1):
       AT_Pi_A = np.matmul(np.matmul(AT, P[i]), A) # A^T * P[i] * A
       AT_Pi_B = np.matmul(np.matmul(AT, P[i]), B) # A^T * P[i] * B
       BT_Pi_B = np.matmul(np.matmul(BT, P[i]), B) # B^T * P[i] * B
       BT_Pi_A = np.matmul(np.matmul(BT, P[i]), A) # B^T * P[i] * A

       R_BT_Pi_B = np.add(R, BT_Pi_B) # R + (B^T * P[i] * B)
       R_BT_Pi_B_inv = np.linalg.inv(R_BT_Pi_B) # (R + (B^T * P[i] * B))^-1
       Q_AT_Pi_A = np.add(Q, AT_Pi_A) # Q + (A^T * P[i] * A)

       P[i - 1] = np.subtract(Q_AT_Pi_A, np.matmul(np.matmul(AT_Pi_B, R_BT_Pi_B_inv), BT_Pi_A))

    K = [0] * N
    u = [0] * N

    for i in range(0, N):
        BT_Piplus1_B = np.matmul(np.matmul(BT, P[i + 1]), B) # B^T * P[i + 1] * B
        BT_Piplus1_A = np.matmul(np.matmul(BT, P[i + 1]), A) # B^T * P[i + 1] * A
        R_BT_Piplus1_B = np.add(R, BT_Piplus1_B) # R + (B^T * P[i + 1] * B)
        R_BT_Piplus1_B_inv = np.linalg.inv(R_BT_Piplus1_B) # (R + (B^T * P[i + 1] * B))^-1
        neg_R_BT_Piplus1_B_inv = np.multiply(-1, R_BT_Piplus1_B_inv) # -(R + (B^T * P[i + 1] * B))^-1
        K[i] = np.matmul(neg_R_BT_Piplus1_B_inv, BT_Piplus1_A)
        u[i] = np.matmul(K[i], x_dist)

    u_opt = u[N - 1]

    return u_opt

'''
This function provides the R matrix to the lqr_steer_control and lqr_ekf_control simulators.

Returnss the input cost matrix R.

Experiment with different gains to see their effect on the vehicle's behavior.

  Output
    :return: R: input cost matrix
'''


# 5 points: Implement this function
def get_R():
    R = [
         [0.1, 0],
         [0, 0.1]
        ]
    R = np.reshape(R, (2, 2))
    return R

'''
This function provides the Q matrix to the lqr_steer_control and lqr_ekf_control simulators.

Returns the input cost matrix R.

Experiment with different gains to see their effect on the vehicle's behavior.

  Output
    :return: Q: State cost matrix
'''
# 5 points: Implement this function
def get_Q():
    Q = [
         [1, 0, 0],
         [0, 1, 0],
         [0, 0, 1]
        ]
    Q = np.reshape(Q, (3, 3))
    return Q
