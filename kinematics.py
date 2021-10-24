'''
####################
Sahil Sharma
10/24/21
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

'''
Some notes about my implementation:
I was not able to get the robot to turn and correctly follow the red line.
This result was evident by the fact that my blue dot continued on a forward trajectory without
correctly turning at any point. In the process of attempting to debug I double checked my calculation
for the A and B matrices (the Jacobian calculation) and those seemed correct based on the process laid
out in the lecture notes. I also double checked my forward function but this seemed trivial since it was just
implementing x[t + 1] = A*x[t] + B*u[t] where x[t] and u[t] are provided as parameters and the A, B matrices
are calculated from the linearize function. I also carefully reviewed my dLQR function and remain unsure
where my mistake is since I am following the pseudocode provided in lecture and the instructions/guidance
provided in office hours. I did attempt the use of different Q and Rs, per the assignment direction, but did not note
an appreciable improvement in my robots attempts to follow the desired path.
The symptom of the robot not turning seems to point to a math error somewhere but after
stepping through my software using the debugger and reviewing my implementation of the equations I was not able to
find my mistake. Any feedback or thoughts on what I missed would be greatly appreciated since I feel like my
implementation is close to the right solution.

Answers to Questions asked in assingment:
Since I could not get my simulation working fully I will attempt to answer these questions by speculating, based
on my understanding of cost matrices (Q and R), what the result would have been on my simulation

1) What happens if there is a high cost on yaw and negligible cost on X and Y position?
A: If the cost for yaw is high relative to x and y position then the LQR controller will attempt to reduce yaw error more than
x and y position error. This imbalance will likely lead to the robot generally facing the right direction on the path but
perhaps often not being on the red line due to a lack of emphasis on the x and y position being correct. 

2) What if the cost on input is higher than the cost on the state?
A: If the input cost (R) is higher than the state cost (Q) then the controller will attempt to emphasize limited use of the
motors (reduced left and right wheel rotation speed and usage) to still achieve the desired state error. Since the control input,
left and right wheel speeds, are the only way for the robot to adjust its x, y and yaw, an emphasis by the LQR controller
to limit their use at the expense of achieving the desired x, y, and yaw error will result in less precise path following
and will see the blue dot deviate often from the desired red path.
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
        cos_theta = np.cos(theta)

        R_div_2 = self.R / 2
        R_div_L = self.R / self.L

        v_left = u[0]
        v_right = u[1]

        # calculate A matrix
        A = np.eye(3)
        A[0, 2] = -R_div_2 * sin_theta * (v_left + v_right) * dt
        A[1, 2] = R_div_2 * cos_theta * (v_left + v_right) * dt
        A[2, 2] = 1

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

    x_dist = np.subtract(x, xf)
    N = 5 # number of steps
    P = [None] * (N + 1)
    P[N] = Q

    A, B = F.linearize(x, u0, dt)
    AT = np.transpose(A)
    BT = np.transpose(B)

    for t in range(N, 0, -1):
        AT_Pt_A = np.matmul(np.matmul(AT, P[t]), A) # A^T * P[t] * A
        AT_Pt_B = np.matmul(np.matmul(AT, P[t]), B) # A^T * P[t] * B
        BT_Pt_B = np.matmul(np.matmul(BT, P[t]), B) # B^T * P[t] * B
        BT_Pt_A = np.matmul(np.matmul(BT, P[t]), A) # B^T * P[t] * A

        R_BT_Pt_B = np.add(R, BT_Pt_B) # R + (B^T * P[t] * B)
        R_BT_Pt_B_inv = la.pinv(R_BT_Pt_B) # (R + (B^T * P[t] * B))^-1
        Q_AT_Pt_A = np.add(Q, AT_Pt_A) # Q + (A^T * P[t] * A)

        P[t - 1] = np.subtract(Q_AT_Pt_A, np.matmul(np.matmul(AT_Pt_B, R_BT_Pt_B_inv), BT_Pt_A))

    K = [None] * N
    u = [None] * N

    for t in range(0, N):
        BT_Ptplus1_B = np.matmul(np.matmul(BT, P[t + 1]), B) # B^T * P[t + 1] * B
        BT_Ptplus1_A = np.matmul(np.matmul(BT, P[t + 1]), A) # B^T * P[t + 1] * A
        R_BT_Ptplus1_B = np.add(R, BT_Ptplus1_B) # R + (B^T * P[t + 1] * B)
        R_BT_Ptplus1_B_inv = la.pinv(R_BT_Ptplus1_B) # (R + (B^T * P[t + 1] * B))^-1
        neg_R_BT_Ptplus1_B_inv = np.multiply(-1, R_BT_Ptplus1_B_inv) # -(R + (B^T * P[t + 1] * B))^-1

        K[t] = np.matmul(neg_R_BT_Ptplus1_B_inv, BT_Ptplus1_A)
        u[t] = np.matmul(K[t], x_dist)

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
         [0.01, 0],
         [0, 0.01]
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
