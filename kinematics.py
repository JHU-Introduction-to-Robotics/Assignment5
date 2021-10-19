''' ####################
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

#################### '''

import sys
import numpy as np
import scipy.linalg as la
import control

class DifferentialDrive(object):
  """
  Implementation of Differential Drive kinematics.
  This represents a two-wheeled vehicle defined by the following states
  state = [x,y,theta]
  and accepts the following control inputs
  input = [left_wheel_rotation_rate,right_wheel_rotation_rate]

  """
  def __init__(self,L=1,R=1):
    """
    Initializes the class
    Input
      :param L: The distance between the wheels
      :param R: The radius of the wheels
    """

    self.L = L
    self.R = R
    self.u_limit = 1 / R
    self.V = None

  def get_state_size(self):
    return 3

  def get_input_size(self):
    return 2

  def get_V(self):
    '''
    This function provides the covariance matrix V which describes the noise that can be applied to the forward kinematics.
    
    Feel free to experiment with different levels of noise.

      Output
        :return: V: input cost matrix
    '''
    if self.V is None:
        self.V = np.eye(3)
        self.V[0,0] = 0
        self.V[1,1] = 0
        self.V[2,2] = 0.075
    return self.V

  def forward(self,x,u,v,dt):
    """
    Computes the forward kinematics for the system.

    Input
      :param x0: The starting state (position) of the system (units:[m,m,rad]) -> np.array with shape (3,)
      :param u: The control input to the system (e.g. wheel rotation rates) (units: [rad/s,rad/s]) -> np.array with shape (2,)
      :param v: The noise applied to the system (units:[m/s, m/s, rad/s^2]) -> np.array with shape (3,)

    Output
      :return: x1: The new state of the system

    """
    
    #20 points: Implement this funciton

    raise NotImplementedError


  def linearize(self,x,u):
    """
    Computes the first order jacobian for A and B around the state x and input u

    Input
      :param x: The state of the system  -> np.array with shape (3,)
      :param u: The control input to the system (e.g. wheel rotation rates) -> np.array with shape (2,)

    Output
      :return: A: The Jacobian of the kinematics with respect to x 
      :return: B: The Jacobian of the kinematics with respect to u

    """

    #20 points: Implement this funciton

    raise NotImplementedError

def dLQR(F,Q,R,x,xf,dt):
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
    #50 points: Implement this funciton

    raise NotImplementedError


def get_R():
    '''
    This function provides the R matrix to the lqr_steer_control and lqr_ekf_control simulators.
    
    Returnss the input cost matrix R.

    Experiment with different gains to see their effect on the vehicle's behavior.

      Output
        :return: R: input cost matrix
    '''
    #5 points: Implement this funciton
    raise NotImplementedError

def get_Q():
    '''
    This function provides the Q matrix to the lqr_steer_control and lqr_ekf_control simulators.
    
    Returns the input cost matrix R.

    Experiment with different gains to see their effect on the vehicle's behavior.

      Output
        :return: Q: State cost matrix
    '''
    #5 points: Implement this funciton
    raise NotImplementedError


