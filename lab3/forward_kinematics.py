#!/usr/bin/env python

import numpy as np
import scipy as sp
import kin_func_skeleton as kfs

def baxter_forward_kinematics_from_angles(joint_angles):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    angles in radians.

    Parameters
    ----------
    joint_angles ((7x) np.ndarray): 7 joint angles (s0, s1, e0, e1, w0, w1, w2)

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """

    qs = np.ndarray((3,8)) # points on each joint axis in the zero configuration
    ws = np.ndarray((3,7)) # axis vector of each joint axis
    
    # Assign the q values
    qs[0:3,0] = [0.0635, 0.2598, 0.1188]
    qs[0:3,1] = [0.1106, 0.3116, 0.3885]
    qs[0:3,2] = [0.1827, 0.3838, 0.3881]
    qs[0:3,3] = [0.3682, 0.5684, 0.3181]
    qs[0:3,4] = [0.4417, 0.6420, 0.3177]
    qs[0:3,5] = [0.6332, 0.8337, 0.3067]
    qs[0:3,6] = [0.7152, 0.9158, 0.3063]
    qs[0:3,7] = [0.7957, 0.9965, 0.3058]

    # Assign the w values
    ws[0:3,0] = [-0.0059,  0.0113,  0.9999]
    ws[0:3,1] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,3] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,5] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                    [-0.7040, 0.7102, -0.0053],
                    [0.7102, 0.7040, 0.0055]]).T # rotation matrix of zero config

    # YOUR CODE HERE (Task 1)
    
    v1 = np.cross(-ws[0:3,0], qs[0:3,0])
    #print(v1.shape)
    #print( ws[0:3,0].shape)
    xi_1 = np.concatenate([v1, ws[0:3,0]])
    v2 = np.cross(-ws[0:3,1], qs[0:3,1])
    xi_2 = np.concatenate([v2, ws[0:3,1]])
    v3 = np.cross(-ws[0:3,2], qs[0:3,2])
    xi_3 = np.concatenate([v3, ws[0:3,2]])
    v4 = np.cross(-ws[0:3,3], qs[0:3,3])
    xi_4 = np.concatenate([v4, ws[0:3,3]])
    v5 = np.cross(-ws[0:3,4], qs[0:3,4])
    xi_5 = np.concatenate([v5, ws[0:3,4]])
    v6 = np.cross(-ws[0:3,5], qs[0:3,5])
    xi_6 = np.concatenate([v6, ws[0:3,5]])
    v7 = np.cross(-ws[0:3,6], qs[0:3,6])
    xi_7 = np.concatenate([v7, ws[0:3,6]])
    p0 = (np.array(qs[0:3,7].T)).reshape(3, 1)
    #print(p0.shape)
    gst0 = np.concatenate([R, p0], axis = 1)
    gst0 = np.concatenate([gst0, (np.array([0, 0, 0, 1])).reshape(1, 4)], axis = 0)
    gst0 = np.array(gst0, dtype=np.float64)
    xi_array = np.array([xi_1, xi_2, xi_3, xi_4, xi_5, xi_6, xi_7], dtype = np.float64).T
    #print(xi_1)
    #print(gst0)
    g = np.matmul(kfs.prod_exp(xi_array, joint_angles), gst0)
    return g

def baxter_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of Baxter robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    angles = np.zeros(7)

    # YOUR CODE HERE (Task 2)
    #(s0, s1, e0, e1, w0, w1, w2)
    s0 = joint_state.position[4]
    angles[0] = s0
    s1 = joint_state.position[5]
    angles[1] = s1
    e0 = joint_state.position[2]
    angles[2] = e0
    e1 = joint_state.position[3]
    angles[3] = e1
    w0 = joint_state.position[6]
    angles[4] = w0
    w1 = joint_state.position[7]
    angles[5] = w1
    w2 = joint_state.position[8]
    angles[6] = w2

    #print(s0)
    # END YOUR CODE HERE
    print(baxter_forward_kinematics_from_angles(angles))
