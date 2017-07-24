"""""""""""""""""""""""""""""""""""""""""""""
DESCRIPTION
This file contains several help function that will be used in other code files.
"""""""""""""""""""""""""""""""""""""""""""""
import numpy as np
import matplotlib.pyplot as plt
import math as m
import quaternions as quat

# create a dictionary to store distance information in robot
def setDist_params(dist):
    dist['g2com'] = 0.93
    dist['com2h'] = 0.33
    dist['h2lid'] = 0.15
    dist['h2cam'] = 0.07
    dist['g2lid'] = 0.93 + 0.33 + 0.15
    dist['g2cam'] = 0.93 + 0.33 + 0.07
    return dist

# convert euler angles to quaternions
def rpy2quat(r, p, y):
    rot = np.dot(np.array([[m.cos(y), -m.sin(y), 0.],
                           [m.sin(y), m.cos(y), 0.],
                           [0., 0., 1.]]),
                 np.dot(np.array([[m.cos(p), 0, m.sin(p)],
                                  [0, 1., 0],
                                  [-m.sin(p), 0., m.cos(p)]]),
                        np.array([[1., 0, 0],
                                  [0, m.cos(r), -m.sin(r)],
                                  [0., m.sin(r), m.cos(r)],
                        ])))
    q = quat.rot_to_qua(rot)
    return q

# generate homogeneous transformation given by rotation and translation
def qp2homo(q,pos):
    rot = quat.qua_to_rot(q)
    t = pos.reshape(-1, 3)
    T = np.hstack((rot, t.T))
    T = np.vstack((T, np.array([0, 0, 0, 1]).reshape(-1, 4)))
    return T

# obtain quaternion and translation from homogeneous transformation
def homo2qp(T):
    q = quat.rot_to_qua(T[:3, :3])
    pos = T[:3, 3]
    return q, pos

# generate homogeneous transformation from camera frame to world frame
def Tcam2w(rob, head, neck):
    # Robot pose
    Rr2w = quat.qua_to_rot(rob.q)
    tr2w = (rob.pos).reshape(-1, 3)

    # Transformation from robot base to world
    Tr2w = np.hstack((Rr2w, tr2w.T))
    Tr2w = np.vstack((Tr2w, np.array([0, 0, 0, 1]).reshape(-1, 4)))

    # Transformation from the robot head to the base
    Th2r = np.array([[1., 0., 0., 0.],
                     [0., 1., 0., 0.],
                     [0., 0., 1., 0.93 + 0.33],
                     [0., 0., 0., 1.]])

    # Transformation from camera to head = R_yaw * R_pitch * R_trans
    Tcam2h = np.dot(np.dot(np.array([[m.cos(neck), -m.sin(neck), 0., 0.],
                                   [m.sin(neck), m.cos(neck), 0., 0.],
                                   [0., 0., 1., 0.],
                                   [0., 0., 0., 1.]]),
                         np.array([[m.cos(head), 0., m.sin(head), 0.],
                                   [0., 1., 0., 0.],
                                   [-m.sin(head), 0., m.cos(head), 0.],
                                   [0., 0., 0., 1.]])),
                  np.array([[1., 0., 0., 0.],
                            [0., 1., 0., 0.],
                            [0., 0., 1., 0.07],
                            [0., 0., 0., 1.]]))

    # Transform from  local coordinates to global coordinates
    T_cam2w = np.dot(Tr2w, np.dot(Th2r, Tcam2h))
    T_cam2w = np.dot(np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]).T, T_cam2w)
    # T_cam2w = np.dot(np.array([[0, -1, 0, 0],[0, 0, -1, 0],[1, 0, 0, 0], [0, 0, 0, 1]]), T_cam2w)
    return T_cam2w