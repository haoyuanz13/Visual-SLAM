"""""""""""""""""""""""""""""""""""""""""""""
DESCRIPTION
This file aims to create a robot class which contains the position and orientation information of robot.
There are two main functions, one is to compute transformation matrix to map coordinate in 3D space into camera frame.
The other is to update current robot's position and orientation in prediction step.
"""""""""""""""""""""""""""""""""""""""""""""

import numpy as np
import math as m
import cv2
import quaternions as quat

class robot(object):
    def __init__(self, pos, q):
        # two variables
        self.pos = pos
        self.q = q


    def wld2cam(self, head, neck, dist, pw):
        # Robot pose
        Rr2w = quat.qua_to_rot(self.q)
        tr2w = self.pos

        # Transformation from robot base to world
        Tr2w = np.vstack((Rr2w,tr2w))
        Tr2w = np.hstack((Tr2w,np.array([0,0,0,1])))

        # Transformation from the robot head to the base
        Th2r = np.array([[1., 0., 0., 0.],
                         [0., 1., 0., 0.],
                         [0., 0., 1., dist['g2com'] + dist['com2h']],
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
                                [0., 0., 1., dist['h2cam']],
                                [0., 0., 0., 1.]]))

        # Transform from  local coordinates to global coordinates
        Tcam2w = np.dot(Tr2w, np.dot(Th2r, Tcam2h))
        pcam = np.dot(np.linalg.inv(Tcam2w), pw)

        return pcam

    def predict(self, q_act, pos_act):
        # print hex(id(self.x))
        # pos act is (3,) and rot_act is 3x3 which represents rotation of next frame in terms of the 1st one
        # Use current orientation
        q_cur = self.q
        # Convert q_act to rot_act
        # rot_act = quat.qua_to_rot(q_act)

        # Find rotation matrix to convert from local to global
        R = quat.qua_to_rot(q_cur)

        # Convert from local to global frame and add the noise
        pos_pred = self.pos + (np.dot(R, pos_act))
        # rot_pred = R*rot_act
        self.pos = pos_pred
        self.q = quat.multi_quas(q_cur, q_act)
