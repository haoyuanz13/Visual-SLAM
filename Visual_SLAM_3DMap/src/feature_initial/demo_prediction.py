"""""""""""""""""""""""""""""""""""""""""""""
DESCRIPTION
This file aims to implement the prediction step of visual SLAM. 
After running this code file, it will generate a predicted trajectory to show the whole procedure, 
as well as orientation and homogeneous transformation of robot in each state. 
"""""""""""""""""""""""""""""""""""""""""""""

import numpy as np
import matplotlib.pyplot as plt
import math as m
import cv2
import pickle
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from mpl_toolkits.mplot3d import Axes3D
# own designed functions
import load_data as ld
from robot import robot
import helper as h
import quaternions as quat


# Functions to load and store the trained models
def save_obj(obj, name):
    with open('obj/'+ name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name):
    with open('obj/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)

def roundup(x, t_jump):
    return int(m.ceil(x / t_jump)) * t_jump


def main():
    # Load lidar and joint data
    lidar_dat = ld.get_lidar('data/train_lidar3')
    joint_dat = ld.get_joint('data/train_joint3')

    # Match the joint index to lidar data
    joint_ind = []
    lidar_t = lidar_dat[0]['t']
    for i in range(len(lidar_dat)):
        joint_ind.append(np.argmin(np.abs(lidar_dat[i]['t'] - joint_dat['ts'])))
        lidar_t = np.vstack((lidar_t, lidar_dat[i]['t']))

    np.save('time_lidar.npy', lidar_t) # save time line of lidar

    # Pick neck and head angles at only that timestep
    neck = joint_dat['head_angles'][0, joint_ind]
    head = joint_dat['head_angles'][1, joint_ind]
    t = joint_dat['ts'][:, joint_ind]

    # Initialize
    pos0 = np.array([0, 0, 0])
    q0 = np.array([1, 0, 0, 0])
    rob = robot(pos0, q0)
    t_start = 0

    pos_arr = np.zeros((1, 2))
    q_arr = np.array([1, 0, 0, 0])
    T_arr = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    for i in range(t_start, t.shape[1] - 1):

        # Find the current pose
        T_cur = h.qp2homo(rob.q, rob.pos)

        # Odometry to get postion action
        # Get rot_act from rpy
        pos_next = lidar_dat[i + 1]['pose'][0]
        pos_next = np.array([pos_next[0], pos_next[1], 0])

        rpy_next = lidar_dat[i + 1]['rpy'][0]
        q_next = h.rpy2quat(rpy_next[0], rpy_next[1], rpy_next[2])
        T_next = h.qp2homo(q_next, pos_next)

        # Get the control action
        T_act = np.dot(np.linalg.inv(T_cur), T_next)
        q_act, pos_act = h.homo2qp(T_act)
        # motion model
        rob.predict(q_act, pos_act)
        # update pos and quaternion array
        pos_arr = np.vstack((pos_arr, np.array([rob.pos[0], rob.pos[1]])))
        q_arr = np.vstack((q_arr, np.array(rob.q)))
        # update homogeneous transformation array
        T_cur = np.linalg.inv(h.Tcam2w(rob, head[i], neck[i]))
        T_arr = np.vstack((T_arr, T_cur))

    # store data
    np.save('pos_pred.npy', pos_arr)
    np.save('q_pred.npy', q_arr)
    np.save('T_pred.npy', T_arr)
    # show the predicted trajectory
    fig1 = plt.figure(1)
    plt.plot(pos_arr[:, 0], pos_arr[:, 1])
    plt.show()

if __name__ == '__main__':
    main()