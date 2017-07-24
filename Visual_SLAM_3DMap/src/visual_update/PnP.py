import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
import util
import quaternions as quat
import cPickle as pkl

def fun(params, points_3d, n_cameras, n_points,camera_indices, points_indices, points_2d,K=None):
    camera_params = params.reshape((n_cameras, 9))
    points_proj = np.zeros(points_2d.shape)
    for i in range(len(points_indices)):
        cam_ind = camera_indices[i]
        p_ind = points_indices[i]
        p3d = points_3d[p_ind]
        # homo 3D
        homo_X = np.hstack((p3d, np.ones(1)))
        # rotation
        vecw2c = camera_params[cam_ind, 0:3]
        quatw2c = quat.exp_qua(np.array([0, vecw2c[0], vecw2c[1], vecw2c[2]]).reshape(-1, 4))
        Rw2c = quat.qua_to_rot(quatw2c)
        Tw2c = camera_params[cam_ind, 3:6]
        Hw2c = util.rot_trans_to_homo(Rw2c, Tw2c)
        pred = np.dot(np.dot(K, Hw2c[0:3, :]), homo_X)
        pred = pred / pred[2]
        points_proj[i,:] = pred[:2]
    # points_proj = project(points_3d[points_indices], camera_params[camera_indices],K)
    # print 'A'
    return (points_proj - points_2d).ravel()


def pnp_sparsity(n_cameras, n_points, camera_indices, points_indices):
    m = camera_indices.size * 2
    n = n_cameras * 9
    A = lil_matrix((m, n), dtype=int)

    i = np.arange(camera_indices.size)
    for s in range(9):
        A[2 * i, camera_indices * 9 + s] = 1
        A[2 * i + 1, camera_indices * 9 + s] = 1
    return A

def NonPnP_main(data_file='linearTed_to_nonlinearT_to_linearPnP_our_data.pkl',input_type='data',dump_file_name='linearTed_to_nonlinearT_to_linearPnP_to_nonlinearPnP_our_data.pkl', PC_set=None):
        # camera_params with shape (n_cameras, 9) contains initial estimates of parameters for all cameras. First 3 components in each row form a rotation vector (https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula), next 3 components form a translation vector, then a focal distance and two distortion parameters.
        # points_3d with shape (n_points, 3) contains initial estimates of point coordinates in the world frame.
        # camera_ind with shape (n_observations,) contains indices of cameras (from 0 to n_cameras - 1) involved in each observation.
        # point_ind with shape (n_observations,) contatins indices of points (from 0 to n_points - 1) involved in each observation.
        # points_2d with shape (n_observations, 2) contains measured 2-D coordinates of points projected on images in each observations.
        # open file to retrieve data
        if PC_set == None and input_type == 'PC_set':
            with open(data_file, 'rb') as f:
                PC_set = pkl.load(f)
        elif PC_set is not None:
            points_3d, points_2d, points_indices, camera_indices, camera_params = util.convertPC(PC_set)

        else:
            with open(data_file, 'rb') as f:
                data = pkl.load(f)
            camera_params = data['params']
            camera_indices = data['cam_indices']
            points_indices = data['points_indices']
            points_2d = data['2d']  #
            points_3d = data['3d']

        n_cameras = camera_params.shape[0]
        n_points = points_3d.shape[0]

        n = 3 * n_points
        m = 2 * points_2d.shape[0]

        print("n_cameras: {}".format(n_cameras))
        print("n_points: {}".format(n_points))
        print("Total number of parameters: {}".format(n))
        print("Total number of residuals: {}".format(m))

        print("n_cameras: {}".format(n_cameras))
        print("n_points: {}".format(n_points))
        print("Total number of parameters: {}".format(n))
        print("Total number of residuals: {}".format(m))

        cprams = camera_params[0]
        _, pu, pv = util.initial_PC()
        K = np.array([[cprams[6], 0, pu], \
                      [0, cprams[6], pv], \
                      [0, 0, 1]])

        x0 = camera_params.ravel()

        f0 = fun(x0, points_3d, n_cameras, n_points, camera_indices, points_indices, points_2d,K)

        # plt.plot(f0)

        A = pnp_sparsity(n_cameras, n_points, camera_indices, points_indices)

        t0 = time.time()
        res = least_squares(fun, x0, jac_sparsity = A, verbose = 2, x_scale = 'jac',max_nfev=100, ftol = 1e-4, method = 'trf', args = (points_3d, n_cameras, n_points, camera_indices, points_indices, points_2d,K))
        t1 = time.time()

        # Save data
        # optimized data
        DATA = {}
        DATA['params'] = camera_params
        DATA['cam_indices'] = camera_indices
        DATA['points_indices'] = points_indices
        DATA['2d'] = points_2d
        DATA['3d'] = points_3d
        with open(dump_file_name, 'wb') as f:
            pkl.dump(DATA, f, pkl.HIGHEST_PROTOCOL)



        print('---Final params:',res.x[0:10])

        print("Optimization took {0:.0f} seconds".format(t1 - t0))
        plt.plot(res.fun)
        plt.show()



# linear PnP
def LinearPnP(K=None, points_3d=None, camera_indices=None, points_indices=None, points_2d=None, camera_params=None):
    terminal_T = max(camera_indices)

    n_obser = points_2d.shape[0]
    A = np.zeros((n_obser * 2, 13))
    for i in range(0, 2 * n_obser, 2):
        p3d = points_3d[points_indices[int(i / 2)], :]
        p2d = points_2d[int(i / 2), :]
        A[i, 0] = camera_indices[int(i / 2)]
        A[i, 1 : 5] = np.array([p3d[0], p3d[1], p3d[2], 1])
        A[i, 9 : ] = np.array([-p2d[0] * p3d[0], -p2d[0] * p3d[1], -p2d[0] * p3d[2], -p2d[0]])

        A[i + 1, 0] = camera_indices[int(i / 2)]
        A[i + 1, 5 : 9] = np.array([p3d[0], p3d[1], p3d[2], 1])
        A[i + 1, 9:] = np.array([-p2d[1] * p3d[0], -p2d[1] * p3d[1], -p2d[1] * p3d[2], -p2d[1]])

    for t in range(terminal_T + 1):
        current = (A[:, 0] == t)
        cur_A = A[current, 1 : ]
        [u, s, v] = np.linalg.svd(cur_A)

        P = v[11, :]
        P_m = np.array([[P[0], P[1], P[2], P[3]],
                        [P[4], P[5], P[6], P[7]],
                        [P[8], P[9], P[10], P[11]]])

        RT = np.einsum('ij,jk->ik', np.linalg.inv(K), P_m)

        R, T = RT[:, : 3], RT[:, 3]
        quat_cur = quat.rot_to_qua(R)
        rot_vec = quat.log_qua(quat_cur.reshape(-1, 4))

        cam_ind = camera_indices[t]

        camera_params[cam_ind, :6] = np.array([rot_vec[0], rot_vec[1], rot_vec[2], T[0], T[1], T[2]])

    return camera_params


