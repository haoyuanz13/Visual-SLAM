import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
import cPickle as pkl
import util
import quaternions as quat

def fun(params, camera_params, n_cameras, n_points, camera_indices, points_indices, points_2d,K=None):
    points_3d = params.reshape((n_points, 3))
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


def nonlinear_triangulation_sparsity(n_points, camera_indices, points_indices):
    m = camera_indices.size * 2
    n = n_points * 3
    A = lil_matrix((m, n), dtype=int)

    i = np.arange(camera_indices.size)

    for s in range(3):
        A[2 * i, points_indices * 3 + s] = 1
        A[2 * i + 1, points_indices * 3 + s] = 1

    return A


def NonLT_main(data_file='linearTed_our_data_PC_set.pkl', dump_file_name='linearTed_to_nonlinearT_our_data.pkl',PC_set=None):
        # camera_params with shape (n_cameras, 9) contains initial estimates of parameters for all cameras. First 3 components in each row form a rotation vector (https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula), next 3 components form a translation vector, then a focal distance and two distortion parameters.
        # points_3d with shape (n_points, 3) contains initial estimates of point coordinates in the world frame.
        # camera_ind with shape (n_observations,) contains indices of cameras (from 0 to n_cameras - 1) involved in each observation.
        # point_ind with shape (n_observations,) contatins indices of points (from 0 to n_points - 1) involved in each observation.
        # points_2d with shape (n_observations, 2) contains measured 2-D coordinates of points projected on images in each observations.
        # open file to retrieve data
        if PC_set == None:
            with open(data_file, 'rb') as f:
                PC_set = pkl.load(f)
        PC_set = util.clean_PC_set(PC_set)
        print('Doing NonlinearT')
        points_3d, points_2d, points_indices, camera_indices, camera_params = util.convertPC(PC_set)

        n_cameras = camera_params.shape[0]
        n_points = points_3d.shape[0]

        n = 9 * n_cameras + 3 * n_points
        m = 2 * points_2d.shape[0]

        print("n_cameras: {}".format(n_cameras))
        print("n_points: {}".format(n_points))
        print("Total number of parameters: {}".format(n))
        print("Total number of residuals: {}".format(m))

        cprams = camera_params[0]
        _, pu, pv = util.initial_PC()
        K = np.array([[cprams[6], 0, pu], \
                      [0, cprams[6], pv], \
                      [0, 0, 1]])
        x0 = points_3d.ravel()
        f0 = fun(x0, camera_params, n_cameras, n_points, camera_indices, points_indices, points_2d,K)

        # plt.plot(f0)

        A = nonlinear_triangulation_sparsity(n_points, camera_indices, points_indices)

        t0 = time.time()
        res = least_squares(fun, x0, jac_sparsity = A, verbose = 2, x_scale = 'jac', ftol = 1e-4, method = 'trf', \
                            args = (camera_params, n_cameras, n_points, camera_indices, points_indices, points_2d,K))
        t1 = time.time()

        print('---Final params:',res.x[0:10])

        print("Optimization took {0:.0f} seconds".format(t1 - t0))
        plt.scatter(range(len(res.fun)),res.fun)
        plt.show()

        # Extract results from res.x
        points_3d = np.reshape(res.x, (n_points, 3))

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


if __name__ == "__main__":
    NonLT_main()

