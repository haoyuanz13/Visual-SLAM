# import urllib
import bz2
import numpy as np
from scipy.sparse import lil_matrix
import matplotlib.pyplot as plt
import time
import cv2
from scipy.optimize import least_squares
import cPickle as pkl
import util
from point_cloud import PointCloud
import quaternions as quat
def fun(params, n_cameras, n_points, camera_indices, points_indices, points_2d,K=None):
    camera_params = params[:n_cameras * 9].reshape((n_cameras, 9))
    points_3d = params[n_cameras * 9:].reshape((n_points, 3))
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
    # points_proj = project(points_3d[point_indices], camera_params[camera_indices],K)
    print 'A'
    return (points_proj - points_2d).ravel()

def bundle_adjustment_sparsity(n_cameras, n_points, camera_indices, points_indices, PC_set):
    m = camera_indices.size * 2
    n = n_cameras * 9 + n_points * 3
    A = lil_matrix((m, n), dtype=int)

    # to construct the sparse matrix, we will go over every single 3D point.
    # > if its p3d value is None, do nothing.
    # > Else, iterate over all p2d points
    # >>> If it is an outlier, do nothing
    # >>> If it is an inlier, update matrix
    # for PCs in PC_set.values():
    #     if PCs.label == None:
    #         print('--None p3d. Ignore!!')
    #         continue
    #     # iterate over all p2d points
    # cur_row = 0
    # for PCs in PC_set.values():
    #     print('******point3d:', PCs.label,'-----------------------------------------')
    #     p3d_ind = PCs.label
    #     num_p = len(PCs.Pos2D)
    #     if PCs.Pos3D == None:
    #         cur_row += 2*num_p
    #         continue
    #
    #     for i in range(num_p):
    #         # only update the sparse matrix if this point is an inliner
    #         print('--i', i, 'point2d: ', PCs.Pos2D[i],'..................')
    #         if PCs.is_inliners[i]:
    #             # set frame[i] at rows: cur_row + 2*i , cur_row + 2*i + 1
    #             A[cur_row + 2*i, PCs.frame[i]*9: PCs.frame[i]*9 + 6] = 1
    #             A[cur_row + 2*i + 1, PCs.frame[i] * 9: PCs.frame[i] * 9 + 6] = 1
    #             # set point PCs.label at rows: cur_row + 2*i , cur_row + 2*i + 1
    #             A[cur_row + 2*i, n_cameras*9 + PCs.label*3: n_cameras*9 + PCs.label*3 + 3] = 1
    #             A[cur_row + 2*i + 1, n_cameras * 9 + PCs.label * 3: n_cameras * 9 + PCs.label * 3 + 3] = 1
    #     cur_row += num_p*2

    i = np.arange(camera_indices.size)
    for s in range(9):
        A[2 * i, camera_indices * 9 + s] = 1
        A[2 * i + 1, camera_indices * 9 + s] = 1

    for s in range(3):
        A[2 * i, n_cameras * 9 + points_indices * 3 + s] = 1
        A[2 * i + 1, n_cameras * 9 + points_indices * 3 + s] = 1

    A[:,:9] = A[:,:9]*0
    return A


def bundle(data_file='linearTed_our_data_PC_set.pkl',input_type='PC_set',dump_file_name ='linearTed_to_bundle_online_data.pkl',PC_set=None):
    # camera_params with shape (n_cameras, 9) contains initial estimates of parameters for all cameras. First 3 components in each row form a rotation vector (https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula), next 3 components form a translation vector, then a focal distance and two distortion parameters.
    # points_3d with shape (n_points, 3) contains initial estimates of point coordinates in the world frame.
    # camera_ind with shape (n_observations,) contains indices of cameras (from 0 to n_cameras - 1) involved in each observation.
    # point_ind with shape (n_observations,) contatins indices of points (from 0 to n_points - 1) involved in each observation.
    # points_2d with shape (n_observations, 2) contains measured 2-D coordinates of points projected on images in each observations.
    # open file to retrieve data
    if PC_set == None and input_type == 'PC_set':
        with open(data_file, 'rb') as f:
            PC_set = pkl.load(f)
    elif  PC_set is not None:
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

    x0 = np.hstack((camera_params.ravel(), points_3d.ravel()))

    f0 = fun(x0, n_cameras, n_points, camera_indices, points_indices, points_2d,K)

    f0 = np.reshape(f0,(points_indices.shape[0],2))

    plt.subplot(1,2,1)
    plt.plot(f0)
    print('Done ploting')

    A = bundle_adjustment_sparsity(n_cameras, n_points, camera_indices, points_indices,PC_set)

    t0 = time.clock()
    res = least_squares(fun, x0, jac_sparsity = A, verbose = 2, x_scale = 'jac',\
            ftol = 1e-4, method = 'trf', max_nfev=100,\
            args = (n_cameras, n_points, camera_indices, points_indices, points_2d, K))
    t1 = time.clock()

    print('---Final params:',res.x[0:10])
    final_rot,_ = cv2.Rodrigues(res.x[0:3])

    # print('---Init rot:\n',init_rot)
    print('---Final rot:\n', final_rot)

    print("Optimization took {0:.0f} seconds".format(t1 - t0))
    plt.subplot(1, 2, 2)
    plt.plot(res.fun)
    plt.show()

    # Extract results from res.x
    camera_params = np.reshape(res.x[0:n_cameras*9],(n_cameras,9))
    points_3d = np.reshape(res.x[n_cameras*9:],(n_points,3))

    # Save data
    # optimized data
    DATA = {}
    DATA['params'] = camera_params
    DATA['cam_indices'] = camera_indices
    DATA['points_indices'] = points_indices
    DATA['2d'] = points_2d
    DATA['3d'] = points_3d
    with open(dump_file_name,'wb') as f:
        pkl.dump(DATA,f,pkl.HIGHEST_PROTOCOL)


def test_bundle_sparse_matrix():
    PC_set = {}
    PC1 = PointCloud(label=0, ini_3D=None, ini_2D=[0,0], ini_frame=1, ini_Hw2c=None,cprams=None,inliners=None)

    PC2 = PointCloud(label=1, ini_3D=[0,0,0], ini_2D=[10,10], ini_frame=0, ini_Hw2c=None, cprams=None, inliners=1)
    PC3 = PointCloud(label=2, ini_3D=[10, 10, 10], ini_2D=[20, 20], ini_frame=0, ini_Hw2c=None, cprams=None, inliners=1)
    PC4 = PointCloud(label=3, ini_3D=[20, 20, 20], ini_2D=[30, 30], ini_frame=1, ini_Hw2c=None, cprams=None, inliners=1)
    PC5 = PointCloud(label=4, ini_3D=[30, 30, 30], ini_2D=[40, 40], ini_frame=1, ini_Hw2c=None, cprams=None, inliners=0)

    PC6 = PointCloud(label=5, ini_3D=None, ini_2D=None, ini_frame=None, ini_Hw2c=None, cprams=None, inliners=None)

    PC7 = PointCloud(label=6, ini_3D=None, ini_2D=None, ini_frame=None, ini_Hw2c=None, cprams=None, inliners=None)

    # add new correspondences
    PC2.newCorres(p2d=[100,100], cam_ind=1, Hwc2=None, cprams=None, inliners=0)
    # add new correspondences
    PC2.newCorres(p2d=[1000, 1000], cam_ind=2, Hwc2=None, cprams=None, inliners=1)

    PC_set[1] = PC1
    PC_set[2] = PC2
    PC_set[3] = PC3
    PC_set[4] = PC4
    PC_set[5] = PC5
    PC_set[6] = PC6
    PC_set[7] = PC7
    points_3d, points_2d, points_indices, camera_indices, camera_params = util.convertPC(PC_set)
    print('Set of points_2d:')
    print(points_2d)
    n_cameras = camera_params.shape[0]
    n_points = points_3d.shape[0]
    A = bundle_adjustment_sparsity(n_cameras, n_points, camera_indices, points_indices, PC_set)
    print A



if __name__ == "__main__":
    # bundle()
    test_bundle_sparse_matrix()
