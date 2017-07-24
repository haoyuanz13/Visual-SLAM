import numpy as np
import quaternions as quat
import util
from point_cloud import  PointCloud
import time
import cv2

# import urllib
import bz2
import os
import numpy as np
from six.moves import urllib
from scipy.sparse import lil_matrix
import matplotlib.pyplot as plt
import time
import cv2
from scipy.optimize import least_squares
import cPickle as pkl
from copy import deepcopy

def compare_with_groundtruth(pred,gt3d):
    error = np.sqrt(np.square(pred[0] - gt3d[0]) + np.square(pred[1] - gt3d[1]) + np.square(pred[2] - gt3d[2]))
    return error

def linearT_errEst(cur_PC, pu=0, pv=0,p3d=None, inliners=None):
    '''Get error from linear triangulation step of a physical point'''
    if inliners == None:
        inliners = range(len(cur_PC.Pos2D))
    error = 0
    if p3d == None:
        homo_X = np.hstack((cur_PC.Pos3D, np.ones(1)))
    else:
        homo_X = np.hstack((p3d, np.ones(1)))
    for i in inliners:
        Hw2c = cur_PC.Hw2c[i]
        cprams = cur_PC.cam_params[i]
        K = np.array([[cprams[6], 0, pu], \
                      [0, cprams[6], pv], \
                      [0, 0, 1]])

        pred = np.dot(np.dot(K, Hw2c[0:3, :]), homo_X)
        pred = pred/pred[2]
        error += np.sqrt(np.square(pred[0] - cur_PC.Pos2D[i][0]) + np.square(pred[1] - cur_PC.Pos2D[i][1]))
    # error = sum(error)
    # time.sleep(1)
    return error


def RansacLinearT(cur_PC,window_frame=10,pu=0,pv=0):
    num_2d = len(cur_PC.Pos2D)
    if num_2d <= 2:
        print('Switch to linearT because there are only two 2d points!')
        opt_P3d =  linearT(cur_PC,range(len(cur_PC.Pos2D)),pu=pu,pv=pv)
        cur_PC.set_3d(opt_P3d)
        cur_PC.set_inliners(range(len(cur_PC.Pos2D)))
        return cur_PC, range(len(cur_PC.Pos2D))

    max_vote, opt_inlier, threshold = 0, [], 30


    # Iterate 1000 times, choose the result with minimum reprojection error (Skew(x) * P * X = 0
    iteration = 200
    # Obtain the last frames (= window_frame)
    if window_frame is not None and num_2d > window_frame:
        indices_in_window = range(num_2d - window_frame, num_2d)
    else:
        indices_in_window = range(num_2d)
    print('*** num_2d: {0}, truncated to {1}'.format(num_2d,len(indices_in_window)))
    for iter in range(iteration):
        # randomly pick two 2d points
        # inds = np.random.randint(indices_in_window[0],num_2d,(2,))
        inds = np.random.choice(indices_in_window, 2, replace=False)
        if inds[0] == inds[1]:
            print('ERROR! in sampling in ransac!!')
            exit(1)
        # print'--Chosing two points:',inds
        # use linear T to obtain physical position (3,)
        p3d = linearT(cur_PC, inds, pu, pv)

        homo_p3d = np.hstack((p3d,np.ones(1)))
        # compute reprojection error
        inliers, cur_vote = [], 0
        for k in indices_in_window:
            Hw2c = cur_PC.Hw2c[k]
            # K, k1, k2
            cprams = cur_PC.cam_params[k]
            K = np.array([[cprams[6], 0, pu], \
                          [0, cprams[6], pv], \
                          [0, 0, 1]])

            P = np.dot(K, Hw2c[0:3, :]) # 3 x 4
            p2d = cur_PC.Pos2D[k]
            skew_p2d = cur_PC.skewM(p2d)

            # First method to compute cost function
            A = np.dot(skew_p2d,P)
            # u, d, v = np.linalg.svd(A)
            # D = np.zeros((3, 4))
            # D[0, 0] = d[0]
            # D[1, 1] = d[1]
            # D[2, 2] = 0
            # A = np.dot(np.dot(u, D), v)


            error = np.dot(A, homo_p3d)
            rmse = np.sqrt(np.dot(error,error))

            # print('k =', 'error:', rmse)

            # pred = np.dot(np.dot(K, Hw2c[0:3, :]), homo_p3d)
            # pred = pred / pred[2]
            # rmse = np.sqrt(np.square(pred[0] - p2d[0]) + np.square(pred[1] - p2d[1]))

            # # check the heck why error large
            # if k in inds and rmse >= threshold:
            #     print('wtf:...... k =',k, 'error:', rmse, 'why inds =',inds)
            #     print('Error:',error)

            if rmse < threshold or k in inds:
                cur_vote += 1
                inliers.append(k)
            #     print('-----------------------k =',k, 'error:', rmse, '----------------------Inlier')
            # print('-----------------------k =', k, 'error:', rmse, '----------------------OOOOOOOOOOOOOOOOOOOUTLIER')
            # print('TOTAL inliers:', cur_vote)


        if cur_vote > max_vote:
            max_vote = cur_vote
            opt_inlier = inliers
            rs_opt_p3d = p3d

    if len(opt_inlier) <= 1:
        print('Why opt_inlier has < 2 item? ')
        print('opt_inlier',opt_inlier)
        print()

        return  None, None
    print('Len optimal inliners:',len(opt_inlier))
    #
    # # remove outliers
    # PC_2ds = [cur_PC.Pos2D[i] for i in opt_inlier]
    # PC_cam_indices = [cur_PC.frame[i] for i in opt_inlier]
    # PC_cam_params = [cur_PC.cam_params[i] for i in opt_inlier]
    # PC_Hw2cs = [cur_PC.Hw2c[i] for i in opt_inlier]
    # cur_PC.Pos2D = deepcopy(PC_2ds)
    # cur_PC.frame = deepcopy(PC_cam_indices)
    # cur_PC.cam_params = deepcopy(PC_cam_params)
    # cur_PC.Hw2c = deepcopy(PC_Hw2cs)
    # # time.sleep(0.4)
    # # apply all inliers to compute optimal 3D position
    # opt_P3d = linearT(cur_PC, range(len(cur_PC.Pos2D)), pu, pv)
    # cur_PC.set_3d(opt_P3d)
    # print('Optimal compare:', opt_P3d, 'vs', rs_opt_p3d)

    # Don't remove outliers
    # apply all inliers to compute optimal 3D position
    opt_P3d = linearT(cur_PC, opt_inlier, pu, pv)
    # update the point cloud
    cur_PC.set_3d(opt_P3d)
    cur_PC.set_inliners(opt_inlier)
    print('Optimal compare:', opt_P3d, 'vs', rs_opt_p3d)
    return cur_PC, opt_inlier




# Implement linear triangulation to update 3D position
def linearT(cur_PC, inliners_ind, pu, pv):
    # number of 2D points
    num_frames = np.asarray(inliners_ind).shape[0]
    # A matrix
    A = np.zeros((3 * num_frames, 4))
    k = 0
    P_set, p2d_set = [], []
    # print('num_frames', num_frames)
    # print cur_PC.Hw2c
    for i in inliners_ind:
        Hw2c = cur_PC.Hw2c[i]
        # K, k1, k2
        cprams = cur_PC.cam_params[i]
        K = np.array([[cprams[6], 0, pu],\
                      [0,  cprams[6], pv],\
                      [0,   0,       1]])

        P = np.dot(K,Hw2c[0:3,:])

        # undis_2D = util.undistort2D(k1, k2, pu, pv, cur_PC.Pos2D[i])
        # p2d_set.append(cur_PC.Pos2D[i])
        skew = cur_PC.skewM(cur_PC.Pos2D[i])
        # skew = cur_PC.skewM(undis_2D)

        A[k:k+3,:] = np.dot(skew, P)[0:3]
        k += 3

    u, d, v = np.linalg.svd(A)
    # print('A:',A)
    # clean up A to enforce its rank to be 3
    # print('d:',d)
    # reconstruct d as a 4x4

    # TODO: not do clean
    # if A.shape[0] == 6:
    #     D = np.zeros((6,4))
    #     D[0,0] = d[0]
    #     D[1,1] = d[1]
    #     D[2,2] = d[2]
    #     D[3,3] = 0
    #
    #     A = np.dot(np.dot(u,D),v)
    #     # print('A:',A)
    #     u, d, v = np.linalg.svd(A)
    Pos3D = v[3, :].T
    # print('Error in SVD:', np.dot(A, Pos3D))
    Pos3D = v[3, :]/v[3, 3]
    p3d = Pos3D[:3] / Pos3D[3]
    # print('Pos3D:',Pos3D)
    # print('Error in SVD:', np.dot(A,Pos3D))
    return p3d

########################################################################################################################
# Testing the linearT() with bundle dataset
def test_linearT(Hw2c_set, points_2d, points_3d,points_indices, camera_indices, camera_params,gt_points_3d):
    n_points = points_3d.shape[0]
    n_cameras = camera_params.shape[0]
    # Obtain first frame
    print('\n---------------Testing linearT()------------------\n')

    # collect point clouds
    pkl_folder = 'pkl'
    if not os.path.exists(pkl_folder):
        os.makedirs(pkl_folder)
    if not os.path.exists(os.path.join(pkl_folder,'PC_set.pkl')):
        PC_set = {}
        gt_3D_set = {}
        for p in range(points_3d.shape[0]):
            p3d = points_3d[p,:]
            gt_p3d =gt_points_3d[p]
            gt_3D_set[p] = gt_p3d
            print('- Point {0}/ {1}'.format(p,points_3d.shape[0]))
            label = points_indices[p]
            PC = PointCloud( label=label, ini_3D=p3d)
            # Find 2d (u,v)
            add2 = 0
            for i in range(points_indices.shape[0]):
                if points_indices[i] == label:
                    PC.newCorres(points_2d[i],camera_indices[i],Hw2c_set[i],camera_params[camera_indices[i],:])
                    # add2 += 1
            PC_set[p] = PC

        PC_data = {}
        PC_data['pred'] = PC_set
        PC_data['gt'] = gt_3D_set
        with open(os.path.join(pkl_folder,'PC_set.pkl'),'wb') as f:
            pkl.dump(PC_data,f,pkl.HIGHEST_PROTOCOL)
    else:
        with open(os.path.join(pkl_folder,'PC_set.pkl'),'rb') as f:
            PC_data = pkl.load(f)
        PC_set = PC_data['pred']
        gt_3D_set = PC_data['gt']


    # Test linear
    Errors_orig = []
    Errors_pred = []
    total_ignore = 0
    for i in PC_set.keys():
        cur_PC = PC_set[i]
        gt_p3d = gt_3D_set[i]
        if len(cur_PC.Pos2D) > 1:
            orgerror = compare_with_groundtruth(PC_set[i].Pos3D, gt_p3d)
            # Errors_orig.append(orgerror)
            # print('\n *    Point',cur_PC.Pos3D)
            # print('* Has {0} frames'.format(len(cur_PC.Pos2D)))
            cur_PC, opt_inliers = RansacLinearT(cur_PC,pu=0,pv=0)

            if cur_PC == None:
                print('-------- Ignore! ------------')
                total_ignore += 1
                PC_set.pop(i)
                continue
            PC_set[i] = cur_PC
            # perror = compare_with_groundtruth(cur_PC.Pos3D, gt_p3d)
            perror = linearT_errEst(cur_PC=cur_PC,pu=0,pv=0,inliners=opt_inliers)
            print('>> ', i, '/', len(PC_set.keys()), '                                                Error:', orgerror,' vs ', perror)
            Errors_pred.append(perror)

            # time.sleep(0.5)

    print('Total errors:')
    print('Original:',np.sum(np.array(Errors_orig))/len(Errors_pred))
    print('Pred:',np.sum(np.array(Errors_pred))/len(Errors_pred))
    print('Total ignore PC:',total_ignore)
    # save the obtained PC_set to the file
    points_3d, points_2d, points_indices, camera_indices, camera_params = util.convertPC(PC_set,n_cameras=n_cameras,n_3dpoints=n_points)
    DATA = {}
    DATA['params'] = camera_params
    DATA['cam_indices'] = camera_indices
    DATA['points_indices'] = points_indices
    DATA['2d'] = points_2d
    DATA['3d'] = points_3d
    with open('linearTed_online_data.pkl', 'wb') as f:
        pkl.dump(DATA, f, pkl.HIGHEST_PROTOCOL)

    plt.figure()
    plt.scatter(range(len(Errors_pred)), Errors_pred)
    plt.show()
    return Errors_orig, Errors_pred

def main():
    '''Test linearT() function with the original data from Bundle Adjustment website'''
    # We will read the given dataset from the bundle sample code and mimic the transformation
    with open('gt_orig_data.pkl', 'rb') as f:
        data = pkl.load(f)

    with open('gt_data.pkl', 'rb') as f:
        gt_data = pkl.load(f)
    gt_points_3d = gt_data['3d']
    # camera_params with shape (n_cameras, 9) contains initial estimates of parameters for all cameras. First 3 components in each row form a rotation vector (https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula), next 3 components form a translation vector, then a focal distance and two distortion parameters.
    # points_3d with shape (n_points, 3) contains initial estimates of point coordinates in the world frame.
    # camera_ind with shape (n_observations,) contains indices of cameras (from 0 to n_cameras - 1) involved in each observation.
    # point_ind with shape (n_observations,) contatins indices of points (from 0 to n_points - 1) involved in each observation.
    # points_2d with shape (n_observations, 2) contains measured 2-D coordinates of points projected on images in each observations.
    camera_params =  data['params']
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

    # plt.subplot(1,2,1)
    # plt.plot(f0)
    # plt.show()
    print('done poltting')

    Hw2c_set = []
    for i in range(points_indices.shape[0]):
        cam_ind = camera_indices[i]
        vecw2c = camera_params[cam_ind,0:3]
        quatw2c = quat.exp_qua(np.array([0, vecw2c[0], vecw2c[1], vecw2c[2]]).reshape(-1,4))
        Rw2c = quat.qua_to_rot(quatw2c)
        Tw2c = camera_params[cam_ind,3:6]
        # append Hw2c
        Hw2c_set.append(rot_trans_to_homo(Rw2c,Tw2c))

    Errors = test_linearT(Hw2c_set, points_2d, points_3d,points_indices, camera_indices, camera_params,gt_points_3d)

if __name__ == "__main__":
    main()







