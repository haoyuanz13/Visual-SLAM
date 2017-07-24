import numpy as np
import os, sys
from point_cloud import PointCloud
import quaternions as quat
import time
import cPickle as pkl

def initial_PC():
    # import calibration data for rgb camera
    rgb_Calib = pkl.load(open("data/RGBcamera_Calib_result.pkl","rb"))
    focal_length = rgb_Calib['fc']  # focal length
    dist_k = rgb_Calib['kc'] # distortion factor
    princip_center = rgb_Calib['cc'] # principle points

    # import feature result
    feature_pos = np.load('data/feature_pos.npy')
    feature_frame = np.load('data/feature_frame.npy')

    # import prediction result
    pred_pos_rob = np.load('data/pos_pred.npy')
    pred_q_rob = np.load('data/q_pred.npy')
    pred_T = np.load('data/T_pred.npy')

    # import time line of lidar and camera
    time_camera = np.load('data/time_camera.npy')
    time_lidar = np.load('data/time_lidar.npy')

    # number of detected physical points
    n_points_3d = len(feature_pos)
    # number of camera
    n_camera = time_camera.shape[0]

    # focal length and distortion factor
    f, k1, k2 = np.sum(focal_length) / 2, dist_k[0], dist_k[1]
    pu, pv = princip_center[0], princip_center[1]

    # match time step between camera and lidar
    lidar_ind = []
    for i in range(time_camera.shape[0]):
        lidar_ind.append(np.argmin(np.abs(time_camera[i] - time_lidar)))

    # initialize Point cloud
    PC_set = {}
    # ind_frame is the current camera frame
    for ind_frame in range(time_camera.shape[0]):
        # extract corresponding lidar frame based on camera frame
        ind_lidar = lidar_ind[ind_frame]
        # current transformation of camera
        cur_T = pred_T[4 * ind_lidar: 4 * ind_lidar + 4, :]
        # construct camera parameters given by transformation
        cur_camp = get_cparams(cur_T, f, k1, k2)

        # i is the label of physical 3D point
        for i in range(len(feature_frame)):
            # if current physical point has less than 2 correspondences, filter it out.
            if len(feature_frame[i]) < 2:
                continue
            # check whether this physical point is detected in current camera frame
            if ind_frame in feature_frame[i]:
                # if so, extract its 2d information as well as store camera frame
                ind = feature_frame[i].index(ind_frame)
                cur_2d = feature_pos[i][ind]

                # if this physical point has been already added into PC set
                # update exist object information
                if i in PC_set:
                    PC_set[i].newCorres(p2d=cur_2d, cam_ind=feature_frame[i][ind], Hwc2=cur_T, cprams=cur_camp)
                # if meet new physical point, create a new Point cloud structure and add into set
                else:
                    PC_set[i] = PointCloud(label=i, ini_3D=None, ini_2D=cur_2d, ini_frame=feature_frame[i][ind], ini_Hw2c=cur_T, cprams=cur_camp)
    return PC_set, pu, pv

def query_groundtruth():
    '''Obtain the ground truth from online dataset'''
    folder = 'data/ground_truth'
    pose = np.loadtxt(os.path.join(folder, 'pose.txt'))
    # todo: change datasize
    times = pose[:, 0]

    data_size = len(times)
    # todo: change datasize
    # data_size = 2000
    rotations = np.empty((data_size, 3, 3))
    translations = np.empty((data_size, 3))
    h_translations = np.empty((data_size,4,4))
    for i in range(data_size):
        p0, p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34 = pose[i, :]
        rot = np.array([[p11, p12, p13], \
                        [p21, p22, p23], \
                        [p31, p32, p33]])
        trans = np.array([p14, p24, p34])
        rotations[i, :, :] = rot
        translations[i, :] = trans
        h_translations[i,:,:] = np.vstack((np.hstack((rot, np.reshape(trans,(3,1)))),np.array([0,0,0,1])))
    return times, rotations, translations,h_translations


def query_cam_times(file_name = 'data/timestamps_cameras.txt'):
    return np.loadtxt(file_name)

def match_cam_time_to_ground(t_cam, gt_times):
    '''Find an index of groundtruth pose that have the same time as camera frame'''
    t_temp = t_cam - gt_times
    error = np.absolute(t_temp)
    index = np.argmin(error)
    return index


# TODO: check this function needed or not
def collectFeats(PC_set, n_3dpoints, ind_frame, fea_pos, fea_frame, Hw2c,cprams):
    '''collect features in one frame'''
    for label in range(n_3dpoints):
        if ind_frame in fea_frame[label]:
            ind = fea_frame[label].index(ind_frame)
            if label in PC_set:
                PC_cur = PC_set[label]
                PC_cur.newCorres(fea_pos[label][ind], ind_frame, Hw2c,cprams)
            else:
                PC_set[label] = PointCloud(label, np.zeros(3), fea_pos[label][ind], ind_frame, Hw2c,cprams)
    return PC_set


# convert homography into rotation and translation vector
def getRTfromH(H):
    rot = H[0:3,0:3]
    trans = H[0:3,3]

    q = quat.rot_to_qua(rot)
    rot_vec = quat.log_qua(q)
    return rot_vec, trans

def rot_trans_to_homo(R,T):
    return np.vstack((np.hstack((R, np.reshape(T,(3,1)))),np.array([0,0,0,1])))


def undistort2D(k1, k2, pu, pv, Pos2d_cur):
    pos2d_new = Pos2d_cur - np.array([pu, pv])
    r2 = np.square(pos2d_new[0]) + np.square(pos2d_new[1])
    fac = 1 + k1 * r2 + k2 * np.square(r2)
    pos2d_new_upd = fac * pos2d_new
    undis2D = pos2d_new_upd + np.array([pu, pv])
    return undis2D


def convertPC(PC_set,n_3dpoints=None,n_cameras=None):
    '''Convert data structure from set of point clouds to points_3d, points_2d, point_indices, camera_indices...
        which will be used for triangulations...
    '''
    # if n_3dpoints and n_cameras are not defined, then we can obtain them
    if n_3dpoints == None:
        n_cameras, n_3dpoints, _, _ = get_num_camera_and_3dpoints(PC_set)

    points_3d = np.zeros((n_3dpoints,3))
    camera_params = np.zeros((n_cameras,9))

    points_2d, points_indices, camera_indices = [], [], []

    for PCs in PC_set.values():
        p3d_ind = PCs.label
        points_3d[p3d_ind,:] = PCs.Pos3D

        num_p = len(PCs.Pos2D)
        for i in range(num_p):
            points_2d.append(PCs.Pos2D[i])
            points_indices.append(p3d_ind)
            cam_ind = PCs.frame[i]
            camera_indices.append(cam_ind)
            try:
                camera_params[cam_ind] = PCs.cam_params[i]
            except:
                pass
    points_2d = np.array(points_2d)
    points_indices = np.array(points_indices)
    camera_indices = np.array(camera_indices)
    return points_3d, points_2d, points_indices, camera_indices,camera_params


def get_cparams(cur_T, f, k1, k2):
    # extract rotation and translation information from T matrix
    cur_R, cur_t = cur_T[:3, :3], cur_T[:3, 3]
    # convert rotation matrix into quaternion and then to the rotation vector
    cur_q = quat.rot_to_qua(cur_R)
    cur_rot_vec = quat.log_qua(cur_q)[1:]
    # construct camera parameters which is size 9 vector
    cam_param = np.array([cur_rot_vec[0], cur_rot_vec[1], cur_rot_vec[2], \
                          cur_t[0], cur_t[1], cur_t[2], \
                          f, k1, k2])
    return cam_param

def get_num_camera_and_3dpoints(PC_set):
    '''Obtain the number of cameras and number of 3dpoints in the PC_set as well as all cam_indices, all point indx (in
    list format)'''
    all_cam_ind = []
    all_point_ind = []
    for PCs in PC_set.values():
        try:
            all_point_ind.append(PCs.label)
        except:
            print("ERROR!!!")
            print('PC_set:', PC_set[0])
        all_cam_ind = all_cam_ind + PCs.frame
    n_3dpoints = np.unique(all_point_ind).shape[0]

    print('Max point index:',max(all_point_ind))
    print('Number of n_3dpoints:',n_3dpoints)

    n_cameras = np.unique(all_cam_ind).shape[0]
    return n_cameras, n_3dpoints, all_cam_ind, all_point_ind

def clean_PC_set(PC_set):
    '''Remove None point cloud (with Pos3D = None - meaning that without estimation information),
    and remove camera frames that have no features'''
    n_cameras, n_3dpoints, all_cam_ind, all_point_ind = get_num_camera_and_3dpoints(PC_set)
    # clean up none elements and relabel point cloud elements
    new_PC_set, label_new = {}, 0
    for cur_PC in PC_set.values():
        if cur_PC.Pos3D == None:
            continue
        else:
            cur_PC.label = label_new
            new_PC_set[label_new] = cur_PC
            label_new += 1

    # delete information of outliers
    for cur_PC in new_PC_set.values():
        # the number of total tracked 2d points for current physical 3D point
        total_2d = len(cur_PC.Pos2D)
        for i in range(total_2d - 1, -1, -1):
            if cur_PC.is_inliners[i] == 0:
                del cur_PC.Pos2D[i]
                del cur_PC.frame[i]
                del cur_PC.Hw2c[i]
                del cur_PC.cam_params[i]
    # remove camera frames that didn't detect any inliers
    remove_cam_ind = []
    for cam_frame in range(n_cameras):
        flag = 0
        for cur_PC in new_PC_set.values():
            if cam_frame in cur_PC.frame:
                flag = 1
                break
        if flag == 0:
            remove_cam_ind.append(cam_frame)

    if remove_cam_ind != None:
        # rewrite frame indices
        for cur_PC in new_PC_set.values():
            # traverse all valid frame indices
            for i in range(len(cur_PC.frame)):
                cur_frame = cur_PC.frame[i]
                count = 0  # count the number of remove indices that is smaller than current valid frame index
                for remove_ind in remove_cam_ind:
                    if remove_ind < cur_frame:
                        count += 1
                    else:
                        break
                # re-order index
                cur_PC.frame[i] = cur_frame - count

    return new_PC_set


def test_clean_PC_set():
    PC_set = {}
    Hw2c = np.zeros((4,4))
    cprams = np.zeros((1,9))
    PC1 = PointCloud(label=0, ini_3D=None, ini_2D=[0,0], ini_frame=1, ini_Hw2c=Hw2c,cprams=cprams,inliners=None)

    PC2 = PointCloud(label=1, ini_3D=[0,0,0], ini_2D=[10,10], ini_frame=0, ini_Hw2c=Hw2c, cprams=cprams, inliners=1)
    PC3 = PointCloud(label=2, ini_3D=[10, 10, 10], ini_2D=[20, 20], ini_frame=0, ini_Hw2c=Hw2c, cprams=cprams, inliners=1)
    PC4 = PointCloud(label=3, ini_3D=[20, 20, 20], ini_2D=[30, 30], ini_frame=1, ini_Hw2c=Hw2c + 1, cprams=cprams + 0.1, inliners=1)
    PC5 = PointCloud(label=4, ini_3D=[30, 30, 30], ini_2D=[40, 40], ini_frame=1, ini_Hw2c=Hw2c + 1, cprams=cprams + 0.1, inliners=0)

    PC6 = PointCloud(label=5, ini_3D=None, ini_2D=None, ini_frame=None, ini_Hw2c=Hw2c, cprams=cprams, inliners=None)

    PC7 = PointCloud(label=6, ini_3D=None, ini_2D=None, ini_frame=None, ini_Hw2c=Hw2c, cprams=cprams, inliners=None)

    # add new correspondences
    PC2.newCorres(p2d=[100,100], cam_ind=1, Hwc2=Hw2c + 1, cprams=cprams + 0.1, inliners=0)
    # add new correspondences
    PC2.newCorres(p2d=[1000, 1000], cam_ind=2, Hwc2=Hw2c + 2, cprams=cprams + 0.2, inliners=1)

    PC_set[1] = PC1
    PC_set[2] = PC2
    PC_set[3] = PC3
    PC_set[4] = PC4
    PC_set[5] = PC5
    PC_set[6] = PC6
    PC_set[7] = PC7



    # do cleaning
    PC_set_after_cleaning = clean_PC_set(PC_set)
    points_3d, points_2d, points_indices, camera_indices, camera_params = convertPC(PC_set_after_cleaning)

    print('------- final  result: -------------')
    print(points_2d)
    print('------------------------------------')
    print(camera_indices)
    print('------------------------------------')
    print(points_indices)
    print('------------------------------------')
if __name__ == "__main__":
    test_clean_PC_set()




