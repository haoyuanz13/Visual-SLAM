import numpy as np
from point_cloud import PointCloud
import util
import cPickle as pkl
from linearT import RansacLinearT, linearT_errEst
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import bundle
import PnP
import util
import NonLinearT
import PnP

def apply_linearT(etol=100000):
    '''etol: error tolaration for each point cloud. If the reprojection error is higher than this value, just don't
    do triangulation.'''

    # Intialize PC set
    PC_set, pu, pv = util.initial_PC()
    print (pu, pv)
    # Apply linearT
    Errors_pred = []
    total_ignore = 0
    for i in PC_set.keys():
        cur_PC = PC_set[i]
        if len(cur_PC.Pos2D) <= 1 :
            print('-------- Ignore because there is only 1 correspondence! ------------')
            total_ignore += 1
            continue
        if len(cur_PC.Pos2D) >= 2:
            # Use ransacLinearT
            # TODO: using window truncatation of 10
            cur_PC, opt_inliers = RansacLinearT(cur_PC=cur_PC,window_frame=10,pu=pu,pv=pv)
            # If return None, ignore the point
            if cur_PC == None:
                print('-------- Ignore due to return None! ------------')
                total_ignore += 1
                continue

            # perror = compare_with_groundtruth(cur_PC.Pos3D, gt_p3d)
            perror = linearT_errEst(cur_PC=cur_PC,pu=pu,pv=pv,inliners=opt_inliers)
            print('>> ', i, '/', len(PC_set.keys()), '                                        Error:', perror)
            PC_set[i] = cur_PC

            if perror > etol:
                cur_PC.Pos3D = None
                print('-------- Ignore! ------------')
                total_ignore += 1
                continue

            # Check why error is large
            # if perror > 100:
            #     print('Why it is larger than 100?')
            #     time.sleep(2)
            Errors_pred.append(perror)

    print('Total errors Pred:', np.sum(np.array(Errors_pred)) / len(Errors_pred))
    print('Total ignore PC:', total_ignore)

    plt.figure()
    plt.scatter(range(len(Errors_pred)), Errors_pred)
    plt.show()


    # save the obtained PC_set to the file
    points_3d, points_2d, points_indices, camera_indices, camera_params = util.convertPC(PC_set)

    DATA = {}
    DATA['params'] = camera_params
    DATA['cam_indices'] = camera_indices
    DATA['points_indices'] = points_indices
    DATA['2d'] = points_2d
    DATA['3d'] = points_3d
    with open('pkl/linearTed.pkl', 'wb') as f:
        pkl.dump(DATA, f, pkl.HIGHEST_PROTOCOL)

    # save PC_set:
    with open('pkl/linearTed_PC_set.pkl', 'wb') as f:
        pkl.dump(PC_set, f, pkl.HIGHEST_PROTOCOL)

    return  Errors_pred

def apply_nonlinearT():
    NonLinearT.NonLT_main(data_file='pkl/linearTed_PC_set.pkl', dump_file_name='pkl/linearTed_to_nonlinearT.pkl',PC_set=None)


def apply_linearPnP(data_file='pkl/linearTed_to_nonlinearT.pkl',dump_file_name='pkl/linearTed_to_nonlinearT_to_linearPnP.pkl'):
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
    cprams = camera_params[0]
    _, pu, pv = util.initial_PC()
    K = np.array([[cprams[6], 0, pu], \
                  [0, cprams[6], pv], \
                  [0, 0, 1]])


    # # Clean up the data to remove None p3d as well as outliers
    # # TODO: there is abug in clean_PC_set which is: not relabel 3D points -> max label > number of 3D points.
    # _, pu, pv = initial_PC()
    # cprams = PC_set[0].cam_params[0]
    # K = np.array([[cprams[6], 0, pu], \
    #               [0, cprams[6], pv], \
    #               [0, 0, 1]])
    #
    # points_3d, points_2d, points_indices, camera_indices, camera_params = util.convertPC(PC_set)

    camera_params = PnP.LinearPnP(K=K, points_3d=points_3d, camera_indices=camera_indices, points_indices=points_indices, points_2d=points_2d, camera_params=camera_params)

    # Save data
    DATA = {}
    DATA['params'] = camera_params
    DATA['cam_indices'] = camera_indices
    DATA['points_indices'] = points_indices
    DATA['2d'] = points_2d
    DATA['3d'] = points_3d
    with open(dump_file_name, 'wb') as f:
        pkl.dump(DATA, f, pkl.HIGHEST_PROTOCOL)

def apply_nonlinearPnP():
    PnP.NonPnP_main(data_file='pkl/linearTed_to_nonlinearT_to_linearPnP.pkl',\
                    dump_file_name='pkl/linearTed_to_nonlinearT_to_linearPnP_to_nonlinearPnP.pkl', PC_set=None)


def apply_bundle(data_file='pkl.linearTed_to_nonlinearT_to_linearPnP_to_nonlinearPnP.pkl', input_type='data'):

    if input_type == 'PC_set':
        # load data after doing linearT
        with open(data_file, 'rb') as f:
            PC_set = pkl.load(f)


        # Clean up the data to remove None p3d as well as outliers
        # TODO: there is abug in clean_PC_set which is: not relabel 3D points -> max label > number of 3D points.
        PC_set = util.clean_PC_set(PC_set)


        bundle.bundle(PC_set=PC_set)
    else:
        bundle.bundle(data_file=data_file,input_type=input_type,dump_file_name ='pkl/linearTed_to_nonlinearT_to_linearPnP_to_nonlinearPnP_to_bundle.pkl',PC_set=None)


def display_point_cloud(data_file='pkl/linearTed_to_nonlinearT_to_linearPnP_to_nonlinearPnP_to_bundle.pkl', display_traj=True, display_pc=True, visual='3D'):
    # Original data
    # data_file = 'gt_data.pkl'
    # Data after do linearT, Bundle
    # data_file = 'linearTed_to_bundle_online_data.pkl'
    # Data after do linearT
    # data_file = 'linearTed_online_data.pkl'

    # We will read the given dataset from the bundle sample code and mimic the transformation
    with open(data_file, 'rb') as f:
        data = pkl.load(f)

    # camera_params with shape (n_cameras, 9) contains initial estimates of parameters for all cameras. First 3 components in each row form a rotation vector (https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula), next 3 components form a translation vector, then a focal distance and two distortion parameters.
    # points_3d with shape (n_points, 3) contains initial estimates of point coordinates in the world frame.
    # camera_ind with shape (n_observations,) contains indices of cameras (from 0 to n_cameras - 1) involved in each observation.
    # point_ind with shape (n_observations,) contatins indices of points (from 0 to n_points - 1) involved in each observation.
    # points_2d with shape (n_observations, 2) contains measured 2-D coordinates of points projected on images in each observations.
    camera_params = data['params']
    camera_indices = data['cam_indices']
    point_indices = data['points_indices']
    points_2d = data['2d']  #
    points_3d = data['3d']
    n_cameras = camera_params.shape[0]
    n_points = points_3d.shape[0]
    print("n_cameras: {}".format(n_cameras))
    print("n_points: {}".format(n_points))

    # obtain the trajectories
    Hw2c_set = np.empty((n_points, 4, 4))
    trajectory = np.empty((n_cameras, 3))
    for i in range(camera_params.shape[0]):
        # TODO: transform from camera-to-world to body-to-world and display body-to-world
        Tw2c = camera_params[i, 3:6]
        # append trajectory
        trajectory[i, :] = -Tw2c

    points_3d = points_3d[np.where(np.max(points_3d, axis=1) < 1000)]
    # The first pose is [0, 0, 0]
    trajectory[0, :] = np.zeros(3)

    # Display in 2D
    fig = plt.figure()
    if visual == '2D':
        plt.scatter(trajectory[:, 0], trajectory[:, 1],linewidth='1',label='Trajectory', color='b')
        plt.scatter(points_3d[:, 0], points_3d[:, 1], linewidth='0.1', label='Point Cloud', color='r')
    else: # or in 3D
        ax = fig.add_subplot(111, projection='3d')
        if display_traj:
            ax.scatter(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], linewidth='3', label='Trajectory', color='b')
        if display_pc:
            ax.scatter(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2], linewidth='0.2', label='Point Cloud', color='r')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_zlim(0, 200)
        ax.set_xlim(-100, 100)
        ax.set_ylim(-100, 100)
        # plt.xlim([-5,4])
        # plt.ylim([-3,3])

    plt.gca().invert_yaxis()
    plt.title('Trajectory and pointcloud')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    apply_linearT(etol = 1000)
    apply_nonlinearT()
    apply_linearPnP()
    apply_nonlinearPnP()
    apply_bundle()
    display_point_cloud(visual='2D')


