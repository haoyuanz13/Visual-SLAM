import numpy as np


class PointCloud(object):
    def __init__(self, label=None, ini_3D=None, ini_2D=None, ini_frame=None, ini_Hw2c=None,cprams=None,inliners=None):
        '''all rotation and translations are respected to the global frame. '''
        self.label = label

        if ini_3D is not None:
            self.Pos3D = ini_3D
        else:
            self.Pos3D = None #np.zeros(3) # None

        if ini_2D is not None:
            self.Pos2D = [ini_2D]
        else:
            self.Pos2D = []

        if ini_frame is not None:
            self.frame = [ini_frame]
        else:
            self.frame = []

        if ini_Hw2c is not None:
            self.Hw2c = [ini_Hw2c]  # must be wrt the world frame
        else:
            self.Hw2c = []

        if cprams is not None:
            self.cam_params = [cprams]
        else:
            self.cam_params = []

        if inliners is not None:
            self.is_inliners = [inliners]
        else:
            self.is_inliners = []
        return

    # add new correspondences
    def newCorres(self, p2d=None, cam_ind=None, Hwc2=None, cprams=None, inliners=None):
        self.Pos2D.append(p2d)
        self.frame.append(cam_ind)
        self.Hw2c.append(Hwc2)
        if cprams is not None:
            self.cam_params.append(cprams)
        if inliners is not None:
            self.is_inliners.append(inliners)
        return

    # skew matrix
    def skewM(self, x):
        Skew = np.array([[0, -1, x[1]],
                         [1,  0, -x[0]],
                         [-x[1], x[0], 0]])
        return Skew

    def set_3d(self, p3d):
        self.Pos3D = p3d

    def set_Hw2c(self,Hw2c):
        self.Hw2c = Hw2c

    def set_inliners(self,inliners):
        '''Return a list of boolean, saying:
          1 - if the 2d point is in inliner set
          0 - Otherwise'''
        self.is_inliners = []
        try:
            for i in range(len(self.Pos2D)):
                if i in inliners:
                    self.is_inliners.append(1)
                else:
                    self.is_inliners.append(0)
        except:
            print('-- ERROR in set_inliners(): there is no 2d point!!!')











