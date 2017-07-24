"""""""""""""""""""""""""""""""""""""""""""""
DESCRIPTION
This file aims to implement feature tracking and store all detected features information.
The main algorithm refers to KLT (Kanade-Lucas-Tomasi Feature Tracker).
When executing the code, a video shows in the screen which contains feature detection and tracking result. 
"""""""""""""""""""""""""""""""""""""""""""""

from __future__ import print_function

import numpy as np
import cv2
import video
from common import anorm2, draw_str
from time import clock
import matplotlib.pyplot as plt
import scipy.io as sio


lk_params = dict( winSize  = (40, 40),  # (21, 21)
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.1, # 0.3
                       minDistance = 10,
                       blockSize = 5)

class App:
    def __init__(self, video_src):
        self.track_len = 20
        self.detect_interval = 5
        self.tracks = []
        self.cam = video.create_capture(video_src)
        self.frame_idx = 0
        # tracking index
        self.track_ind = []
        self.feature_num = 0

    def run(self):
        fea_pos, fea_frame = [], []
        while True:
            # set the constraints for total number of detected features
            if self.feature_num > 10000000:
                break

            ret, frame = self.cam.read()
            if frame is None:
                break

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            vis = frame.copy()
            if len(self.tracks) > 0:
                img0, img1 = self.prev_gray, frame_gray
                p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
                # find correspondences within two frames
                p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
                p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)

                d = abs(p0 - p0r).reshape(-1, 2).max(-1)
                good = d < 1
                new_tracks = []
                new_track_ind = []
                for ind_cur, tr, (x, y), good_flag in zip(self.track_ind, self.tracks, p1.reshape(-1, 2), good):
                    if not good_flag:
                        if len(tr) >= 2:
                            # store features that not track anymore
                            fea_pos.append(tr)  # first element represents position information
                            fea_frame.append(ind_cur) # second element represents frame in2formation
                            # np.save('%d' %self.feature_num, dict)
                            self.feature_num += 1
                        continue

                    tr.append((x, y))
                    ind_cur.append(self.frame_idx)
                    # set the size constraints for tracking window
                    if len(tr) > self.track_len:
                        del tr[0]
                        del ind_cur[0]

                    new_tracks.append(tr)
                    new_track_ind.append(ind_cur)

                    cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)

                self.tracks = new_tracks
                self.track_ind = new_track_ind

                cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
                draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))

            if self.frame_idx % self.detect_interval == 0:
                mask = np.zeros_like(frame_gray)
                mask[:] = 255
                for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                    cv2.circle(mask, (x, y), 5, 0, -1)
                p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                if p is not None:
                    for x, y in np.float32(p).reshape(-1, 2):
                        self.tracks.append([(x, y)])
                        self.track_ind.append([self.frame_idx])

            # frame
            # print(self.tracks)
            # print(self.track_ind)
            # print('---------------------------------------------------------------------------')
            self.frame_idx += 1
            self.prev_gray = frame_gray
            cv2.imshow('lk_track', vis)

            # ch = cv2.waitKey(10)
            ch = cv2.waitKey(50)
            if ch == 27:
                break

        # sio.savemat('feature_pos.mat', {'pos2d': fea_pos})
        # sio.savemat('feature_frame.mat', {'frame_ind': fea_frame})
        # np.save('feature_pos', fea_pos)
        # np.save('feature_frame', fea_frame)

        print('********Total number of detected features*************')
        print(len(fea_pos))
        print('number of frames:',self.frame_idx)



def main():
    try:
        video_src = 'data/perch_video.mp4'
        # video_src = 'Test_rgb_3VC.mp4'
    except:
        video_src = 0

    print(__doc__)
    App(video_src).run()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
