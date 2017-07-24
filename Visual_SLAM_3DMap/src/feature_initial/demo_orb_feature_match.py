"""""""""""""""""""""""""""""""""""""""""""""
DESCRIPTION
This file aims to implement feature detection and matching.
It contains RANSAC to deal with outliers problem, image color contrast to deal with dark region feature detection problem.
When execute the code file, you can see a figure on the screen to show the match result before and after image color contrast.
"""""""""""""""""""""""""""""""""""""""""""""

import cv2
import numpy as np
import matplotlib.pyplot as plt

# generate homogeneous coordinate
def gen_pos(matches, img1, img2, kp1, kp2):
    good, pts1, pts2 = [], [], []
    total = len(matches)
    for i in range(total):
        match_cur = matches[i]

        good.append(match_cur)
        pts1.append(kp1[match_cur.queryIdx].pt)
        pts2.append(kp2[match_cur.trainIdx].pt)

    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)

    pts1 = np.hstack([pts1, np.ones((pts1.shape[0], 1))])
    pts2 = np.hstack([pts2, np.ones((pts2.shape[0], 1))])
    return pts1, pts2


# Estimate fundamental matrix
def FundmentEst(x1, x2):
    total = x1.shape[0]
    A = np.ones((total, 9))
    A[:, 0], A[:, 1], A[:, 2], A[:, 3] = np.einsum('i,i->i', x1[:, 0], x2[:, 0]), np.einsum('i,i->i', x1[:, 1], x2[:, 0]), x2[:, 0], np.einsum('i,i->i', x1[:, 0], x2[:, 1])
    A[:, 4], A[:, 5], A[:, 6], A[:, 7] = np.einsum('i,i->i', x1[:, 1], x2[:, 1]), x2[:, 1], x1[:, 0], x1[:, 1]

    u, d, v = np.linalg.svd(A)
    f = v[8, :]

    F_r = np.zeros((3, 3))
    F_r[0, 0], F_r[0, 1], F_r[0, 2] = f[0], f[1], f[2]
    F_r[1, 0], F_r[1, 1], F_r[1, 2] = f[3], f[4], f[5]
    F_r[2, 0], F_r[2, 1], F_r[2, 2] = f[6], f[7], f[8]

    u, d, v = np.linalg.svd(F_r)
    clean_d = np.array([[d[0], 0, 0], [0, d[1], 0], [0, 0, 0]])

    F = np.einsum('ij, jk, kl->il', u, clean_d, v)
    return F

# RANSAC
def do_RANSAC(matches, im_query, im_train, kp_query, kp_train, pos_query, pos_train, threshold):
    total = len(matches)
    max_vote, opt_F, ind_inlier = 0, np.zeros((3, 3)), 0

    for k in range(10000):
        ind = np.random.randint(0, total, (8,))
        # picked = [matches[i] for i in list(ind)]
        # F = FM.fundamental_Epi(picked, im_query, im_train, kp_query, kp_train, False)

        F = FundmentEst(pos_query[ind], pos_train[ind])
        lines = np.einsum('ji', np.einsum('ij, jk->ik', F, np.einsum('ji', pos_query)))

        error = np.einsum('ij, ij->ij', pos_train, lines)
        sum_error = np.abs(np.sum(error, axis=1)) / np.sqrt(np.square(lines[:, 0]) + np.square(lines[:, 1]))

        vote = np.sum(sum_error < threshold)
        if vote > max_vote:
            max_vote = vote
            ind_inlier = (sum_error < threshold)

            if max_vote > 0.9 * total:
                break

    pos_query_in, pos_train_in = pos_query[ind_inlier], pos_train[ind_inlier]
    opt_F = FundmentEst(pos_query_in, pos_train_in)

    ind = np.arange(0, total)
    matches_good = [matches[i] for i in list(ind[ind_inlier])]

    return matches_good, opt_F, ind_inlier, pos_query_in, pos_train_in


# feature match with original images
def orb_feature_original(im_query, im_train):
    orb = cv2.ORB_create()

    kp_query, des_query = orb.detectAndCompute(im_query, None)
    kp_train, des_train = orb.detectAndCompute(im_train, None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # Match descriptors.
    matches = bf.match(des_query, des_train)

    # Sort them in the order of their distance.
    # matches = sorted(matches, key=lambda x: x.distance)

    # RANSAC
    pos_query, pos_train = gen_pos(matches, im_query, im_train, kp_query, kp_train)
    threshold = 5
    matches_good, opt_F, ind_inlier, pos_query_in, pos_train_in = do_RANSAC(matches, im_query, im_train, kp_query, kp_train, pos_query, pos_train, threshold)

    # match features
    img_match = cv2.drawMatches(im_query, kp_query, im_train, kp_train, matches_good[: 100], flags=2, outImg=1)
    return img_match


# feature match with color contrast images
def orb_feature_ReduceContrast(im_query, im_train):
    # cv2.FastFeatureDetector_create()
    maxIntensity = 255.0 # depends on dtype of image data
    x = np.arange(maxIntensity)

    # Parameters for manipulating image data
    phi = 1
    theta = 1
    # Increase intensity such that
    # dark pixels become much brighter,

    # bright pixels become slightly bright
    # refer: http://stackoverflow.com/questions/39308030/how-do-i-increase-the-contrast-of-an-image-in-python-opencv
    newImage_query = (maxIntensity / phi) * (im_query / (maxIntensity / theta)) ** 0.5
    im_query = np.array(newImage_query, dtype = np.uint8)

    newImage_train = (maxIntensity / phi) * (im_train / (maxIntensity / theta)) ** 0.5
    im_train= np.array(newImage_train, dtype = np.uint8)

    # cv2.imshow('bright', im_query)
    # cv2.waitKey(20000)

    query_hsv = cv2.cvtColor(im_query, cv2.COLOR_BGR2HSV)
    train_hsv = cv2.cvtColor(im_train, cv2.COLOR_BGR2HSV)

    # refer: http://stackoverflow.com/questions/19363293/whats-the-fastest-way-to-increase-color-image-contrast-with-opencv-in-python-c
    query_hsv[:, :, 2] = [[max(pixel - 25, 0) if pixel < 190 else min(pixel + 25, 255) for pixel in row] for row in query_hsv[:, :, 2]]
    # cv2.imshow('contrast', cv2.cvtColor(query_hsv, cv2.COLOR_HSV2BGR))

    train_hsv[:, :, 2] = [[max(pixel - 25, 0) if pixel < 190 else min(pixel + 25, 255) for pixel in row] for row in train_hsv[:, :, 2]]
    # cv2.imshow('contrast', cv2.cvtColor(train_hsv, cv2.COLOR_HSV2BGR))

    # cv2.waitKey(20000)
    # cv2.destroyAllWindows()
    # exit(1)

    im_query = cv2.cvtColor(query_hsv, cv2.COLOR_HSV2BGR)
    im_train = cv2.cvtColor(train_hsv, cv2.COLOR_HSV2BGR)
    orb = cv2.ORB_create(nfeatures = 1000)

    kp_query, des_query = orb.detectAndCompute(im_query, None)
    kp_train, des_train = orb.detectAndCompute(im_train, None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
    # Match descriptors.
    matches = bf.match(des_query, des_train)

    # RANSAC
    pos_query, pos_train = gen_pos(matches, im_query, im_train, kp_query, kp_train)
    threshold = 5
    matches_good, opt_F, ind_inlier, pos_query_in, pos_train_in = do_RANSAC(matches, im_query, im_train, kp_query, kp_train, pos_query, pos_train, threshold)

    img_match = cv2.drawMatches(im_query, kp_query, im_train, kp_train, matches_good[: 100], flags=2, outImg=1)
    return img_match


if __name__ == '__main__':
    # change the name of images as you want
    im_query = cv2.imread('data/test1.jpg') # query Image
    im_train = cv2.imread('data/test2.jpg') # train Image

    im_ori = orb_feature_original(im_query, im_train)
    im_ReduceCon = orb_feature_ReduceContrast(im_query, im_train)

    # Three subplots sharing both x/y axes
    f, (ax1, ax2) = plt.subplots(2, 1, figsize = (8, 8))
    ax1.imshow(im_ori)
    ax1.set_title('Feature Match Result Before Image Contrast')

    ax2.imshow(im_ReduceCon)
    ax2.set_title('Feature Match Result After Image Contrast')
    plt.show()

