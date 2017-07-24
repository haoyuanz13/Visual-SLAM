import numpy as np
import random as rand


# rotation matrix to quaternions
def rot_to_qua(rot):
    r11, r12, r13 = rot[0, 0], rot[0, 1], rot[0, 2]
    r21, r22, r23 = rot[1, 0], rot[1, 1], rot[1, 2]
    r31, r32, r33 = rot[2, 0], rot[2, 1], rot[2, 2]

    qw = 0.5 * np.sqrt(r11 + r22 + r33 + 1)
    qx = (r32 - r23) / float(4 * qw)
    qy = (r13 - r31) / float(4 * qw)
    qz = (r21 - r12) / float(4 * qw)

    qua = np.array([qw, qx, qy, qz])
    return qua


# quaternions to rotation matrix
def qua_to_rot(qua):
    qw, qx, qy, qz = qua[0], qua[1], qua[2], qua[3]
    r11 = 1 - 2 * np.square(qy) - 2 * np.square(qz)
    r12 = 2 * qx * qy - 2 * qz * qw
    r13 = 2 * qx * qz + 2 * qy * qw

    r21 = 2 * qx * qy + 2 * qz * qw
    r22 = 1 - 2 * np.square(qx) - 2 * np.square(qz)
    r23 = 2 * qy * qz - 2 * qx * qw

    r31 = 2 * qx * qz - 2 * qy * qw
    r32 = 2 * qy * qz + 2 * qx * qw
    r33 = 1 - 2 * np.square(qx) - 2 * np.square(qy)

    rot_new = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
    return rot_new


# quaternions to euler angle
def qua_to_ea(qua):
    w, x, y, z = qua[0], qua[1], qua[2], qua[3]

    angle_x = np.arctan2(2 * (w * x + y * z), 1 - 2 * (np.square(x) + np.square(y)))
    angle_y = np.arcsin(2 * (w * y - z * x))
    angle_z = np.arctan2(2 * (w * z + x * y), 1 - 2 * (np.square(y) + np.square(z)))

    res = [angle_x, angle_y, angle_z]
    return res


# euler angle to quaternions
def ea_to_qua(r, p, y):
    r = float(r)
    p = float(p)
    y = float(y)

    qw = np.cos(r / 2) * np.cos(p / 2) * np.cos(y / 2) + np.sin(r / 2) * np.sin(p / 2) * np.sin(y / 2)
    qx = np.sin(r / 2) * np.cos(p / 2) * np.cos(y / 2) - np.cos(r / 2) * np.sin(p / 2) * np.sin(y / 2)
    qy = np.cos(r / 2) * np.sin(p / 2) * np.cos(y / 2) + np.sin(r / 2) * np.cos(p / 2) * np.sin(y / 2)
    qz = np.cos(r / 2) * np.cos(p / 2) * np.sin(y / 2) - np.sin(r / 2) * np.sin(p / 2) * np.cos(y / 2)

    res = np.array([qw, qx, qy, qz])
    return res


# value of quaternion
def mod_qua(qua):
    qua = np.square(qua)
    res = np.sqrt(np.sum(qua))
    return res


# norm of quaternion
def norm_qua(qua):
    return np.square(mod_qua(qua))


# normalized form of quaternion
def normalize_qua(qua):
    divi = np.array(mod_qua(qua))
    qua = qua / divi
    return qua


# conjugate form
def conj_qua(qua):
    if qua.ndim == 1:
        qua = qua.reshape(1, 4)
    total, n = qua.shape[0], qua.shape[1]
    t = np.tile(np.array([1, -1, -1, -1]), (total, 1))
    qua  = qua * t
    if total == 1:
        qua = qua.reshape(-1)
    return qua

# inverse form
def inverse_qua(qua):
    nome = conj_qua(qua)
    if nome.ndim == 1:
        nome = nome.reshape(1, 4)
    total, n = qua.shape[0], qua.shape[1]
    divi = np.array(norm_qua(qua))
    di = np.tile(divi, (n, 1)).transpose()
    qua = nome/di
    if total == 1:
        qua = qua.reshape(-1)
    return qua

# multiplication (order matters)
def multi_quas(qua1, qua2):
    # print(qua1.ndim)
    if qua1.ndim == 1:
        qua1 = qua1.reshape(1, 4)
        qua2 = qua2.reshape(1, 4)
    # print(qua1.ndim)
    total1, n = qua1.shape[0], qua1.shape[1]
    total2, n = qua2.shape[0], qua2.shape[1]
    a, b, c, d = qua1[:, 0], qua1[:, 1], qua1[:, 2], qua1[:, 3]
    t, x, y, z = qua2[:, 0], qua2[:, 1], qua2[:, 2], qua2[:, 3]

    v1 = np.array([a * x, a * y, a * z])
    v2 = np.array([t * b, t * c, t * d])
    v3 = np.array([c * z - d * y,
                   d * x - b * z,
                   b * y - c * x])
    vec = v1 + v2 + v3

    sca = a * t - b * x - c * y - d * z
    num = max(total1, total2)
    res = np.zeros((num, n))
    res[:, 0] = sca
    for i in range(1, n):
        res[:, i] = vec[i - 1, :]
    if total1 ==1:
        res = res.reshape(-1)
    return res


# exp: construct a quaternion from a rotation vector
def exp_qua(qua):
    if qua.ndim == 1:
        qua = qua.reshape(1, 4)

    total, n = qua.shape[0], qua.shape[1]

    qs = qua[:, 0]
    qv = qua[:, 1:4]

    sca = np.cos(mod_qua(qv))

    di = np.tile(np.sin(mod_qua(qv)), (n - 1, 1)).transpose()
    vec = normalize_qua(qv) * di

    res = np.zeros((total, n))
    res[:, 0] = sca
    for i in range(total):
        if np.isnan(vec[i, :]).all():
            res[i, 1: 4] = np.array([0, 0, 0])
        else:
            res[i, 1: 4] = vec[i, :]

    amp = np.tile(np.exp(qs), (n, 1)).transpose()
    amp = amp * res
    if total == 1:
        amp = amp.reshape(-1)
    return amp


# log: recover a rotation vector from a quaternion
def log_qua(qua):
    qua = normalize_qua(qua)
    # print('qua;',qua)
    # qs = qua[0,:]
    # qv = qua[1:4,:]
    qs = qua[0]
    qv = qua[1:4]

    sca = np.log(mod_qua(qua))

    di = np.arccos(qs / mod_qua(qua))
    vec = normalize_qua(qv) * di

    total, n = 1, 4
    res = np.zeros((total, n))
    res[:, 0] = sca
    for i in range(total):
        if np.isnan(vec).all():
            res[i, 1:4] = np.array([0, 0, 0])
        else:
            res[i, 1:4] = vec[:]
    if total == 1:
        res = res.reshape(-1)
    return res


# quaternions averaging with different weights
def aver_quas_weight(quas, weight):
    total, n = quas.shape[0], quas.shape[1]

    ini_index = rand.randint(0, total - 1)
    mean_q = quas[ini_index, :].reshape(-1, n)

    iteration = 1000
    wei = np.tile(weight, (n - 1, 1)).transpose()

    for i in range(iteration):
        qi_e = multi_quas(quas, inverse_qua(mean_q))
        evi = 2 * log_qua(qi_e)[:, 1:4]

        sca = (-np.pi + np.mod(mod_qua(evi) + np.pi, 2 * np.pi)) / mod_qua(evi)
        index = np.tile(~np.isnan(sca), (n - 1, 1)).transpose()
        evi = evi * index * wei

        ev = np.sum(evi, axis=0) / 2
        temp = np.array([0, ev[0], ev[1], ev[2]]).reshape(-1, 4)
        mean_q = multi_quas(exp_qua(temp), mean_q)

        if mod_qua(temp * 2) < 0.0001:
            break

    return mean_q
