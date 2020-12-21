import numpy as np
import matplotlib.pyplot as plt

def segmentIntersection(cur_x, cur_y, tar_x, tar_y, points):
    v1_x = cur_x[1] - cur_x[0]
    v1_y = cur_y[1] - cur_y[0]
    v2_x = tar_x[1] - tar_x[0]
    v2_y = tar_y[1] - tar_y[0]
    s = (-v1_y * (cur_x[0] - tar_x[0]) + v1_x * (cur_y[0] - tar_y[0])) / (-v2_x * v1_y + v1_x * v2_y)
    t = (v2_x * (cur_y[0] - tar_y[0]) - v2_y * (cur_x[0] - tar_x[0])) / (-v2_x * v1_y + v1_x * v2_y)
    p = np.zeros((1, 2))
    if 0 <= s <= 1 and 0 <= t <= 1:
        p[0, 0] = cur_x[0] + (t * v1_x)
        p[0, 1] = cur_y[0] + (t * v1_y)
        points = np.append(points, p, axis=0)
    return points

def intersection(cur_line_x, cur_line_y, target_line_x, target_line_y):
    cur_seg_x = np.c_[cur_line_x[:-1], cur_line_x[1:]]
    cur_seg_y = np.c_[cur_line_y[:-1], cur_line_y[1:]]
    tar_seg_x = np.c_[target_line_x[:-1], target_line_x[1:]]
    tar_seg_y = np.c_[target_line_y[:-1], target_line_y[1:]]
    cur_seg_x_min = np.min(cur_seg_x, 1)
    cur_seg_x_max = np.max(cur_seg_x, 1)
    cur_seg_y_min = np.min(cur_seg_y, 1)
    cur_seg_y_max = np.max(cur_seg_y, 1)
    tar_seg_x_min = np.min(tar_seg_x, 1)
    tar_seg_x_max = np.max(tar_seg_x, 1)
    tar_seg_y_min = np.min(tar_seg_y, 1)
    tar_seg_y_max = np.max(tar_seg_y, 1)
    points = np.empty((0,2))
    for i in range(len(cur_seg_x[:, 0])):
        for j in range(len(tar_seg_x[:, 0])):
            if cur_seg_x_min[i] > tar_seg_x_max[j] or tar_seg_x_min[j] > cur_seg_x_max[i]:
                continue
            if cur_seg_y_min[i] > tar_seg_y_max[j] or tar_seg_y_min[j] > cur_seg_y_max[i]:
                continue
            points = segmentIntersection(cur_seg_x[i, :], cur_seg_y[i, :], tar_seg_x[j, :], tar_seg_y[j, :], points)
            # print("points")
            # print(points)
    return points[:, 0], points[:, 1]

def checkIntersection(cur_line_x, cur_line_y, target_line_x, target_line_y):
    cur_seg_x = np.c_[cur_line_x[:-1], cur_line_x[1:]]
    cur_seg_y = np.c_[cur_line_y[:-1], cur_line_y[1:]]
    tar_seg_x = np.c_[target_line_x[:-1], target_line_x[1:]]
    tar_seg_y = np.c_[target_line_y[:-1], target_line_y[1:]]
    cur_seg_x_min = np.min(cur_seg_x, 1)
    cur_seg_x_max = np.max(cur_seg_x, 1)
    cur_seg_y_min = np.min(cur_seg_y, 1)
    cur_seg_y_max = np.max(cur_seg_y, 1)
    tar_seg_x_min = np.min(tar_seg_x, 1)
    tar_seg_x_max = np.max(tar_seg_x, 1)
    tar_seg_y_min = np.min(tar_seg_y, 1)
    tar_seg_y_max = np.max(tar_seg_y, 1)
    for i in range(len(cur_seg_x[:, 0])):
        for j in range(len(tar_seg_x[:, 0])):
            if cur_seg_x_min[i] > tar_seg_x_max[j] or tar_seg_x_min[j] > cur_seg_x_max[i]:
                continue
            if cur_seg_y_min[i] > tar_seg_y_max[j] or tar_seg_y_min[j] > cur_seg_y_max[i]:
                continue
            return True
    return False
