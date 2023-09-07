import numpy as np
import os
import json
import scipy.spatial.transform as trans
from camera_pose_object import CameraPoseObject
from camera_pose_object import Calib


def get_gt(gt, calib_K):
    cam_t = np.array([gt[1], gt[2], gt[3]])
    gt_r = trans.Rotation.from_quat([gt[4], gt[5], gt[6], gt[7]]).as_matrix()
    r_gt_cam = np.eye(3)
    r_gt_cam[1, 1] = -1
    r_gt_cam[2, 2] = -1
    cam_r = np.dot(gt_r, r_gt_cam)
    cam_pose_i = CameraPoseObject(position=cam_t, rotation=cam_r)
    P = cam_pose_i.get_P(calib_K)
    return cam_pose_i, P


calib = Calib(PP=[320.0, 240.0], focal=[600.0, 600.00])
filenames = json.load(open('support_files/diamond_dji.json'))
data = np.zeros((len(filenames['gt']), 8))
for i in range(len(filenames['gt'])):
    rgb_file = filenames['rgb'][i]
    head, filename = os.path.split(rgb_file)
    timestamp = round(float(filename[:-4])*1e-10,3)
    #print(timestamp)
    cam_pose, _ = get_gt(filenames['gt'][i], calib_K=calib.K)
    position = cam_pose.position
    quat = cam_pose.quaternion
    data[i, 0] = timestamp
    data[i, 1:4] = position
    data[i, 4:] = quat
np.savetxt('support_files/diamond_dji_gt.txt', data, fmt='%f', delimiter=' ')