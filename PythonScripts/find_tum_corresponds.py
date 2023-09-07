import numpy as np
import json
import os
import glob
import yaml
from yaml.loader import SafeLoader

# Open the file and load the file
rgb_files = sorted(filter(os.path.isfile, glob.glob(
    '/home/yutong/data/rgbd_dataset_freiburg2_large_with_loop/rgb/' + '*.png')))
rgb_filenames = []
for file in rgb_files:
    head, filename = os.path.split(file)
    filename = filename[:-4]
    rgb_filenames.append(float(filename))
print('r',len(rgb_filenames))

depth_files = sorted(filter(os.path.isfile, glob.glob(
    '/home/yutong/data/rgbd_dataset_freiburg2_large_with_loop/depth/' + '*.png')))
depth_filenames = []
for file in depth_files:
    head, filename = os.path.split(file)
    filename = filename[:-4]
    depth_filenames.append(float(filename))
print('d',len(depth_filenames))

corres_depth = []
j = 0
for i in range(len(rgb_filenames)):
    rgb_time = rgb_filenames[i]
    delta_time_min = 10
    while j < len(depth_filenames) and abs(rgb_time - depth_filenames[j]) < delta_time_min:
        delta_time_min = abs(rgb_time - depth_filenames[j])
        j = j+1
    j = j - 1
    print(rgb_time, depth_filenames[j])
    corres_depth.append(depth_files[j])

gt = np.loadtxt('/home/yutong/data/rgbd_dataset_freiburg2_large_with_loop/groundtruth.txt', delimiter=' ')
timestamp_gt = list(np.array(gt)[:, 0])
#print(len(timestamp_gt))
corres_gt = []
j = 0
for i in range(len(rgb_filenames)):
    rgb_time = rgb_filenames[i]
    delta_time_min = 100
    while j < len(timestamp_gt) and abs(rgb_time - timestamp_gt[j]) < delta_time_min:
        delta_time_min = abs(rgb_time - timestamp_gt[j])
        j = j+1
    #j = j - 1
    #print(rgb_time, timestamp_gt[j - 1])
    corres_gt.append(list(gt[j - 1]))

dict_all = {"rgb": rgb_files, "depth": depth_files, "gt": corres_gt}
with open('support_files/rgbd_dataset_freiburg2_large_with_loop.json', 'w') as outfile:
    json.dump(dict_all, outfile)