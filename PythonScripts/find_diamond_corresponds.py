import numpy as np
import json
import os
import glob
import yaml
from yaml.loader import SafeLoader

# Open the file and load the file
rgb_files = sorted(filter(os.path.isfile, glob.glob(
    '/home/yutong/data/diamond_vr_slow/cam0/data/' + '*.png')))
rgb_filenames = []
for file in rgb_files:
    head, filename = os.path.split(file)
    filename = filename[:-4]
    rgb_filenames.append(float(filename))
print('r',len(rgb_filenames))

depth_files = sorted(filter(os.path.isfile, glob.glob(
    '/home/yutong/data/diamond_vr_slow/depth0/data/' + '*.png')))
depth_filenames = []
for file in depth_files:
    head, filename = os.path.split(file)
    filename = filename[:-4]
    depth_filenames.append(float(filename))
print('d',len(depth_filenames))


gt = np.loadtxt('/home/yutong/data/diamond_vr_slow/poses.txt', delimiter=',')

corres_gt = []
for i in range(len(gt)-1):
    tmp_gt = [gt[i][0], gt[i][1], gt[i][2], gt[i][3], gt[i][5], gt[i][6], gt[i][7], gt[i][4]]
    corres_gt.append(tmp_gt)

dict_all = {"rgb": rgb_files, "depth": depth_files, "gt": corres_gt}
with open('support_files/diamond_vr_slow.json', 'w') as outfile:
    json.dump(dict_all, outfile)