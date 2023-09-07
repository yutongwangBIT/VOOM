import numpy as np
import json
import matplotlib.pyplot as plt
import os

filenames = json.load(open('support_files/diamond_vr_slow.json'))
rgbs = filenames['rgb']
depths = filenames['depth']
f = open('support_files/diamond_vr_slow_associated.txt','a')
for rgb, depth in zip(rgbs, depths):
    _, filename_rgb = os.path.split(rgb)
    time_rgb = filename_rgb[:-4]
    filename_rgb = 'rgb/' + filename_rgb
    _, filename_depth = os.path.split(depth)
    time_depth = filename_depth[:-4]
    filename_depth = 'depth/' + filename_depth
    line = time_rgb + ' ' + filename_rgb + ' ' + time_depth + ' ' + filename_depth + '\n'
    f.write(line)
'''for i in range(1508):
    time = (i+1)/3.0
    line = str(time) + ' rgb/' + str(i+1) + '.png ' + str(time) + ' depth/' + str(i+1) + '.png' + '\n'
    f.write(line)'''
f.close()