import numpy as np
import os
import cv2
import json
import math
from ultralytics import YOLO


def get_image(ori_path, new_path=None, is_color=True, is_tum=False):
    head, filename = os.path.split(ori_path)
    if is_color:
        if is_tum:
            return cv2.imread(ori_path)
            # return cv2.imread(new_path + filename[:-4] + '.png')
        return cv2.imread(new_path + filename)
    else:
        if is_tum:
            return cv2.imread(ori_path, -1)
        im_depth = cv2.imread(new_path + filename[:-4] + '.png', -1)
        return im_depth

def estimate_mask_contour(mask_box):
    mask_box = (mask_box * 255).astype(np.uint8)
    canny = cv2.Canny(mask_box, 100, 150)
    contours, hierarchies = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0:
        return None
    contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
    return contour

filenames = json.load(open('support_files/diamond_dji.json'))

predictor = YOLO('/home/yutong/QISO_SLAM/support_files/weights/yolov8x-seg.pt')
list_to_save = []
count = 0
for rgb_file in filenames['rgb']:
    count += 1
    #if count == 10:
        #break
    head, filename = os.path.split(rgb_file)
    dict_per_im = dict()
    dict_per_im["file_name"] = filename
    dict_per_im["detections"] = []
    im_rgb = cv2.imread(rgb_file)
    results = predictor.predict(im_rgb, save=False, conf=0.1, device=0, visualize=False, show=False)
    if results[0].masks is None:
        list_to_save.append(dict_per_im)
        continue
    boxes = results[0].boxes.to("cpu").numpy()
    masks = results[0].masks.to("cpu").numpy()
    for box_, cls, conf, mask in zip(boxes.xyxy, boxes.cls, boxes.conf, masks.data):
        y1, x1, y2, x2 = box_
        box = np.array([y1, x1, y2, x2], dtype=np.float64)
        contour = estimate_mask_contour(mask)
        if contour is None:
            continue
        if len(contour) < 10:
            continue
        ellipse = cv2.fitEllipse(contour)
        theta = ellipse[2] * math.pi / 180
        ellipse_data = np.array([ellipse[0][0], ellipse[0][1], ellipse[1][0], ellipse[1][1], theta],
                                dtype=np.float64)
        category_id = int(cls)
        det = dict()
        det["category_id"] = category_id
        det["detection_score"] = np.float64(conf)
        det["bbox"] = list(box)
        det["ellipse"] = list(ellipse_data)
        dict_per_im["detections"].append(det)
    list_to_save.append(dict_per_im)
with open('support_files/detections_yolov8x_seg_diamond_dji_with_ellipse.json', 'w') as outfile:
    json.dump(list_to_save, outfile)