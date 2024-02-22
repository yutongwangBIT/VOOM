# VOOM: Robust Visual Object Odometry and Mapping using Hierarchical Landmarks

VOOM is a real-time visual SLAM library that uses high-level objects and low-level points as hierarchical landmarks in a coarse-to-fine manner. It computes the camera trajectory and a sparse 3D reconstruction. 

This work has been accepted by ICRA 2024 :tada: [[pdf](todo)] [[video](https://www.bilibili.com/video/BV1w14y1C7Jb/)].

## Abstract
We propose a Visual Object Odometry and Mapping framework (VOOM) using high-level objects and low-level points as the hierarchical landmarks in a coarse-to-fine manner instead of directly using object residuals in bundle adjustment. Firstly, we introduce an improved observation model and a novel data association method for dual quadrics, employed to represent physical objects. It facilitates the creation of a 3D map that closely reflects reality. Next, we use object information to enhance the data association of feature points and consequently update the map. In our visual object odometry backend, the updated map is employed to further optimize the camera pose and the objects. At the same time, local bundle adjustment is performed utilizing the objects and points-based covisibility graphs in our visual object mapping process. Our experiments demonstrate that the localization accuracy of the proposed VOOM not only exceeds that of other object-oriented SLAM but also surpasses that of feature points SLAM systems such as ORB-SLAM2. The videos of the results can be found at: https://www.bilibili.com/video/BV1w14y1C7Jb/ .

## Prerequisites
### Need Install
- **Pangolin**
We use Pangolin for visualization and user interface. Download and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.
- **OpenCV**
We use OpenCV to manipulate images and features. Download and install instructions can be found at: http://opencv.org. Required at least 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2.
- **Eigen3**
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. Required at least 3.1.0.
- **Dlib**
We use Dlib in the comparison object data association method used by OA-SLAM. Download and install instructions can be found at: https://github.com/davisking/dlib
- **Protocol Buffers**
It is used for Osmap (see below). Download and install instructions can be found at: https://github.com/protocolbuffers/protobuf

### Included in the Thirdparty folder
- DBoW2 and g2o 
We use modified versions of the DBoW2 library to perform place recognition and g2o library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the Thirdparty folder.
- [Json](https://github.com/nlohmann/json) for I/O json files.
- [Osmap](https://github.com/AlejandroSilvestri/osmap) for map saving/loading. Modified version to handle objects.

## Compilation

1. Clone the repository recursively:

    ```git clone https://github.com/yutongwangBIT/VOOM.git VOOM```
3. Build:
 
   ```sh build.sh```

## Data
1. [TUM RGBD](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download)
2. [LM Data](https://peringlab.org/lmdata/) Diamond sequences

Our system takes instance segmentation as input. We provide detections in JSON files in the Data folder. We used an off-the-shelf version of YOLOv8, the Python script to prepare the JSON file is in the PythonScripts folder. 
The camera parameters are available in the Cameras folder.

## Run our system
All command lines can be found in https://github.com/yutongwangBIT/VOOM/blob/main/script

An example usage on TUM Fr2_desk sequence:
```
cd bin/
./rgbd_tum_with_ellipse ../Vocabulary/ORBvoc.txt ../Cameras/TUM2.yaml PATH_TO_DATASET ../Data/fr2_desk/fr2_desk.txt ../Data/fr2_desk/detections_yolov8x_seg_tum_rgbd_fr2_desk_with_ellipse.json points fr2_desk
```

## License
VOOM is released under a [GPLv3](https://github.com/yutongwangBIT/VOOM/blob/main/LICENSE) license. For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/yutongwangBIT/VOOM/blob/main/Dependencies.md).

## Citation
If you use VOOM in an academic work, please cite our paper:

	@inproceedings{wang2024icra,
		author = {Yutong Wang and Chaoyang Jiang and Xieyuanli Chen},
		title = {{VOOM: Robust Visual Object Odometry and Mapping using Hierarchical Landmarks}},
		booktitle = {Proc. of the IEEE Intl. Conf. on Robotics \& Automation (ICRA)},
		year = 2024
	}
 
