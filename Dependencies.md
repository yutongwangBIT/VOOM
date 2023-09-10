##List of Known Dependencies
###VOOM version 1.0

In this document we list all the pieces of code included by VOOM and linked libraries that are not the property of the authors of VOOM.


#####Code in **src** and **include** folders

* *ORBextractor.cc*.
This is a modified version of orb.cpp of OpenCV library. The original code is BSD licensed.

* *PnPsolver.h, PnPsolver.cc*.
This is a modified version of the epnp.h and epnp.cc of Vincent Lepetit. 
This code can be found in popular BSD licensed computer vision libraries as [OpenCV](https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/epnp.cpp) and [OpenGV](https://github.com/laurentkneip/opengv/blob/master/src/absolute_pose/modules/Epnp.cpp). The original code is FreeBSD.

* Function *ORBmatcher::DescriptorDistance* in *ORBmatcher.cc*.
The code is from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel.
The code is in the public domain.

* *Viewer.h, Viewer.cc, Framedrawer.cc, Framedrawer.h, Mapdrawer.cc, Mapdrawer.h*
  This is a modified version based on ORB SLAM2 and OA-SLAM.

* *Tracking.h, Tracking.cc, LocalMapping.h, LocalMapping.cc, System.h, System.cc*
  This is a modified version based on ORB SLAM2

#####Code in Thirdparty folder

* All code in **DBoW2** folder.
This is a modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib) library. All files included are BSD licensed.

* All code in **g2o** folder.
This is a modified version of [g2o](https://github.com/RainerKuemmerle/g2o). All files included are BSD licensed.

* Code in **Json** folder.
This is an unchanged version of [Json](https://github.com/nlohmann/json). Their licenses include Apache BSD GPLv3 and MIT.

* Code in **Osmap** folder.
This is a modified version of [Osmap](https://github.com/AlejandroSilvestri/osmap). All files included are GPLv3 licensed.

#####Library dependencies 

* **Pangolin (visualization and user interface)**.
[MIT license](https://en.wikipedia.org/wiki/MIT_License).

* **OpenCV**.
BSD license.

* **Eigen3**.
For versions greater than 3.1.1 is MPL2, earlier versions are LGPLv3.





