/**
* This file is part of OA-SLAM.
*
* Copyright (C) 2022 Matthieu Zins <matthieu.zins@inria.fr>
* (Inria, LORIA, Université de Lorraine)
* OA-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OA-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OA-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/


/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"
#include "ImageDetections.h"
#include "Ellipse.h"

#include <Eigen/Dense>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

struct DetectionWidget
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    DetectionWidget(BBox2 bb, unsigned int idx, unsigned int cat, double s, cv::Scalar col, 
                    double thick, bool disp_info, std::vector<int> neigh=std::vector<int>())
        : bbox(bb), id(idx), category_id(cat), score(s), color(col), 
          thickness(thick), display_info(disp_info), neighbors(neigh) {}

    BBox2 bbox;
    unsigned int id;
    unsigned int category_id;
    double score;
    cv::Scalar color;
    double thickness;
    bool display_info;
    std::vector<int> neighbors=std::vector<int>();
};

struct ObjectProjectionWidget
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    ObjectProjectionWidget(const Ellipse& ell, unsigned int idx, unsigned int cat, cv::Scalar col, bool is_in_map, double unc)
        : ellipse(ell), id(idx), category_id(cat), color(col), in_map(is_in_map), uncertainty(unc) {}
    Ellipse ellipse;
    unsigned int id;
    unsigned int category_id;
    cv::Scalar color;
    bool in_map;
    double uncertainty;
};

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker, bool force_reloc=false);

    // Draw last processed frame.
    cv::Mat DrawFrame();

    //cv::Mat DrawDetections(cv::Mat img);
    //cv::Mat DrawProjections(cv::Mat img);
    //CHANGED BY YUTONG TO SAVE COMPLETE IMGS
    cv::Mat DrawDetections(cv::Mat img, std::vector<DetectionWidget, Eigen::aligned_allocator<DetectionWidget>> dets=std::vector<DetectionWidget, Eigen::aligned_allocator<DetectionWidget>>());
    cv::Mat DrawProjections(cv::Mat img, std::vector<ObjectProjectionWidget, Eigen::aligned_allocator<ObjectProjectionWidget>> projs=std::vector<ObjectProjectionWidget, Eigen::aligned_allocator<ObjectProjectionWidget>>());

    void SetUseCategoryColors(bool use_cat_cols) {
        use_category_cols_ = use_cat_cols;
    }

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    vector<cv::Scalar> mvColor;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;

    Map* mpMap;
    std::vector<DetectionWidget, Eigen::aligned_allocator<DetectionWidget>> detections_widgets_;
    std::vector<ObjectProjectionWidget, Eigen::aligned_allocator<ObjectProjectionWidget>> object_projections_widgets_;

    std::mutex mMutex;
    std::mutex mMutex2;//added by yutong, cause she wants to use DrawDetections and DrawProjections in Tracking thread too.
    bool use_category_cols_ = false;
    int frame_id_;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
