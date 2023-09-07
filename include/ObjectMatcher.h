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


#ifndef OBJECTMATCHER_H
#define OBJECTMATCHER_H

#include<vector>

#include"Object.h"
#include "Graph.h"
#include "Utils.h"


namespace ORB_SLAM2
{

class ObjectMatcher
{    
public:

    ObjectMatcher(Eigen::Matrix3d K);

    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    int MatchObjectsByProjection(Frame &F, std::unordered_map<Object*, Ellipse> proj_bboxes);

    int MatchObjectsHungarian(Frame &F, std::unordered_map<Object*, Ellipse> proj_bboxes);

    int MatchObjectsWasserDistance(Frame &F, std::unordered_map<Object*, Ellipse> proj_bboxes);
    
    int MatchObjectsIoU(Frame &F, std::unordered_map<Object*, Ellipse> proj_bboxes);

    float AssociateFeaturesWithObjects(Frame &CurrentFrame);

    void SearchInBox(KeyFrame *pKF, const vector<MapPoint*> &vpMapPoints, const vector<size_t> vIndices_in_box);

    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const vector<size_t> vIndices, const float th=3);

    int SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th, const std::vector<bool> vb_in_boxes);


public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;


protected:
    float RadiusByViewingCos(const float &viewCos);
    Eigen::Matrix3d K_;

};

}// namespace ORB_SLAM

#endif // OBJECTMATCHER_H
