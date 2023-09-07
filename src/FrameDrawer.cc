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

#include "FrameDrawer.h"
#include "ImageDetections.h"
#include "Tracking.h"
#include "ColorManager.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0 && i < vIniKeys.size() && vMatches[i] < vCurrentKeys.size())
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK || state==Tracking::LOST) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                if(!(mvColor[i][0]==0 && mvColor[i][1]==0 && mvColor[i][2]==0 && mvColor[i][3]==0)){
                    cv::rectangle(im,pt1,pt2,mvColor[i]);
                    cv::circle(im,vCurrentKeys[i].pt,2,mvColor[i],-1);
                    mnTracked++; 
                    continue;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    //cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    //cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    s << " | FRAME "  << frame_id_;

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker, bool force_reloc)
{
    unique_lock<mutex> lock(mMutex);
    // pTracker->mImGray.copyTo(mIm);
    pTracker->im_rgb_.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mvColor = vector<cv::Scalar>(N, cv::Scalar(0,0,0,0));
    mbOnlyTracking = pTracker->mbOnlyTracking;
    frame_id_ = pTracker->GetCurrentFrameIdx();


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK || pTracker->mLastProcessedState==Tracking::LOST)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);

    //auto tracks = pTracker->GetObjectTracks();
    detections_widgets_.clear();

    //const auto& detections = pTracker->GetCurrentFrameDetections();

    
    /*for (auto det : detections) {
        detections_widgets_.push_back(DetectionWidget(det->bbox, 0, det->category_id,
                                        det->score, cv::Scalar(255, 255, 255),
                                        2, false)); //1.5
        
    }
    for (size_t i = 0; i < detections.size(); ++i){
        auto det = detections[i];
        std::vector<int> neighbours_i = graph->nodes[i];//graph->get_neighbours(i);
        detections_widgets_.push_back(DetectionWidget(det->bbox, 0, det->category_id,
                                        det->score, cv::Scalar(0, 0, 0),
                                        2, true, neighbours_i));
    }*/
    auto current_frame_id = pTracker->GetCurrentFrameIdx();
    /*for (auto tr : tracks) {
        if (current_frame_id == tr->GetLastObsFrameId()) {
            auto bb = tr->GetLastBbox();
            detections_widgets_.push_back(DetectionWidget(bb, tr->GetId(), tr->GetCategoryId(),
                                                            tr->GetLastObsScore(), tr->GetColor(),
                                                            3, true));
        }
    }*/ //OUTCOMMENTED BY YUTONG

    object_projections_widgets_.clear();
    cv::Mat cv_Rt = pTracker->mCurrentFrame.mTcw;
    cv::Mat cv_K = pTracker->mCurrentFrame.mK;
    if (cv_Rt.rows == 4 && cv_Rt.cols == 4) {

        Eigen::Matrix<double, 3, 4> Rt;
        Eigen::Matrix3d K;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                Rt(i, j) = cv_Rt.at<float>(i, j);
                if (j < 3)
                    K(i, j) = cv_K.at<float>(i, j);
            }
        }
        Eigen::Matrix<double, 3, 4> P = K * Rt;

        /*if(force_reloc){
            auto ellipsoids = pTracker->mCurrentFrame.ellipsoids;
            //std::cout<<"Viewer ellipsoids:"<<ellipsoids.size()<<std::endl;
            for (auto& ell : ellipsoids) {
                auto proj = ell.project(P);
                    object_projections_widgets_.push_back(ObjectProjectionWidget(proj, 0,
                                                                                 0, ell.tmp_color,
                                                                                 true,
                                                                                 0.5));
            }
        }
        else{
            for (auto& tr : tracks) {
            // if (tr->GetLastObsFrameId() == pTracker->GetCurrentFrameIdx()) {
                // if (tr->GetCategoryId() == 1 || tr->GetCategoryId() == 2 || tr->GetCategoryId() == 3 || tr->GetCategoryId() == 9 ||tr->GetCategoryId() == 10 || tr->GetCategoryId() == 11) continue;

                const auto* obj = tr->GetMapObject();
                if (obj) {
                    auto proj = obj->GetEllipsoid().project(P);
                    object_projections_widgets_.push_back(ObjectProjectionWidget(proj, tr->GetId(),
                                                                                 tr->GetCategoryId(), tr->GetColor(),
                                                                                 tr->GetStatus() == ObjectTrackStatus::IN_MAP,
                                                                                 tr->unc_));
                }
            // }
            }
            
        }*/ //OUTCOMMENTED BY YUTONG
        for(auto [node_id, attribute] : pTracker->mCurrentFrame.graph->attributes){
            if(attribute.obj){
                auto proj = attribute.obj->GetEllipsoid().project(P);
                object_projections_widgets_.push_back(ObjectProjectionWidget(proj, attribute.obj->GetId(),
                                                                            attribute.obj->GetCategoryId(), 
                                                                            attribute.obj->GetColor(),
                                                                            true,
                                                                            attribute.confidence));
                auto c = attribute.obj->GetColor();
                auto bb = attribute.bbox;
                detections_widgets_.push_back(DetectionWidget(bb, 0, attribute.label,
                                        attribute.confidence, cv::Scalar(255, 255, 255),
                                        2, false));
                vector<size_t> indecies_in_box = pTracker->mCurrentFrame.GetFeaturesInBox(bb[0], bb[2], bb[1], bb[3]);
                for(auto ind : indecies_in_box){
                    mvColor[ind] = c;
                }
            }
        }
    }
    
    //std::cout << "FrameDrawer updated, det:" << detections.size() << "," << detections_widgets_.size() << std::endl;
}


cv::Mat FrameDrawer::DrawDetections(cv::Mat img, std::vector<DetectionWidget, Eigen::aligned_allocator<DetectionWidget>> dets)
{   
    unique_lock<mutex> lock(mMutex2);
    std::vector<DetectionWidget, Eigen::aligned_allocator<DetectionWidget>> detections;
    {
        unique_lock<mutex> lock(mMutex);
        if (dets.empty())
            detections = detections_widgets_;
        else
            detections = dets;
    }
    //std::cout<<"detections_widgets_ size:"<<detections_widgets_.size()<<std::endl;
    if (detections.empty())
        return img;
    //std::cout<<"DrawDetections size:"<<detections.size()<<std::endl;
    const auto& manager = CategoryColorsManager::GetInstance();
    cv::Scalar color;
    for (size_t i = 0; i < detections.size(); ++i){
        auto d = detections[i];
        const auto& bb = d.bbox;
        if (use_category_cols_) {
            color = manager[d.category_id];
        } else {
            color = d.color;
        }
        cv::rectangle(img, cv::Point2i(bb[0], bb[1]),
                           cv::Point2i(bb[2], bb[3]),
                           color,
                           d.thickness);
        if (d.display_info) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << d.score;
            // cv::putText(img, std::to_string(d.id) + "(" + std::to_string(d.category_id) + ") | " + ss.str(),
            cv::putText(img, std::to_string(i) + "|" + ss.str() + "|" + std::to_string(d.category_id),
                        cv::Point2i(bb[0]-10, bb[1]-5), cv::FONT_HERSHEY_DUPLEX,
                        0.55, cv::Scalar(255, 255, 255), 1, false);
        }
        //ADDED FOR GRAPH
        if (!d.neighbors.empty()){
            for (auto n_id : d.neighbors){
            auto n_bb = detections[n_id].bbox;
            auto center1 = (bb.segment(0,2) + bb.segment(2,2)) / 2.0;
            auto center2 = (n_bb.segment(0,2) + n_bb.segment(2,2)) / 2.0;
            cv::line(img,cv::Point2i(center1[0], center1[1]),cv::Point2i(center2[0], center2[1]),
                        cv::Scalar(0,255,0),2);
            }
        }


    }
    return img;
}

void draw_ellipse_dashed(cv::Mat img, const Ellipse& ell, const cv::Scalar& color, int thickness)
{
    int size = 8;
    int space = 16;
    const auto& c = ell.GetCenter();
    const auto& axes = ell.GetAxes();
    double angle = ell.GetAngle();
    for (int i = 0; i < 360; i += space) {
        cv::ellipse(img, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]),
                    TO_DEG(angle), i, i+size, color, thickness);
    }
}

cv::Mat FrameDrawer::DrawProjections(cv::Mat img, std::vector<ObjectProjectionWidget, Eigen::aligned_allocator<ObjectProjectionWidget>> projs)
{
    unique_lock<mutex> lock(mMutex2);
    std::vector<ObjectProjectionWidget, Eigen::aligned_allocator<ObjectProjectionWidget>> projections;
    {
        unique_lock<mutex> lock(mMutex);
        if(projs.empty())
            projections = object_projections_widgets_;
        else
            projections = projs;
    }
    if (projections.empty())
        return img;
    //std::cout<<"DrawProjections:"<<projections.size()<<std::endl;
    const auto& manager= CategoryColorsManager::GetInstance();
    cv::Scalar color;
    
    for (auto w : projections)
    {
        const auto& ell = w.ellipse;
        const auto& c = ell.GetCenter();
        const auto& axes = ell.GetAxes();
        double angle = ell.GetAngle();
        if(axes[0] <= 0.001 || axes[1] <= 0.001)
            continue;
//        auto bb = ell.ComputeBbox();
        // auto [status_, bb] = find_on_image_bbox(ell, img.cols, img.rows);
        if (use_category_cols_) {
            color = manager[w.category_id];
        } else {
            color = w.color;
        }
        //std::cout<<"DrawProjections color:"<<color<<std::endl;
        //std::cout<<"DrawProjections center point:"<<c[0]<<","<<c[1]<<std::endl;
        //std::cout<<"DrawProjections axes:"<<axes[0]<<","<<axes[1]<<std::endl;
        if (w.in_map) {
            cv::ellipse(img, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]), TO_DEG(angle), 0, 360, color, 2);
            // stringstream ss;
            // ss << std::fixed << std::setprecision(3) << w.uncertainty;
            // cv::putText(img, ss.str(), cv::Point2f(c[0], c[1]), cv::FONT_HERSHEY_DUPLEX, 0.55, cv::Scalar(255, 255, 255), 1, false);
        }
        else
            draw_ellipse_dashed(img, ell, color, 2);
        // cv::rectangle(img, cv::Point2i(bb[0], bb[1]), cv::Point2i(bb[2], bb[3]), color, 2);
    }

    return img;
}

} //namespace ORB_SLAM
