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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>
//#include <opencv2/calib3d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"
#include "Ellipsoid.h"
#include "Ellipse.h"
#include "System.h"
#include "Utils.h"
#include "Graph.h"
#include "ObjectMatcher.h"
#include <iostream>

#include <mutex>
#include <unordered_set>
#include <unordered_map>
#include <unistd.h>
#include <Eigen/Dense>
#include <unistd.h>
#include <dlib/optimization/max_cost_assignment.h>
#include <chrono>

using namespace std;

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

namespace ORB_SLAM2
{


Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);
    K_ = cvToEigenMatrix<double, float, 3, 3>(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, const std::vector<Detection::Ptr>& detections, bool force_relocalize, bool bProcessDepth)
{
    current_frame_idx_ = (current_frame_idx_ + 1) % (std::numeric_limits<size_t>::max()-1);
    mImGray = imRGB;
    cv::Mat imDepth = imD;
    
    imRGB.copyTo(im_rgb_);

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,bProcessDepth);
    //ADDED BY YUTONG TO PROCESS DETECTION TO AVOID SIMILAR DETS
    current_frame_detections_.clear();
    for(auto det1 : detections){
        bool has_similar_det = false;
         //0.2, 300, 0.3, 5, 0.4 for diamond
        //0.2, 300, 0.1, 10, 0.4 for TUM fr2
        if(det1->score < 0.2 || bbox_area(det1->bbox) < 300 ||
            bbox_area(det1->bbox) > 0.5*im_rgb_.rows*im_rgb_.cols ||
            //is_near_boundary(det1->bbox, im_rgb_.cols, im_rgb_.rows, 10) ||
            bboxes_iou(det1->bbox, det1->ell.ComputeBbox())<0.2) 
            continue;
        //if(det1->category_id==0 || det1->category_id==56) continue; //person & chair
        for (auto det2 : current_frame_detections_ ){
            double bbox_iou = bboxes_iou(det1->bbox, det2->bbox);
            if(det1->category_id == det2->category_id){
                if(bbox_iou>0.3)
                    has_similar_det = true;
            }
            else{
                if(bbox_iou>0.6)
                    has_similar_det = true;
            }
        }
        if(!has_similar_det)
            current_frame_detections_.push_back(det1);
    }

    current_depth_data_per_det_.clear();
    current_depth_data_per_det_.resize(current_frame_detections_.size(), std::make_pair(0.0f, 0.0f));

    for (size_t i = 0; i < current_frame_detections_.size(); ++i) {
        auto det = current_frame_detections_[i];
        auto bbox = det->bbox;
        //choose random pixels to estimate
        int number_pixels = 30;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis_u(bbox[0], bbox[2]);
        std::uniform_int_distribution<> dis_v(bbox[1], bbox[3]);
        float sum_d = 0.0f;
        float min_d = 100.0;
        float max_d = -1.0;
        float count_avg = 0.0f;
        for(int j = 0; j<number_pixels; j++){
            int u = dis_u(gen);
            int v = dis_v(gen);
            float d = imDepth.at<float>(v,u);
            //std::cout<<d<<",";
            if(d>0.0f){
                sum_d += d;
                count_avg += 1.0f;
                if(d<min_d) min_d = d;
                if(d>max_d) max_d = d;
            }
        }
        //std::cout<<std::endl;
        if(count_avg>0.0f){
            current_depth_data_per_det_[i].first = std::min(sum_d/count_avg,5.0f);
            current_depth_data_per_det_[i].second = std::min(std::max(max_d - min_d, 0.05f), 0.2f);
        }
    }
    //current_frame_detections_ = detections;
    //current_frame_good_detections_.clear();
    /*for (auto det : current_frame_detections_) {
        if (det->score > 0.5) {
            // if (det->category_id != 73 && det->score > 0.5 ||  det->score > 0.7) { // for table scene to ignore book on the nappe
            current_frame_good_detections_.push_back(det);
        }
    }*/
    /*std::map<int, int> good_det_to_det;
    for (size_t i = 0; i < current_frame_detections_.size(); ++i){
        auto det = current_frame_detections_[i];
        if (det->score > 0.2 && !is_near_boundary(det->bbox, imRGB.cols, imRGB.rows, 10) && bbox_area(det->bbox)>300) {
            current_frame_good_detections_.push_back(det);
            good_det_to_det[current_frame_good_detections_.size()-1] = i;
            //std::cout<<"good_det_to_det:"<<(current_frame_good_detections_.size()-1)<<","<<i<<"/n";
        }
    }*/

    //ADDED BY YUTONG FOR GRAPH
    //std::cout<<"frame idx:"<<mCurrentFrame.mnId<<std::endl;
    mCurrentFrame.graph = new Graph();
    std::vector<Eigen::Vector2d> center_points; //TODO 3d points
    for (size_t i = 0; i < current_frame_detections_.size(); ++i) {
        BBox2 box = current_frame_detections_[i]->bbox;
        mCurrentFrame.graph->add_node(i, current_frame_detections_[i]->category_id,
            current_frame_detections_[i]->score, -1.0f, box, current_frame_detections_[i]->ell);
        Eigen::Vector2d center_point = (box.segment(0,2) + box.segment(2,2)) / 2.0;
        center_points.push_back(center_point);
    }
    int k = std::min(4, static_cast<int>(current_frame_detections_.size()-1));
    for (size_t i = 0; i < current_frame_detections_.size(); ++i) {
        std::vector<pair<int,double>> distances;
        for (size_t j = 0; j < current_frame_detections_.size(); ++j) {
            if (i != j){
                double distance = (center_points[i] - center_points[j]).norm();
                distances.push_back(make_pair(j,distance));
            }
        }
        sort(distances.begin(),distances.end(),[](auto& left, auto& right) { 
            return left.second < right.second; 
        });
        for (int m=0; m<k; m++){
            mCurrentFrame.graph->add_edge(i, distances[m].first);
        }
    }
    mCurrentFrame.graph->compute_feature_vectors();
    //current_frame_graph_ = mCurrentFrame.graph;
    
    //ENDED GENERATE GRAPH 

    if (force_relocalize)
    {
        auto t1 = high_resolution_clock::now();

        bool bOK = false;
        if (mpSystem->GetRelocalizationMode() == RELOC_POINTS) {
            std::cout << "Relocalize with points.\n";
            bOK = Relocalization();
        } else if (mpSystem->GetRelocalizationMode() == RELOC_OBJECTS) {
            std::cout << "Relocalize with objects.\n";
            bOK = RelocalizationFromObjects(true);
        } else if (mpSystem->GetRelocalizationMode() == RELOC_OBJECTS_POINTS) {
            std::cout << "Relocalize with objects and points.\n";
            bOK = Relocalization();
            if (!bOK)
                bOK = RelocalizationFromObjects(true);
        } else if (mpSystem->GetRelocalizationMode() == RELOC_GRAPH){
            bOK = RelocalizationWith2dGraph();
            //}
        }

        auto t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;

        
        if (bOK){
            mpFrameDrawer->Update(this, true);
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
        }
            

        mpSystem->relocalization_duration = ms_double.count();
        mpSystem->relocalization_status = bOK;
        if (!bOK) // ADDED BY YUTONG
        {
            cv::Mat a;
            return a;
        }
    }
    else
    {
        Track(true);
        mpFrameDrawer->Update(this);
        // ADDED END
    } 
    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp,
                                     const std::vector<Detection::Ptr>& detections, bool force_relocalize)
{
    std::cout<<"GrabImageMonocular:"<<std::endl;
    current_frame_idx_ = (current_frame_idx_ + 1) % (std::numeric_limits<size_t>::max()-1);
    mImGray = im;
    im.copyTo(im_rgb_);

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    current_frame_detections_ = detections;
    std::cout<<"current_frame_detections_:"<<current_frame_detections_.size()<<std::endl;
    current_frame_good_detections_.clear();
    for (auto det : current_frame_detections_) {
        if (det->score > 0.5) {
            // if (det->category_id != 73 && det->score > 0.5 ||  det->score > 0.7) { // for table scene to ignore book on the nappe
            current_frame_good_detections_.push_back(det);
        }
    }
    std::cout<<"current_frame_good_detections_:"<<current_frame_good_detections_.size()<<std::endl;

    if (force_relocalize)
    {
        auto t1 = high_resolution_clock::now();

        bool bOK = false;
        if (mpSystem->GetRelocalizationMode() == RELOC_POINTS) {
            std::cout << "Relocalize with points.\n";
            bOK = Relocalization();
        } else if (mpSystem->GetRelocalizationMode() == RELOC_OBJECTS) {
            std::cout << "Relocalize with objects.\n";
            bOK = RelocalizationFromObjects(false);
        } else if (mpSystem->GetRelocalizationMode() == RELOC_OBJECTS_POINTS) {
            std::cout << "Relocalize with objects and points.\n";
            bOK = Relocalization();
            if (!bOK)
                bOK = RelocalizationFromObjects(true);
        }

        auto t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;

        mpFrameDrawer->Update(this);
        if (bOK)
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mpSystem->relocalization_duration = ms_double.count();
        mpSystem->relocalization_status = bOK;
    }
    else
    {

        Track();
        // if (!mbOnlyTracking)    // if in localization-only mode, no neeed to track objects
        //     break;
        mpFrameDrawer->Update(this);
    }


    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track(bool use_object)
{
    createdNewKeyFrame_ = false;
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    // int debug = 0; // for starting a new map without reset

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD){
            StereoInitialization();
            if(use_object)
                ObjectsInitialization();
        }
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }

                // if (bOK)
                //     std::cout << "Tracking is OK\n";
                // else
                //     std::cout << "Tracking failed\n";
            }
            else
            {
                if (mpSystem->GetRelocalizationMode() == RELOC_POINTS) {
                    std::cout << "Relocalize with points.\n";
                    bOK = Relocalization();
                } else if (mpSystem->GetRelocalizationMode() == RELOC_OBJECTS) {
                    std::cout << "Relocalize with objects.\n";
                    bOK = RelocalizationFromObjects(true);
                } else if (mpSystem->GetRelocalizationMode() == RELOC_OBJECTS_POINTS) {
                    std::cout << "Relocalize with objects and points.\n";
                    bOK = Relocalization();
                    if (!bOK)
                        bOK = RelocalizationFromObjects(true);
                }

                if (bOK)
                    std::cout << "Relocalization is OK\n";
                else {
                    std::cout << "Relocalization failed\n";
                }
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated
            // std::cout << "Tracking: Mapping is disabled \n";
            if(mState==LOST)
            {
                if (mpSystem->GetRelocalizationMode() == RELOC_POINTS) {
                    std::cout << "Relocalize with points.\n";
                    bOK = Relocalization();
                } else if (mpSystem->GetRelocalizationMode() == RELOC_OBJECTS) {
                    std::cout << "Relocalize with objects.\n";
                    bOK = RelocalizationFromObjects(false);
                } else if (mpSystem->GetRelocalizationMode() == RELOC_OBJECTS_POINTS) {
                    std::cout << "Relocalize with objects and points.\n";
                    bOK = Relocalization();
                    if (!bOK)
                        bOK = RelocalizationFromObjects(true);
                }
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        ObjectMatcher obj_matcher = ObjectMatcher(K_);
        BBox2 img_bbox(0, 0, im_rgb_.cols, im_rgb_.rows);
        std::unordered_map<Object*, Ellipse> proj_bboxes;
        if(use_object && bOK){
            //std::cout<<"mCurrentFrame idx:"<<mCurrentFrame.mnId<<std::endl;
            //SAVE IM FOR DEBUGGING
            Matrix34d Rt = cvToEigenMatrix<double, float, 3, 4>(mCurrentFrame.mTcw);
            Matrix34d P;
            P = K_ * Rt;
            for(auto obj : mpMap->GetAllObjects()){
                //if(obj->GetLastObsFrameId() + 500 < mCurrentFrame.mnId  && !obj->GetFlagOptimized()){//already M frames not seen and it has been not usually seen
                    //TODO DELETE
                    //continue;
                //}
                auto proj = obj->GetEllipsoid().project(P);
                auto c3d = obj->GetEllipsoid().GetCenter();
                auto bb_proj = proj.ComputeBbox();
                double z = Rt.row(2).dot(c3d.homogeneous());
                if ( z < 0  || bboxes_intersection(bb_proj, img_bbox) < 0.3 * bbox_area(bb_proj) ) continue;
                proj_bboxes[obj] = proj;
                // Check occlusions and keep only the nearest
                std::unordered_set<Object*> hidden;
                for (auto it : proj_bboxes) {
                    if (it.first != obj && bboxes_iou(it.second.ComputeBbox(), bb_proj) > 0.8) {
                        Eigen::Vector3d c2 = it.first->GetEllipsoid().GetCenter();
                        double z2 = Rt.row(2).dot(c2.homogeneous());
                        if (z < z2) {
                            // remove z2
                            hidden.insert(it.first);
                        } else {
                            // remove z
                            hidden.insert(obj);
                        }
                        break;
                    }
                }
                for (auto hid : hidden) {
                    proj_bboxes.erase(hid);
                }
            }

            //for(auto& [obj, proj] : proj_bboxes){
            //    auto axes = proj.GetAxes();
            //    double angle = proj.GetAngle();
            //    auto c = proj.GetCenter();
            //    cv::ellipse(im_rgb_, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]), TO_DEG(angle), 0, 360, obj->GetColor(), 1);
            //}

            
            //int nmatches = obj_matcher.MatchObjectsByProjection(mCurrentFrame, proj_bboxes);
            //int nmatches = obj_matcher.MatchObjectsHungarian(mCurrentFrame, proj_bboxes);
            //int nmatches = obj_matcher.MatchObjectsIoU(mCurrentFrame, proj_bboxes);
            int nmatches = obj_matcher.MatchObjectsWasserDistance(mCurrentFrame, proj_bboxes);
            //std::cout<<"nmatches:"<<nmatches<<std::endl;

        }

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap(use_object);
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap(use_object);
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // // Update drawer
        // mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame()) {
                CreateNewKeyFrame();
                createdNewKeyFrame_ = true;
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        if(use_object && bOK){
            KeyFrame *kf = mpLastKeyFrame;
            if (!createdNewKeyFrame_)
                kf = nullptr;

            if(kf){
                Matrix34d Rt = cvToEigenMatrix<double, float, 3, 4>(mCurrentFrame.mTcw);
                Matrix34d P;
                P = K_ * Rt;

                for(auto [node_id, attribute] : mCurrentFrame.graph->attributes){
                    auto bb_det = attribute.bbox;
                    cv::rectangle(im_rgb_, cv::Point2i(bb_det[0], bb_det[1]),
                                            cv::Point2i(bb_det[2], bb_det[3]),
                                            cv::Scalar(255, 255, 255),
                                            2);
                    if(attribute.obj){
                        //std::cout<<"node "<<node_id<<" is matched with object "<<attribute.obj->GetId()<<std::endl;
                        //check iou again???
                        auto proj = attribute.obj->GetEllipsoid().project(P);
                        auto bb_proj = proj.ComputeBbox();
                        double iou = bboxes_iou(bb_proj, bb_det);
                        if(iou>0.01){
                            auto c = proj.GetCenter();
                            auto axes = proj.GetAxes();
                            double angle = proj.GetAngle();
                            //if(iou<0.3)
                            //    cv::ellipse(im_rgb_, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]), TO_DEG(angle), 0, 360, cv::Scalar(0, 0, 255), 2);
                            //else
                            //    cv::ellipse(im_rgb_, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]), TO_DEG(angle), 0, 360, cv::Scalar(0, 255, 0), 2);
                            if(axes[0] <= 0.001 || axes[1] <= 0.001)
                                continue;
                            //cv::ellipse(im_rgb_, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]), TO_DEG(angle), 0, 360, attribute.obj->GetColor(), 2);
                            attribute.obj->AddDetection(attribute.label, bb_det, attribute.ell, attribute.confidence, Rt, mCurrentFrame.mnId, kf);
                            //attribute.obj->AddDetection(attribute.label, bb_det, Ellipse::FromBbox(bb_det), attribute.confidence, Rt, mCurrentFrame.mnId, kf);
                            //proj_bboxes.erase(attribute.obj);
                            //double dis_min = normalized_gaussian_wasserstein_2d(proj, Ellipse::FromBbox(bb_det), 10);
                            //std::cout<<"wasser:"<<dis_min<<std::endl;
                            //cv::putText(im_rgb_,  std::to_string(attribute.hue), cv::Point2i(bb_det[0]-10, bb_det[1]-5), cv::FONT_HERSHEY_DUPLEX,
                            //    0.55, cv::Scalar(255, 255, 0), 1, false);
                        }
                        else{//TODO??
                            //std::cout<<"BUT IOU IS NOT ENOUGH"<<std::endl;
                            //attribute.obj = nullptr;
                            continue;
                        }
                    }
                }

                //cv::putText(im_rgb_,  std::to_string(average_intersect_ratio), cv::Point(15, 15), cv::FONT_HERSHEY_DUPLEX,
                                //0.55, cv::Scalar(255, 255, 0), 1, false);

                for(auto [node_id, attribute] : kf->graph->attributes){
                    if(!attribute.obj){
                        //std::cout<<"not asscociated node id:"<<node_id<<std::endl;
                        //TODO check if match new
                        //create new
                        Object* obj = new Object(attribute.label, attribute.bbox, attribute.ell, attribute.confidence, current_depth_data_per_det_[node_id], K_,
                                    Rt, mCurrentFrame.mnId, kf);
                        if(obj->GetAssociatedMapPoints().size()<5){
                            delete obj;
                            continue;
                        }
                        mpMap->AddObject(obj);
                        kf->graph->attributes[node_id].obj = obj;
                        auto proj = obj->GetEllipsoid().project(P);
                        auto c = proj.GetCenter();
                        auto axes = proj.GetAxes();
                        double angle = proj.GetAngle();
                        //cv::ellipse(im_rgb_, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]), TO_DEG(angle), 0, 360, cv::Scalar(0, 255, 255), 2);
                        //if(axes[0] <= 0.001 || axes[1] <= 0.001)
                        //    continue;
                        //cv::ellipse(im_rgb_, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]), TO_DEG(angle), 0, 360, obj->GetColor(), 2);
                    }
                }
                //std::string image_path = "/home/yutong/VOOM/im_save/keyframes/" + to_string(kf->mnId) + "_" + to_string(mCurrentFrame.mnId) + ".png";
                //cv::imwrite(image_path.c_str(), im_rgb_);
                //std::cout<<"saved im"<<std::endl;
            }

        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty() && mCurrentFrame.mpReferenceKF)
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}

void Tracking::ObjectsInitialization(){
    if(current_frame_detections_.empty()){
        std::cout<<"WARNING: NO DETECTION IN THE INITIALIZATION FRAME"<<std::endl;
    }
    //std::cout<<"current_frame_detections_ size:"<<current_frame_detections_.size()<<std::endl;
    Matrix34d Rt = cvToEigenMatrix<double, float, 3, 4>(mCurrentFrame.mTcw);
    int count = 0;
    for (size_t di = 0; di < current_frame_detections_.size(); ++di) {
        auto det = current_frame_detections_[di];
        //std::cout<<current_depth_data_per_det_[di].first<<",";
        if(current_depth_data_per_det_[di].first > 0.01f && current_depth_data_per_det_[di].first < 15.0f){
            Object* obj = new Object(det->category_id, det->bbox, det->ell, det->score, current_depth_data_per_det_[di], K_,
                          Rt, 0, mpReferenceKF);
            //std::cout<<"obj:"<<obj->GetId()<<" has mappoint size:"<<obj->GetAssociatedMapPoints().size()<<std::endl;
            mpMap->AddObject(obj);
            count += 1;
        }
    }
    std::vector<Object*> objects = mpMap->GetAllObjects();
    std::cout<<"count:"<<count<<"Map has "<<objects.size()<<" objects"<<std::endl;
}

void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        //added by YUTONG
        //if(!im_rgb_.empty())
        //    im_rgb_.copyTo(pKFini->im); 

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }
    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap(bool use_object)
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap(use_object);

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    //added by YUTONG
    //if(!im_rgb_.empty())
    //    im_rgb_.copyTo(pKF->im); 

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap(bool use_object)
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames(use_object);
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames(bool use_object) //YUTONG TODO
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;

    if(use_object){
        for(auto [node_id, attribute] : mCurrentFrame.graph->attributes){
            Object* obj = attribute.obj;
            if(!obj) continue;
            if(obj->isBad()) continue;
            auto mps = obj->GetAssociatedMapPoints();
            for(auto pMP : mps){
                if(pMP){
                    if(!pMP->isBad()){
                        const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                        for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                            keyframeCounter[it->first]++;
                    }
                }
            }
        }
    }
    
    if(keyframeCounter.empty()){
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                    for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::RelocalizationFromObjects(bool use_points)
{
    //TODO
    return false;
}

bool Tracking::RelocalizationWithGraph(){
    //TODO
    return false;
}

bool Tracking::RelocalizationWith2dGraph(){
    //TODO
    return false;
}


bool Tracking::Relocalization()
{
    mLastProcessedState = mState;

    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    std::cout<<"vpCandidateKFs:"<<vpCandidateKFs.size()<<std::endl;

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        //std::cout<<i<<std::endl;
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            //std::cout<<"nmatches:"<<nmatches<<std::endl;
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);
    int most_inliers = -1;
    int keyframe_idx_with_most_inliers = 0;

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            if(nInliers > most_inliers){ //ADDED BY YUTONG
                keyframe_idx_with_most_inliers = i;
                most_inliers = nInliers;
            }

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    // ADDED FOR DEBUG
    /*cv::Mat img_h;
    cv::Mat im_kf;
    cv::Mat im_f;
    im_rgb_.copyTo(im_f);
    vpCandidateKFs[keyframe_idx_with_most_inliers]->im.copyTo(im_kf);
    
    for(int i=0; i<mCurrentFrame.mvKeys.size(); i++)
        cv::circle(im_f,mCurrentFrame.mvKeys[i].pt,2,cv::Scalar(255,0,0),-1);
    for(int i=0; i<vpCandidateKFs[keyframe_idx_with_most_inliers]->mvKeysUn.size(); i++)
        cv::circle(im_kf,vpCandidateKFs[keyframe_idx_with_most_inliers]->mvKeysUn[i].pt,2,cv::Scalar(255,0,0),-1);
    cv::hconcat(im_f, im_kf, img_h);
    //for(int i=0; i<mCurrentFrame.mvpMapPoints.size(); i++){
    for(int i=0; i<vvpMapPointMatches[keyframe_idx_with_most_inliers].size(); i++){
        auto mp = mCurrentFrame.mvpMapPoints[i];
        if(!mp)
            continue;
        cv::Mat p3d = mp->GetWorldPos();
        cv::Mat p3Dc = mCurrentFrame.mTcw(cv::Rect(0, 0, 3, 3))*p3d + mCurrentFrame.mTcw(cv::Rect(3, 0, 1, 3));
        // Project into Image
        float invz = 1/p3Dc.at<float>(2);
        float x = p3Dc.at<float>(0)*invz;;
        float y = p3Dc.at<float>(1)*invz;;
        float u = mCurrentFrame.fx*x+mCurrentFrame.cx;
        float v = mCurrentFrame.fy*y+mCurrentFrame.cy;
        cv::circle(img_h,cv::Point2f(u, v),4,cv::Scalar(0,255,0),-1);

        int ind_mp_in_kf = mp->GetIndexInKeyFrame(vpCandidateKFs[keyframe_idx_with_most_inliers]);
        if(ind_mp_in_kf == -1)
            continue;
        cv::circle(img_h,vpCandidateKFs[keyframe_idx_with_most_inliers]->mvKeysUn[ind_mp_in_kf].pt+cv::Point2f(640,0),
                    4,cv::Scalar(0,255,0),-1);
        cv::line(img_h, mCurrentFrame.mvKeys[i].pt, vpCandidateKFs[keyframe_idx_with_most_inliers]->mvKeysUn[ind_mp_in_kf].pt+cv::Point2f(640,0),
                    cv::Scalar(0,255,0), 1, cv::LINE_AA);
        
    }
    
    std::string image_path = "/home/yutong/OA-SLAM/bin/points/"+to_string(mCurrentFrame.mnId)+".png";
    cv::imwrite(image_path.c_str(), img_h);*/

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}


double Tracking::get_mean_iou(std::vector<Ellipsoid, Eigen::aligned_allocator<Ellipsoid>> ellipsoids_tmp,
                            std::vector<BBox2, Eigen::aligned_allocator<BBox2>> bboxes_tmp, cv::Mat Rt){
    Eigen::Matrix3d R_old;
    cv::cv2eigen(Rt(cv::Rect(0, 0, 3, 3)), R_old);
    Eigen::Vector3d T_old;
    cv::cv2eigen(Rt(cv::Rect(3, 0, 1, 3)), T_old);
    Matrix34d R_frame;
    R_frame << R_old, T_old;
    Matrix34d P_frame = K_ * R_frame;

    double sum_iou = 0.0;
    //double max_iou = 0.0;
    for (int i = 0; i < ellipsoids_tmp.size(); i++){
        auto ell = ellipsoids_tmp[i];
        auto bbox = bboxes_tmp[i];
        auto proj_bbox = ell.project(P_frame).ComputeBbox();
        double iou = bboxes_iou(bbox, proj_bbox);
        sum_iou += iou;
    }
    return sum_iou/ellipsoids_tmp.size();
}

bool Tracking::check_ious(std::vector<Ellipsoid, Eigen::aligned_allocator<Ellipsoid>>& ellipsoids,
                         std::vector<BBox2, Eigen::aligned_allocator<BBox2>>& bboxes, 
                         cv::Mat Rt, double thres, double &mean_iou){

    Eigen::Matrix3d R_old;
    cv::cv2eigen(Rt(cv::Rect(0, 0, 3, 3)), R_old);
    Eigen::Vector3d T_old;
    cv::cv2eigen(Rt(cv::Rect(3, 0, 1, 3)), T_old);
    Matrix34d R_frame;
    R_frame << R_old, T_old;
    Matrix34d P_frame = K_ * R_frame; 

    double sum_iou = 0.0;
    std::vector<int> inliers = std::vector<int>();

    for (int i = 0; i < ellipsoids.size(); i++){
        auto ell = ellipsoids[i];
        auto bbox = bboxes[i];
        auto proj_bbox = ell.project(P_frame).ComputeBbox();
        double iou = bboxes_iou(bbox, proj_bbox);
        double z = (ell.GetCenter() - T_old).dot(R_old.col(2)); // Check that the camera is in front of the scene too
        if(iou>0.1f && z>0){
            inliers.push_back(i);
            sum_iou += iou;
        }
    }

    //double mean_iou = 0.0;
    if (inliers.empty()){
        mean_iou = 0.0;
        ellipsoids.clear();
        bboxes.clear();
    }
    else{
        mean_iou = sum_iou / inliers.size();
        std::vector<Ellipsoid, Eigen::aligned_allocator<Ellipsoid>> ellipsoids_tmp;
        std::vector<BBox2, Eigen::aligned_allocator<BBox2>> bboxes_tmp;
        for(auto i: inliers){
            ellipsoids_tmp.push_back(ellipsoids[i]);
            bboxes_tmp.push_back(bboxes[i]);
        }
        ellipsoids.clear();
        bboxes.clear();
        ellipsoids = ellipsoids_tmp;
        bboxes = bboxes_tmp;
    }
    if (mean_iou < thres || bboxes.size()<4)
       return false;
    return true;
}



void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();


    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
