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

#include "ObjectMatcher.h"

#include<limits.h>
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include<opencv2/core/eigen.hpp>
#include "Optimizer.h"
#include "Distance.h"

using namespace std;

namespace ORB_SLAM2
{

const int ObjectMatcher::TH_HIGH = 100;
const int ObjectMatcher::TH_LOW = 50;
const int ObjectMatcher::HISTO_LENGTH = 30;

ObjectMatcher::ObjectMatcher(Eigen::Matrix3d K)
{
    K_ = K;
}

int ObjectMatcher::MatchObjectsHungarian(Frame &CurrentFrame, std::unordered_map<Object*, Ellipse> proj_bboxes){
    int nmatches=0;

    std::vector<Object*> objects;
    for(auto it : proj_bboxes){
        objects.push_back(it.first);
    }

    int m = std::max(objects.size(), CurrentFrame.graph->nodes.size());
    dlib::matrix<long> cost = dlib::zeros_matrix<long>(m, m);
    std::vector<long> assignment(m, std::numeric_limits<long>::max());

    for(auto [node_id, attribute] : CurrentFrame.graph->attributes){
        for (size_t obj_idx = 0; obj_idx < objects.size(); ++obj_idx) {
            auto obj = objects[obj_idx];
            if (obj->GetCategoryId() == attribute.label) {
                double iou_3d = 0;
                iou_3d = bboxes_iou(proj_bboxes[obj].ComputeBbox(), attribute.bbox);

                if (iou_3d < 0.3) iou_3d = 0;

                //std::cout << "2D: " << iou_2d << "\n";
                //std::cout << "3D: " << iou_3d << "\n";
                cost(node_id, obj_idx) = iou_3d * 1000;
            }
        }
    }
    assignment = dlib::max_cost_assignment(cost);


    for(auto [node_id, attribute] : CurrentFrame.graph->attributes){
        auto assigned_obj_idx = assignment[node_id];
        if (assigned_obj_idx < static_cast<long>(objects.size()) && cost(node_id, assigned_obj_idx) > 0){
            nmatches += 1;
            CurrentFrame.graph->attributes[node_id].obj = objects[assigned_obj_idx];
            CurrentFrame.graph->attributes[node_id].obj->last_obs_ids_and_max_iou.first = std::make_pair(CurrentFrame.mnId, node_id);
            //CurrentFrame.graph->attributes[node_id].obj->last_obs_ids_and_max_iou.second = cost(node_id, assigned_obj_idx);
        }
    }

    return nmatches;
}

int ObjectMatcher::MatchObjectsIoU(Frame &CurrentFrame, std::unordered_map<Object*, Ellipse> proj_bboxes)
{
    int nmatches=0;

    for(auto [node_id, attribute] : CurrentFrame.graph->attributes){
        double iou_max = 0;
        Object* matched_obj = nullptr;
        auto bb_det = attribute.bbox;
        for(auto it : proj_bboxes){
            auto proj = it.second;
            double threshold = 0.3;
            if(it.first->GetCategoryId() == attribute.label) threshold = 0.1;
            double iou = bboxes_iou(proj.ComputeBbox(), bb_det);
            
            if(iou>threshold && iou > iou_max){
                iou_max = iou;
                matched_obj = it.first;
            }
        }
        
        if(iou_max > 0.005) {
            if(matched_obj->last_obs_ids_and_max_iou.first.first == CurrentFrame.mnId){
                //compare and select the best
                double iou_last = matched_obj->last_obs_ids_and_max_iou.second;
                if(iou_max < iou_last){
                    continue;
                }
                else{
                    CurrentFrame.graph->attributes[matched_obj->last_obs_ids_and_max_iou.first.second].obj = nullptr;
                }
            }
            
            nmatches += 1;
            CurrentFrame.graph->attributes[node_id].obj = matched_obj;
            CurrentFrame.graph->attributes[node_id].obj->last_obs_ids_and_max_iou.first = std::make_pair(CurrentFrame.mnId, node_id);
            CurrentFrame.graph->attributes[node_id].obj->last_obs_ids_and_max_iou.second = iou_max;
        }
    }
    
    return nmatches;
}

int ObjectMatcher::MatchObjectsWasserDistance(Frame &CurrentFrame, std::unordered_map<Object*, Ellipse> proj_bboxes)
{
    int nmatches=0;

    for(auto [node_id, attribute] : CurrentFrame.graph->attributes){
        double dis_max = 0;
        Object* matched_obj = nullptr;
        auto bb_det = attribute.bbox;
        for(auto it : proj_bboxes){
            auto proj = it.second;
            double wasser_dis = normalized_gaussian_wasserstein_2d(proj, attribute.ell, 10);
            double iou = bboxes_iou(proj.ComputeBbox(), bb_det);
            if(wasser_dis>dis_max && iou>0.01){
                dis_max = wasser_dis;
                matched_obj = it.first;
            }
        }
        
        if(dis_max > 0.00001) {
            if(matched_obj->last_obs_ids_and_max_iou.first.first == CurrentFrame.mnId){
                //compare and select the best
                double iou_last = matched_obj->last_obs_ids_and_max_iou.second;
                if(dis_max < iou_last){
                    continue;
                }
                else{
                    CurrentFrame.graph->attributes[matched_obj->last_obs_ids_and_max_iou.first.second].obj = nullptr;
                    nmatches -= 1;
                }
            }
            
            nmatches += 1;
            CurrentFrame.graph->attributes[node_id].obj = matched_obj;
            CurrentFrame.graph->attributes[node_id].obj->last_obs_ids_and_max_iou.first = std::make_pair(CurrentFrame.mnId, node_id);
            CurrentFrame.graph->attributes[node_id].obj->last_obs_ids_and_max_iou.second = dis_max;
        }
    }
    
    return nmatches;
}

float ObjectMatcher::AssociateFeaturesWithObjects(Frame &CurrentFrame){
    float sum_intersection_ratio = 0.0f;
    float count_useful_objects = 0.0f;
    //CurrentFrame.mvpMapPoints = std::vector<MapPoint*>(CurrentFrame.N,static_cast<MapPoint*>(NULL));   
    int count_all_new_mps = 0;
    for(auto [node_id, attribute] : CurrentFrame.graph->attributes){
        auto matched_obj = attribute.obj;
        if(!matched_obj) continue;
        auto bb_det = attribute.bbox;
        auto vIndices_in_box = CurrentFrame.GetFeaturesInBox(bb_det[0], bb_det[2], bb_det[1], bb_det[3]);
        std::set<MapPoint*> asscociated_mps = matched_obj->GetAssociatedMapPoints();
        if(vIndices_in_box.size() < 3 || asscociated_mps.size() < 3) continue;
        std::set<MapPoint*> set_mp_in_box;
        int original_size_features = vIndices_in_box.size();
        auto iter = vIndices_in_box.begin();
        while(iter != vIndices_in_box.end()){
            MapPoint* mp = CurrentFrame.mvpMapPoints[*iter];
            if(!mp) ++iter;
            else{
                set_mp_in_box.insert(mp);
                iter = vIndices_in_box.erase(iter);
            }
        }

        std::vector<MapPoint*> non_intersected_mps = std::vector<MapPoint*>();
        int count_intersected_mps = 0;
        for(auto mp : matched_obj->GetAssociatedMapPoints()){
            if(set_mp_in_box.count(mp)==0){
                non_intersected_mps.push_back(mp);
                continue;
            }
            else{
                count_intersected_mps += 1;
            }
        }
        

        //if(intersection_ratio < 0.75f){//ADD MORE ACCOCIATED MAPPOINTS INTO.
        for(auto ind : vIndices_in_box){
            const cv::Mat &dF = CurrentFrame.mDescriptors.row(ind);
            int bestDist = 256;
            int bestIdx = -1;
            for(size_t i=0; i<non_intersected_mps.size(); i++){
                auto pMP = non_intersected_mps[i];
                const cv::Mat dMP = pMP->GetDescriptor();
                const int dist = DescriptorDistance(dMP,dF);
                
                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = i;
                }
            }
            if(bestDist<=50)
            {
                //map_obj->InsertNewAsscoatedMapPoint(vpPoints[bestIdx]);
                CurrentFrame.mvpMapPoints[ind] = non_intersected_mps[bestIdx];
                non_intersected_mps.erase(non_intersected_mps.begin()+bestIdx);
                //nmatches_points++;
                count_intersected_mps += 1;
                count_all_new_mps += 1;
            }
        }
        //}
        float intersection_ratio = float(count_intersected_mps)/float(original_size_features);
        count_useful_objects += 1.0f;
        sum_intersection_ratio += intersection_ratio;
        CurrentFrame.graph->attributes[node_id].hue = intersection_ratio; //TOBE DELETED
    }

    
    //if(count_all_new_mps<20) return 0.0f;

    // Optimize frame pose with all matches
    /*Optimizer::PoseOptimization(&CurrentFrame);

    // Discard outliers
    for(int i =0; i<CurrentFrame.N; i++)
    {
        if(CurrentFrame.mvpMapPoints[i])
        {
            if(CurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = CurrentFrame.mvpMapPoints[i];

                CurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                CurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = CurrentFrame.mnId;
            }
        }
    } */

    return sum_intersection_ratio/(count_useful_objects+0.001f);
}


int ObjectMatcher::MatchObjectsByProjection(Frame &CurrentFrame, std::unordered_map<Object*, Ellipse> proj_bboxes)
{
    int nmatches=0;

    int total_pts = 0;
    int total_inter_mps = 0;
    std::vector<std::tuple<Object*, std::vector<MapPoint*>, std::vector<size_t>>> vt_non_asccociated_mps;
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    std::vector<bool> b_marked_kpts = std::vector<bool>(CurrentFrame.N, false); 
    for(auto [node_id, attribute] : CurrentFrame.graph->attributes){
        double iou_max = 0;
        Object* matched_obj = nullptr;
        auto bb_det = attribute.bbox;
        for(auto it : proj_bboxes){
            auto proj = it.second;
            double threshold = 0.3;
            //if(it.first->GetCategoryId() == attribute.label) threshold = 0.1;
            double iou = bboxes_iou(proj.ComputeBbox(), bb_det);
            //auto center_distance = bboxes_distance(bb_proj, bb_det);
            //auto thres_c_dis = 0.5*sqrt(bbox_area(bb_det));
            //double wasser_dis = normalized_gaussian_wasserstein_2d(proj, Ellipse::FromBbox(bb_det), 10);
            //double wasser_dis = normalized_gaussian_wasserstein_2d(proj, attribute.ell, 10);
            
            if(iou>threshold && iou > iou_max){
                iou_max = iou;
                matched_obj = it.first;
            }
            //if(wasser_dis>iou_max){// && iou > 0.01){
            //    iou_max = wasser_dis;
            //    matched_obj = it.first;
            //}

        }
        //std::cout<<"iou_max:"<<iou_max<<std::endl;
        if(iou_max > 0.005) {
            Eigen::Vector2d center_2d = bbox_center(bb_det);
            Eigen::Vector3d center_obj =  matched_obj->GetEllipsoid().GetCenter();
            imagePoints.push_back(cv::Point2f(center_2d(0), center_2d(1)));
            objectPoints.push_back(cv::Point3f(center_obj(0), center_obj(1), center_obj(2)));
            std::tuple<Object*, std::vector<MapPoint*>, std::vector<size_t>> t_non_asccociated_mps;
            std::get<0>(t_non_asccociated_mps) = matched_obj;
            std::get<1>(t_non_asccociated_mps) = std::vector<MapPoint*>();
            std::get<2>(t_non_asccociated_mps) = std::vector<size_t>();
            if(matched_obj->last_obs_ids_and_max_iou.first.first == CurrentFrame.mnId){
                //compare and select the best
                double iou_last = matched_obj->last_obs_ids_and_max_iou.second;
                if(iou_max < iou_last){
                    continue;
                }
                else{
                    CurrentFrame.graph->attributes[matched_obj->last_obs_ids_and_max_iou.first.second].obj = nullptr;
                }
            }
            
            int count_mp_associated = 0;
            std::set<MapPoint*> set_mp_in_box;
            auto vIndices_in_box = CurrentFrame.GetFeaturesInBox(bb_det[0], bb_det[2], bb_det[1], bb_det[3]);
            std::get<2>(t_non_asccociated_mps) = vIndices_in_box;
            for(auto i : vIndices_in_box){
                b_marked_kpts[i] = true;
                MapPoint* mp = CurrentFrame.mvpMapPoints[i];
                if (!mp) {
                    //std::get<2>(t_non_asccociated_mps).push_back(i);
                    continue;
                }
                else{
                    set_mp_in_box.insert(mp);
                }
            }
            int mp_inter_size = 0;
            for(auto mp : matched_obj->GetAssociatedMapPoints()){
                std::get<1>(t_non_asccociated_mps).push_back(mp);
                if(set_mp_in_box.count(mp)==0){
                    //std::get<1>(t_non_asccociated_mps).push_back(mp);
                    continue;
                }
                else{
                    mp_inter_size += 1;
                }
            }
            
            total_pts += vIndices_in_box.size();
            total_inter_mps += mp_inter_size;
            //std::cout<<"obj mps:"<<matched_obj->GetAssociatedMapPoints().size()<<",count_in_box_pts:"<<vIndices_in_box.size()<<",set_mp_in_box:"<<set_mp_in_box.size()<<",mp intersection:"<<mp_inter_size<<std::endl;

            //if(float(mp_inter_size)/(float(vIndices_in_box.size())+0.01f) > 0.3){
            //    for(auto mp:set_mp_in_box){
            //        matched_obj->InsertNewAsscoatedMapPoint(mp);
            //    }
            //}

            nmatches += 1;
            CurrentFrame.graph->attributes[node_id].obj = matched_obj;
            //CurrentFrame.graph->attributes[node_id].obj->last_obs_frame_id_ = CurrentFrame.mnId;
            CurrentFrame.graph->attributes[node_id].obj->last_obs_ids_and_max_iou.first = std::make_pair(CurrentFrame.mnId, node_id);
            CurrentFrame.graph->attributes[node_id].obj->last_obs_ids_and_max_iou.second = iou_max;
            
            vt_non_asccociated_mps.push_back(t_non_asccociated_mps);
        }
    }
    float average_intersection_ratio = float(total_inter_mps)/(float(total_pts)+0.01f);
    std::cout<<"average intersection ratio:"<<average_intersection_ratio<<std::endl;

    /*if(average_intersection_ratio<0.5 && nmatches>5){
        cv::Mat distCoeffs;// = cv::Mat::zeros(4,1,CV_32F);
        cv::Mat r, rvec, tvec, tmp_r;
        std::vector<int> inliers;
        cv::Mat cv_K_;
        cv::eigen2cv(K_, cv_K_);
        cv::Mat Rt_cv = cv::Mat::eye(4, 4, CV_32F);
        
        double mean_iou = 0.0;
        bool success = cv::solvePnPRansac(objectPoints, imagePoints, cv_K_, distCoeffs, rvec, tvec, 
                                        false, 50, 40.0f, 0.9, inliers);
        if(success){
            cv::Rodrigues(rvec, r);
            r.copyTo(Rt_cv(cv::Rect(0, 0, 3, 3)));
            tvec.copyTo(Rt_cv(cv::Rect(3, 0, 1, 3)));
            //std::cout<<"mTcw:"<<mTcw<<", new Rt:"<<Rt_cv<<std::endl;
            Eigen::Matrix3d R_tmp;
            Eigen::Vector3d T_tmp;
            cv::cv2eigen(r, R_tmp);
            cv::cv2eigen(tvec, T_tmp);
            CurrentFrame.SetPose(Rt_cv); 
        }
        CurrentFrame.mvpMapPoints = std::vector<MapPoint*>(CurrentFrame.N,static_cast<MapPoint*>(NULL));   
        for(size_t i=0; i<vt_non_asccociated_mps.size();i++){
            if(std::find(inliers.begin(), inliers.end(), i) == inliers.end())
                continue; //TODO delete obj
            //Object* map_obj = std::get<0>(t_non_asccociated_mps);
            auto t_non_asccociated_mps = vt_non_asccociated_mps[i];
            std::vector<MapPoint*> vpPoints = std::get<1>(t_non_asccociated_mps);
            std::vector<size_t> indices = std::get<2>(t_non_asccociated_mps);
            std::cout<<"candidate indices size:"<<indices.size()<<", vppoints size:"<<vpPoints.size()<<std::endl;
            if(indices.empty()) continue;
            int nmatches_points = SearchByProjection(CurrentFrame, vpPoints, indices, 2);
            std::cout<<"newly added mappoints size:"<<nmatches_points<<std::endl;
        }

        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&CurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for(int i =0; i<CurrentFrame.N; i++)
        {
            if(CurrentFrame.mvpMapPoints[i])
            {
                if(CurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = CurrentFrame.mvpMapPoints[i];

                    CurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    CurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = CurrentFrame.mnId;
                    //nmatches--;
                }
                else if(CurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        } 
        std::cout<<"nmatchesMap:"<<nmatchesMap<<std::endl;
        CurrentFrame.mvpMapPoints = std::vector<MapPoint*>(CurrentFrame.N,static_cast<MapPoint*>(NULL));
        for(auto t_non_asccociated_mps : vt_non_asccociated_mps){
            int nmatches_points = 0;
            Object* map_obj = std::get<0>(t_non_asccociated_mps);
            std::vector<MapPoint*> vpPoints = std::get<1>(t_non_asccociated_mps);
            std::vector<size_t> indices = std::get<2>(t_non_asccociated_mps);
            //std::cout<<"candidate indices size:"<<indices.size()<<", vppoints size:"<<vpPoints.size()<<std::endl;
            for(auto ind : indices){
                const cv::Mat &dF = CurrentFrame.mDescriptors.row(ind);
                int bestDist = 256;
                int bestIdx = -1;
                for(size_t i=0; i<vpPoints.size(); i++){
                    auto pMP = vpPoints[i];
                    const cv::Mat dMP = pMP->GetDescriptor();
                    const int dist = DescriptorDistance(dMP,dF);
                    
                    if(dist<bestDist)
                    {
                        bestDist = dist;
                        bestIdx = i;
                    }
                }
                if(bestDist<=50)
                {
                    //map_obj->InsertNewAsscoatedMapPoint(vpPoints[bestIdx]);
                    CurrentFrame.mvpMapPoints[ind] = vpPoints[bestIdx];
                    vpPoints.erase(vpPoints.begin()+bestIdx);
                    nmatches_points++;
                    
                }
            }
            //nmatchesMap += nmatches_points;
            //std::cout<<"newly added mappoints size:"<<nmatches_points<<std::endl;
        }
        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&CurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        //std::set<MapPoint*> set_mp_already_found;
        for(int i =0; i<CurrentFrame.N; i++)
        {
            if(CurrentFrame.mvpMapPoints[i])
            {
                if(CurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = CurrentFrame.mvpMapPoints[i];

                    CurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    CurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = CurrentFrame.mnId;
                    //nmatches--; //TODO map_obj erase mp
                }
                else if(CurrentFrame.mvpMapPoints[i]->Observations()>0){
                    nmatchesMap++;
                    //set_mp_already_found.insert(CurrentFrame.mvpMapPoints[i]);
                }
            }
        } 
        std::cout<<"nmatchesMap:"<<nmatchesMap<<std::endl;

        std::vector<MapPoint*> vp_kf_mps = std::vector<MapPoint*>();
        for(int i=0; i<CurrentFrame.mpReferenceKF->N; i++)
        {
            MapPoint* pMP = CurrentFrame.mpReferenceKF->mvpMapPoints[i];
            if(pMP){
                if(!pMP->isBad()){
                    vp_kf_mps.push_back(pMP);
                }
            }
        }
        int macthes2 = SearchByProjection(CurrentFrame, vp_kf_mps, 0.5, b_marked_kpts);
        //std::cout<<"nmatchesMap"<<nmatchesMap<<"macthes2:"<<macthes2<<std::endl;

        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&CurrentFrame);

        // Discard outliers
        nmatchesMap = 0;
        for(int i =0; i<CurrentFrame.N; i++)
        {
            if(CurrentFrame.mvpMapPoints[i])
            {
                if(CurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = CurrentFrame.mvpMapPoints[i];

                    CurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    CurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = CurrentFrame.mnId;
                    //nmatches--;
                }
                else if(CurrentFrame.mvpMapPoints[i]->Observations()>0){
                    nmatchesMap++;
                }
            }
        } 
        std::cout<<"nmatchesMap:"<<nmatchesMap<<std::endl;
    }    */
    

    return nmatches;
}

void ObjectMatcher::SearchInBox(KeyFrame *pKF, const vector<MapPoint*> &vpMapPoints, const vector<size_t> vIndices_in_box)
{
    for(size_t i=0; i<vpMapPoints.size(); i++){
        auto pMP = vpMapPoints[i];
        const cv::Mat dMP = pMP->GetDescriptor();
        int bestDist = 256;
        int bestIdx = -1;
        for(auto ind : vIndices_in_box){
            const cv::Mat &dKF = pKF->mDescriptors.row(ind);
            const int dist = DescriptorDistance(dMP,dKF);
            
            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = ind;
            }
        }
        if(bestDist<=50)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                {
                    if(pMPinKF->Observations()>pMP->Observations())
                        pMP->Replace(pMPinKF);
                    else
                        pMPinKF->Replace(pMP);
                }
            }
            else
            {
                pMP->AddObservation(pKF,bestIdx);
                pKF->AddMapPoint(pMP,bestIdx);
            }
        }
    }
}

int ObjectMatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const vector<size_t> vIndices_in_box, const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;

    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;
        r *= F.mvScaleFactors[nPredictedLevel];

        cv::Mat p3d = pMP->GetWorldPos();
        cv::Mat p3Dc = F.mTcw(cv::Rect(0, 0, 3, 3))*p3d + F.mTcw(cv::Rect(3, 0, 1, 3));
        // Project into Image
        float invz = 1/p3Dc.at<float>(2);
        float x = p3Dc.at<float>(0)*invz;;
        float y = p3Dc.at<float>(1)*invz;;
        float u = F.fx*x+F.cx;
        float v = F.fy*y+F.cy;

        vector<size_t> vIndices;
        vIndices.reserve(vIndices_in_box.size());

        for(auto ind : vIndices_in_box){
            auto kpUn = F.mvKeysUn[ind];
            const float distx = kpUn.pt.x-u;
            const float disty = kpUn.pt.y-v;
            if(fabs(distx)<r && fabs(disty)<r)
                vIndices.push_back(ind);
        }

        if(vIndices.empty()) continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=256;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::Mat &d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist=dist;
                bestIdx=idx;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)
        {
            if(F.mvpMapPoints[bestIdx]==NULL){
                F.mvpMapPoints[bestIdx]=pMP;
                nmatches++;
            }
            else{
                F.mvpMapPoints[bestIdx]=pMP;
            }
            
        }
    }

    return nmatches;
}

int ObjectMatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th, const std::vector<bool> vb_in_boxes)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;

    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;
        r *= F.mvScaleFactors[nPredictedLevel];

        //std::cout<<"r:"<<r<<std::endl;

        cv::Mat p3d = pMP->GetWorldPos();
        cv::Mat p3Dc = F.mTcw(cv::Rect(0, 0, 3, 3))*p3d + F.mTcw(cv::Rect(3, 0, 1, 3));
        // Project into Image
        float invz = 1/p3Dc.at<float>(2);
        float x = p3Dc.at<float>(0)*invz;;
        float y = p3Dc.at<float>(1)*invz;;
        float u = F.fx*x+F.cx;
        float v = F.fy*y+F.cy;

        const vector<size_t> vIndices = F.GetFeaturesInArea(u, v, r, nPredictedLevel-1, nPredictedLevel+1);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=256;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            if(vb_in_boxes[idx]) continue;

            const cv::Mat &d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist=dist;
                bestIdx=idx;
            }
        }
        
        if(bestDist<=TH_LOW)
        {
            if(F.mvpMapPoints[bestIdx]==NULL){
                F.mvpMapPoints[bestIdx]=pMP;
                nmatches++;
            }
            else{
                F.mvpMapPoints[bestIdx]=pMP;
            }
            
        }
    }

    return nmatches;
}

float ObjectMatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}

int ObjectMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

}
