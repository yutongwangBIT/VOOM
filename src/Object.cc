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


#include "Object.h"
#include "ColorManager.h"
#include "OptimizerObject.h"


namespace ORB_SLAM2 
{
    unsigned int Object::factory_id = 0;

    Object::Object(unsigned int cat, const BBox2& bb, const Ellipse ell, double score, std::pair<float, float> depth_data, Eigen::Matrix3d K,
            const Matrix34d& Rt, long unsigned int frame_idx, KeyFrame *kf){
        id_ = factory_id++;
        category_id_ = cat;
        N_ += 1;
        K_ = K;
        last_obs_score_ = score;
        last_obs_frame_id_ = frame_idx;
        flag_optimized = false;
        mbBad = false;

        bboxes_.push_back(bb);
        Rts_.push_back(Rt);

        color_ = RandomUniformColorGenerator::Generate();

        //RECONSTRUCT WITH DEPTH
        float avg_depth = depth_data.first;
        float diff_depth = depth_data.second;

        Eigen::Vector2d bb_center = bbox_center(bb);
        double u = (bb_center(0) - K_(0, 2)) / K_(0, 0);
        double v = (bb_center(1) - K_(1, 2)) / K_(1, 1);
        Eigen::Vector3d bb_center_cam(u*avg_depth, v*avg_depth, avg_depth);
        auto Rcw = Rt.block<3,3>(0,0);
        auto tcw = Rt.col(3);
        Eigen::Vector3d center_world = Rcw.transpose() * bb_center_cam + (-Rcw.transpose() * tcw);
        //ROTATION
        Eigen::Vector3d zc = bb_center_cam / bb_center_cam.norm();
        Eigen::Vector3d up_vec{0, -1, 0};
        Eigen::Vector3d xc = (-up_vec).cross(zc);
        xc = xc / xc.norm();
        Eigen::Vector3d yc = zc.cross(xc);
        Eigen::Matrix3d rot_cam;
        rot_cam.col(0) = xc;
        rot_cam.col(1) = yc;
        rot_cam.col(2) = zc;
        Eigen::Matrix3d rot_world = Rcw.transpose() * rot_cam;
        //AXES
        double width_in_img = bb[2] - bb[0];
        double height_in_img = bb[3] - bb[1];
        double width_in_world = (width_in_img/K_(0, 0))*avg_depth;
        double height_in_world = (height_in_img/K_(1, 1))*avg_depth;
        Eigen::Vector3d axes(0.5*width_in_world, 0.5*height_in_world, 0.5*diff_depth);
        ellipsoid_ = Ellipsoid(axes, rot_world, center_world);
        if(kf){
            mnFirstKFid = kf->mnId;
            mnLastKFid = kf->mnId;
            ellipses_.push_back(ell);
            observed_kfs.push_back(kf);
            std::unique_lock<std::mutex> lock(mutex_associated_map_points_);
            auto vIndices_in_box = kf->GetFeaturesInBox(bb[0], bb[2], bb[1], bb[3]);
            for(auto i : vIndices_in_box){
                MapPoint* mp = kf->mvpMapPoints[i];
                if (mp) {
                    associated_map_points_.insert(mp);
                }
            }
        }
    }

    void Object::AddDetection(unsigned int cat, const BBox2& bbox, const Ellipse ell, double score, const Matrix34d& Rt, unsigned int frame_idx, KeyFrame* kf){
        //todo more cat id...
        unique_lock<mutex> lock(mutex_add_detection_);
        last_obs_frame_id_ = frame_idx;
        last_obs_score_ = score;
        N_ += 1;
        bboxes_.push_back(bbox);
        Rts_.push_back(Rt);

        if(kf){
            mnLastKFid = kf->mnId;
            ellipses_.push_back(ell);
            observed_kfs.push_back(kf);
            std::unique_lock<std::mutex> lock(mutex_associated_map_points_);
            auto vIndices_in_box = kf->GetFeaturesInBox(bbox[0], bbox[2], bbox[1], bbox[3]);
            for(auto i : vIndices_in_box){
                MapPoint* mp = kf->mvpMapPoints[i];
                if (mp) {
                    associated_map_points_.insert(mp);
                }
            }
            if(observed_kfs.size() > 2 && observed_kfs.size() % 1 == 0 && observed_kfs.size()<30){
                //OptimizeReconstruction(false);
                OptimizeReconstructionQuat(true);
                flag_optimized = true;
            }
        }

        //if(!flag_optimized && bboxes_.size()>4){
        //    OptimizeReconstruction(false);
        //}
        
    }

    void Object::OptimizeReconstruction(bool b_random_detections)
    {
        const Ellipsoid& ellipsoid = this->GetEllipsoid();

        // std::cout << "===============================> Start ellipsoid optimization " << id_ << std::endl;
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> BlockSolver_6_1;
        BlockSolver_6_1::LinearSolverType *linear_solver = new g2o::LinearSolverDense<BlockSolver_6_1::PoseMatrixType>();


        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            new BlockSolver_6_1(linear_solver)
        );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);


        VertexEllipsoidNoRot* vertex = new VertexEllipsoidNoRot();
        // VertexEllipsoid* vertex = new VertexEllipsoid();
        vertex->setId(0);
        Eigen::Matrix<double, 6, 1> e;
        e << ellipsoid.GetAxes(), ellipsoid.GetCenter();
        vertex->setEstimate(e);
        optimizer.addVertex(vertex);

    
        std::vector<size_t> chosen_indexes; //We only choose part of the detections for optimization
        size_t N = bboxes_.size();
        size_t min_opt_N = 20;

        for(size_t i=0; i<N; i++){
            chosen_indexes.push_back(i);
        }

        if(b_random_detections && N > min_opt_N){
            random_shuffle(chosen_indexes.begin(), chosen_indexes.end());
            std::vector<size_t> tmp;
            for(size_t i=0; i<min_opt_N; i++){
                tmp.push_back(chosen_indexes[i]);
            }
            chosen_indexes = tmp;
        }

        auto it_bb = bboxes_.begin();
        auto it_Rt = Rts_.begin();
        
        for (auto i : chosen_indexes){//size_t i = 0; i < bboxes_.size() && it_bb != bboxes_.end() && it_Rt != Rts_.end(); ++i, ++it_bb, ++it_Rt) {
            it_bb = bboxes_.begin() + i;
            it_Rt = Rts_.begin() + i;
            Eigen::Matrix<double, 3, 4> P = K_ * (*it_Rt);
            //auto kf = observed_kfs[i];
            //Matrix34d Rt = cvToEigenMatrix<double, float, 3, 4>(kf->GetPose());
            //Eigen::Matrix<double, 3, 4> P = K_ * Rt;
            EdgeEllipsoidProjection *edge = new EdgeEllipsoidProjection(P, Ellipse::FromBbox(*it_bb), ellipsoid.GetOrientation());
            edge->setId(i);
            edge->setVertex(0, vertex);
            Eigen::Matrix<double, 1, 1> information_matrix = Eigen::Matrix<double, 1, 1>::Identity();
            edge->setInformation(information_matrix);
            optimizer.addEdge(edge);
        }
        
        optimizer.initializeOptimization();
        optimizer.optimize(8);
        Eigen::Matrix<double, 6, 1> ellipsoid_est = vertex->estimate();

        Ellipsoid new_ellipsoid(ellipsoid_est.head(3), ellipsoid.GetOrientation(), ellipsoid_est.tail(3));
        this->SetEllipsoid(new_ellipsoid);
    }

    void Object::OptimizeReconstructionQuat(bool b_random_detections)
    {
        const Ellipsoid& ellipsoid = this->GetEllipsoid();

        //std::cout << "===============================> Start ellipsoid optimization quat " << id_ << std::endl;
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 1>> BlockSolver;
        BlockSolver::LinearSolverType *linear_solver = new g2o::LinearSolverDense<BlockSolver::PoseMatrixType>();

        // std::cout << "Optim obj " << obj->GetTrack()->GetId()<< "\n";
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            new BlockSolver(linear_solver)
        );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);


        VertexEllipsoidQuat* vertex = new VertexEllipsoidQuat();
        vertex->setId(0);
        EllipsoidQuat ellipsoid_quat = EllipsoidQuat::FromEllipsoid(ellipsoid);
        vertex->setEstimate(ellipsoid_quat);
        optimizer.addVertex(vertex);

        //We only choose part of the detections for optimization
        std::vector<size_t> chosen_indexes; 
        size_t N = observed_kfs.size();
        size_t min_opt_N = 10;

        for(size_t i=0; i<N; i++){
            chosen_indexes.push_back(i);
        }

        if(b_random_detections && N > min_opt_N){
            random_shuffle(chosen_indexes.begin(), chosen_indexes.end());
            std::vector<size_t> tmp;
            for(size_t i=0; i<min_opt_N; i++){
                tmp.push_back(chosen_indexes[i]);
            }
            chosen_indexes = tmp;
        }

        //auto it_bb = bboxes_.begin();
        auto it_ell = ellipses_.begin();
        //auto it_Rt = Rts_.begin();

        for (auto i : chosen_indexes){
            //it_bb = bboxes_.begin() + i;
            it_ell = ellipses_.begin() + i;
            //it_Rt = Rts_.begin() + i;
            auto kf = observed_kfs[i];
            //Eigen::Matrix<double, 3, 4> P = K_ * (*it_Rt);
            Matrix34d Rt = cvToEigenMatrix<double, float, 3, 4>(kf->GetPose());
            Eigen::Matrix<double, 3, 4> P = K_ * Rt;
            EdgeEllipsoidProjectionQuat *edge = new EdgeEllipsoidProjectionQuat(P, *it_ell, ellipsoid.GetOrientation());
            edge->setId(i);
            edge->setVertex(0, vertex);
            Eigen::Matrix<double, 1, 1> information_matrix = Eigen::Matrix<double, 1, 1>::Identity();
            edge->setInformation(information_matrix);
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            optimizer.addEdge(edge);
        }
        
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        EllipsoidQuat ellipsoid_quat_est = vertex->estimate();
        Ellipsoid new_ellipsoid = ellipsoid_quat_est.ToEllipsoid();
        SetEllipsoid(new_ellipsoid);
    }


    std::vector<MapPoint*> Object::GetFilteredAssociatedMapPoints(int threshold){
        std::unique_lock<std::mutex> lock(mutex_associated_map_points_);
        int limit = threshold; //  std::max(10.0, threshold * keyframes_bboxes_.size());

        std::vector<MapPoint*> filtered;

        if (true) {
            for (auto mp : associated_map_points_) {
                if(!mp) continue;
                if(mp->isBad()) continue;
                cv::Mat p = mp->GetWorldPos();
                Eigen::Vector3d pos(p.at<float>(0), p.at<float>(1), p.at<float>(2));
                if (ellipsoid_.IsInside(pos, 1.0))
                    filtered.push_back(mp);
            }
        }

        return filtered;
    }


}
