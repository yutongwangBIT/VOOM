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


#ifndef OBJECT_H
#define OBJECT_H

#include "Utils.h"

#include <random>
#include <memory>
#include <list>
#include <iostream>

#include <algorithm>

#include <Eigen/Dense>


#include "Ellipse.h"
#include "Ellipsoid.h"
#include "Map.h"

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"



namespace ORB_SLAM2
{
    
class Object
{
    public:

        static unsigned int factory_id;

        Object(unsigned int cat, const BBox2& bbox, const Ellipse ell, double score, std::pair<float, float> depth_data, Eigen::Matrix3d K, 
              const Matrix34d& Rt, long unsigned int frame_idx, KeyFrame *kf);

        Object(const Ellipsoid& ellipsoid) : ellipsoid_(ellipsoid){
            id_ = 0;
        }

        void AddDetection(unsigned int cat, const BBox2& bbox, const Ellipse ell, double score, const Matrix34d& Rt, unsigned int frame_idx, KeyFrame* kf);

        const Ellipsoid& GetEllipsoid() const {
            std::unique_lock<std::mutex> lock(mutex_ellipsoid_);
            return ellipsoid_;
        }

        void SetEllipsoid(const Ellipsoid& ell) {
            std::unique_lock<std::mutex> lock(mutex_ellipsoid_);
            ellipsoid_ = ell;
        }

        unsigned int GetId() const {
            return id_;
        }

        void SetId(unsigned int id) {
            //std::cout<<"mapobj:"<<id_<<"set id to:"<<id<<std::endl;
            id_ = id;
        }

        size_t GetNbObservations() const {
            return N_;
        }

        bool operator<(const Object* rhs) const{
            return this->id_ < rhs->GetId();
        }

        cv::Scalar GetColor() const {
            return color_;
        }

        unsigned int GetCategoryId() const {
            return category_id_;
        }

        size_t GetLastObsFrameId() const {
            return last_obs_frame_id_;
        }

        bool GetFlagOptimized() const{
            return flag_optimized;
        }

        std::set<MapPoint*> GetAssociatedMapPoints() const {
            std::unique_lock<std::mutex> lock(mutex_associated_map_points_);
            return associated_map_points_;
        }

        std::vector<MapPoint*> GetFilteredAssociatedMapPoints(int threshold);

        void InsertNewAsscoatedMapPoint(MapPoint* mp){
            std::unique_lock<std::mutex> lock(mutex_associated_map_points_);
            associated_map_points_.insert(mp);
        }

        void OptimizeReconstruction(bool b_random_detections);

        void OptimizeReconstructionQuat(bool b_random_detections);

        int GetObservationNumber(){
            return observed_kfs.size();
        }

        std::vector<KeyFrame*> GetObservations(){
            unique_lock<mutex> lock(mutex_add_detection_);
            return observed_kfs;
        }

        void SetBadFlag(){
            mbBad = true;
        }

        bool isBad(){
            return mbBad;
        }

        

    public:
        long unsigned int last_obs_frame_id_ = -1;
        std::pair<std::pair<long unsigned int, int>, double> last_obs_ids_and_max_iou; // for multiple match

        long unsigned int  mnFirstKFid = -1;
        long unsigned int  mnLastKFid = -1;

    protected:
        unsigned int category_id_;
        unsigned int id_;
        double last_obs_score_ = 0.0;
        cv::Scalar color_;
        Eigen::Matrix3d K_;
        bool flag_optimized;
        bool mbBad;
        size_t N_ = 0;

        std::vector<BBox2, Eigen::aligned_allocator<BBox2>> bboxes_ = std::vector<BBox2, Eigen::aligned_allocator<BBox2>>();
        std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>> Rts_ = std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>();

        std::vector<Ellipse, Eigen::aligned_allocator<Ellipse>> ellipses_ = std::vector<Ellipse, Eigen::aligned_allocator<Ellipse>>();

        std::vector<KeyFrame*> observed_kfs;

        Ellipsoid ellipsoid_;

        std::set<MapPoint*> associated_map_points_;

        mutable std::mutex mutex_ellipsoid_;
        mutable std::mutex mutex_associated_map_points_;
        mutable std::mutex mutex_add_detection_;

        Object() = delete;
};


}

#endif //  OBJECT_H
