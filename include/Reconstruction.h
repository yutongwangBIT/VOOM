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


#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include "Ellipsoid.h"
#include "Utils.h"

namespace ORB_SLAM2
{



std::pair<bool, Ellipsoid>
ReconstructEllipsoidFromCenters(const std::vector<BBox2, Eigen::aligned_allocator<BBox2>>& boxes,
                                const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& Rts, 
                                const Eigen::Matrix3d& K);

Eigen::Vector3d TriangulatePoints2(const Eigen::Vector2d& uv1, const Eigen::Vector2d& uv2,
                                  const Matrix34d& P1, const Matrix34d& P2);

Eigen::Vector3d TriangulatePoints(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points,
                                  const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& projections);

Eigen::Vector3d TriangulatePointsRansac(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points,
                                        const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& projections,
                                        int max_iter);

std::pair<bool, Ellipsoid>
ReconstructEllipsoidFromLandmarks(const std::vector<BBox2, Eigen::aligned_allocator<BBox2>>& bboxes,
                                  const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& Rts, 
                                  const Eigen::Matrix3d& K, const Eigen::Matrix<double, 3, Eigen::Dynamic>& pts);

Ellipsoid reconstruct_ellipsoid_lstsq(const std::vector<Ellipse, Eigen::aligned_allocator<Ellipse>>& ellipses,
                                      const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& projections);

std::pair<bool, Ellipsoid>
ReconstructEllipsoidCrocco(const std::vector<BBox2, Eigen::aligned_allocator<BBox2>>& bboxes,
                           const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& Rts, 
                           const Eigen::Matrix3d& K, bool use_two_passes);

}

#endif // RECONSTRUCTION_H