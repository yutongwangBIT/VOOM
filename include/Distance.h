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

#ifndef DISTANCE_H
#define DISTANCE_H

#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "Ellipse.h"
#include "Ellipsoid.h"
#include "Utils.h"


namespace ORB_SLAM2
{

double gaussian_wasserstein_2d(const Ellipse& ell1, const Ellipse& ell2);

double normalized_gaussian_wasserstein_2d(const Ellipse& ell1, const Ellipse& ell2, const double constant_C);

double gaussian_bhattacharrya_2d(const Ellipse& ell1, const Ellipse& ell2);

std::vector<std::vector<double>> generate_sampling_points(const Ellipse& ell, int count_az, int count_dist, double scale);

std::tuple<double, std::vector<double>, std::vector<double>, std::vector<std::vector<double>>>
ellipses_sampling_metric_ell(const Ellipse& ell1, const Ellipse& ell2, int N_az, int N_dist, double sampling_scale);

std::tuple<bool, BBox2> find_on_image_bbox(const Ellipse& ell, int width, int height);

inline double algebraic_distance(const Ellipse& ell1, const Ellipse& ell2)
{
    return (sym2vec<double, 3>(ell1.AsDual()) - sym2vec<double, 3>(ell2.AsDual())).norm();
}

inline double tangency_segment_ellipsoid(const Eigen::Vector3d& line, const Ellipsoid& ellipsoid, const Matrix34d& P)
{
    Eigen::Vector4d pl = P.transpose() * line;
    return pl.transpose() * ellipsoid.AsDual() * pl;
}

}

#endif // DISTANCE_H