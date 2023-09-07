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


#include "Camera.h"


namespace ORB_SLAM2
{

Eigen::Matrix3d look_at(const Eigen::Vector3d& center,
                        const Eigen::Vector3d& target,
                        const Eigen::Vector3d& up)
{
    Eigen::Vector3d z = target - center;
    Eigen::Vector3d zn = z;
    zn.normalize();
    Eigen::Vector3d x = -up.cross(z);
    Eigen::Vector3d xn = x;
    xn.normalize();
    Eigen::Vector3d y = z.cross(x);
    Eigen::Vector3d yn = y;
    yn.normalize();
    Eigen::Matrix3d R;
    R << xn, yn, zn;
    return R;
}


std::pair<Eigen::Matrix<double, Eigen::Dynamic, 3>, Eigen::Matrix<int, Eigen::Dynamic, 3>>
generate_camera_triaxe(const Eigen::Matrix3d& orientation, const Eigen::Vector3d& position, int sampling)
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> pts(sampling*3, 3);
    Eigen::Matrix<int, Eigen::Dynamic, 3> colors(sampling*3, 3);
    for (int i = 0; i < sampling; ++i) {
        double f = static_cast<double>(i) / (sampling-1);
        pts.row(i) = (orientation.col(0) * f + position).transpose();
        colors(i, 0) = 255;
        colors(i, 1) = 0;
        colors(i, 2) = 0;
    }
    for (int i = 0; i < sampling; ++i) {
        double f = static_cast<double>(i) / (sampling-1);
        pts.row(sampling+i) = (orientation.col(1) * f + position).transpose();
        colors(sampling+i, 0) = 0;
        colors(sampling+i, 1) = 255;
        colors(sampling+i, 2) = 0;
    }
    for (int i = 0; i < sampling; ++i) {
        double f = static_cast<double>(i) / (sampling-1);
        pts.row(2*sampling+i) = (orientation.col(2) * f + position).transpose();
        colors(2*sampling+i, 0) = 0;
        colors(2*sampling+i, 1) = 0;
        colors(2*sampling+i, 2) = 255;
    }
    return std::make_pair(pts, colors);
}

}