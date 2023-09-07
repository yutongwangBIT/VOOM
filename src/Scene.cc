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


#include "Scene.h"

#include <random>
#include "Camera.h"

namespace ORB_SLAM2
{

std::default_random_engine gen(time(NULL));
std::uniform_real_distribution<double> dist(0.0,1.0);

std::vector<Ellipsoid, Eigen::aligned_allocator<Ellipsoid>>
generate_random_scene(size_t n_ellipsoids, double axes_min, double axes_max, double rot_min, double rot_max, double pos_min, double pos_max)
{
    std::vector<Ellipsoid, Eigen::aligned_allocator<Ellipsoid>> ellipsoids(n_ellipsoids);
    for (size_t i = 0; i < n_ellipsoids; ++i) {
        Eigen::Vector3d axes, euler, position;
        axes << dist(gen) * (axes_max-axes_min) + axes_min,
                dist(gen) * (axes_max-axes_min) + axes_min,
                dist(gen) * (axes_max-axes_min) + axes_min;
        euler << dist(gen) * (rot_max-rot_min) + rot_min,
                 dist(gen) * (rot_max-rot_min) + rot_min,
                 dist(gen) * (rot_max-rot_min) + rot_min;
        position << dist(gen) * (pos_max-pos_min) + pos_min,
                    dist(gen) * (pos_max-pos_min) + pos_min,
                    dist(gen) * (pos_max-pos_min) + pos_min;
        

        Eigen::Matrix3d R = (Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ())
                             * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitX())
                             * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
        Ellipsoid ell(axes, R, position);
        ellipsoids[i] = ell;
    }
    return ellipsoids;    
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d> generate_random_camera(double azimuth_min, double azimuth_max, double elev_min, double elev_max,
                                                                   double dist_min, double dist_max, double target_min, double target_max)
{
    // std::default_random_engine gen;
    // std::uniform_real_distribution<double> dist(0.0,1.0);
    double d = dist(gen) * (dist_max - dist_min) + dist_min;
    double az = dist(gen) * (azimuth_max - azimuth_min) + azimuth_min;
    double elev = dist(gen) * (elev_max - elev_min) + elev_min;
    Eigen::Vector3d target;
    target << dist(gen) * (target_max - target_max) + target_max,
              dist(gen) * (target_max - target_max) + target_max,
              dist(gen) * (target_max - target_max) + target_max;
    Eigen::Vector3d pos;
    pos << std::cos(az) * std::cos(elev) * d,
           std::sin(az) * std::cos(elev) * d,
           std::sin(elev) * d;
    return std::make_pair(look_at(pos, target), pos);
}
    
}