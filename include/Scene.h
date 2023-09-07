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


#ifndef SCENE_H
#define SCENE_H

#include "Ellipsoid.h"

namespace ORB_SLAM2
{

std::vector<Ellipsoid, Eigen::aligned_allocator<Ellipsoid>>
generate_random_scene(size_t n_ellipsoids, double axes_min, double axes_max, double rot_min, double rot_max, double pos_min, double pos_max);

std::pair<Eigen::Matrix3d, Eigen::Vector3d>
generate_random_camera(double azimuth_min, double azimuth_max, double elev_min, double elev_max,
                       double dist_min, double dist_max, double target_min, double target_max);


}

#endif // SCENE_H