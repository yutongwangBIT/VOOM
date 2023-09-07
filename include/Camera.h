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

#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>

namespace ORB_SLAM2
{


// Compute the orientaiton of a camera placed at center and looking at target, with y axis aligned
// with up.
Eigen::Matrix3d look_at(const Eigen::Vector3d& center,
                        const Eigen::Vector3d& target,
                        const Eigen::Vector3d& up=Eigen::Vector3d::UnitZ());

inline Eigen::Matrix3d generate_K(double fx, double fy, double ppx, double ppy) {
    Eigen::Matrix3d K;
    K << fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0;
    return K;
}

std::pair<Eigen::Matrix<double, Eigen::Dynamic, 3>, Eigen::Matrix<int, Eigen::Dynamic, 3>>
generate_camera_triaxe(const Eigen::Matrix3d& orientation, const Eigen::Vector3d& position, int sampling);

// Compute a rotation error between two rotation matrices.
inline double rotation_error(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2) {
    return Eigen::AngleAxisd(R1 * R2.transpose()).angle();
}

// Compute the pose error between two cameras
inline std::pair<double, double> pose_error(const Eigen::Matrix3d& o1, const Eigen::Vector3d& p1,
                                            const Eigen::Matrix3d& o2, const Eigen::Vector3d& p2)
{
    return std::make_pair(rotation_error(o1, o2), (p1 - p2).norm());
}


}


#endif // CAMERA_H