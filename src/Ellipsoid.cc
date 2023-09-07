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


#include "Ellipsoid.h"

namespace ORB_SLAM2
{





Ellipsoid::Ellipsoid(const Eigen::Vector3d& axes,
                        const Eigen::Matrix3d& R,
                        const Eigen::Vector3d& center)
{
    Eigen::Matrix4d Q_star = Eigen::Matrix4d(Eigen::Vector4d(std::pow(axes[0], 2),
                                                                std::pow(axes[1], 2),
                                                                std::pow(axes[2], 2),
                                                                -1.0).asDiagonal());
    Eigen::Matrix4d T_center = Eigen::Matrix4d::Identity();
    T_center(0, 3) = center[0];
    T_center(1, 3) = center[1];
    T_center(2, 3) = center[2];
    Eigen::Matrix4d Rw_e = Eigen::Matrix4d::Identity();
    Rw_e.block<3, 3>(0, 0) = R;

    Eigen::Matrix4d transf = T_center * Rw_e;
    Q_star = transf * Q_star * transf.transpose();

    Q_ = 0.5 * (Q_star + Q_star.transpose());
    Q_ /= -Q_(3, 3);

    axes_ = axes;
    R_ = R;
    center_ = center;
    has_changed_ = false;
}

void Ellipsoid::decompose() const {
    // assumes Q_(3, 3) == -1
    center_ = -Q_.col(3).head(3);

    Eigen::Matrix4d T_c = Eigen::Matrix4d::Identity();
    T_c(0, 3) = -center_[0];
    T_c(1, 3) = -center_[1];
    T_c(2, 3) = -center_[2];
    Eigen::Matrix4d temp = T_c * Q_ * T_c.transpose();
    Eigen::Matrix4d Q_center = 0.5 * (temp + temp.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(Q_center.block<3, 3>(0, 0));
    Eigen::Matrix3d eig_vectors = eigen_solver.eigenvectors();
    Eigen::Vector3d eig_values = eigen_solver.eigenvalues();

    if (eig_vectors.determinant() < 0.0) {
        eig_vectors.col(2) *= -1;
    }

    axes_ = eig_values.cwiseAbs().cwiseSqrt();
    R_ = eig_vectors;
    has_changed_ = false;
}


BBox3 Ellipsoid::ComputeBbox() const {
    if (has_changed_) {
        decompose();
    }

    Eigen::Matrix3d M = R_ * Eigen::DiagonalMatrix<double, 3>(axes_);
    double dx = M.row(0).norm();
    double dy = M.row(1).norm();
    double dz = M.row(2).norm();
    BBox3 bb;
    bb << center_[0] - dx, center_[1] - dy, center_[2] - dz,
          center_[0] + dx, center_[1] + dy, center_[2] + dz;
    return bb;
}

bool Ellipsoid::IsInside(const Eigen::Vector3d& pt, double dilatation_coefficient) const {
    if (has_changed_) {
        decompose();
    }
    Eigen::Vector3d pt_e = R_.transpose() * (pt - center_);
    double a0_new = axes_[0] * dilatation_coefficient;
    double a1_new = axes_[1] * dilatation_coefficient;
    double a2_new = axes_[2] * dilatation_coefficient;
    return std::pow(pt_e[0] / a0_new, 2) + std::pow(pt_e[1] / a1_new, 2) + std::pow(pt_e[2] / a2_new, 2) <= 1.0;
}


std::ostream& operator <<(std::ostream& os, const Ellipsoid& ell)
{
    if (ell.has_changed_) {
        ell.decompose();
    }
    os << "Ellipsoid:\n";
    os << "axes: " << ell.axes_.transpose() << "\n";
    os << "orientation: \n" << ell.R_ << "\n";
    os << "center: " << ell.center_.transpose();
    return os;
}


Eigen::Matrix<double, Eigen::Dynamic, 3> generate_ellipsoid_points(int azimuths, int elevations, int sampling) {
    int n = (azimuths + elevations) * (sampling);
    Eigen::Matrix<double, Eigen::Dynamic, 3> pts(n, 3);
    int k = 0;

    double a_step = 360.0 / azimuths;

    for (int i = 0; i < azimuths; ++i) {
        double az = TO_RAD(a_step * i);
        double e_step = 180.0 / (sampling-1);
        for (int j = 0; j < sampling; ++j) {
            double elev = TO_RAD(-90.0 + e_step * j);
            pts(k, 0) = std::cos(az) * std::cos(elev);
            pts(k, 1) = std::sin(az) * std::cos(elev);
            pts(k, 2) = std::sin(elev);
            ++k;
        }
    }

    double e_step = 180.0 / elevations;
    for (int i = 0; i < elevations; ++i) {
        double el = TO_RAD(-90.0 + e_step * i);
        double a_step = 360.0 / (sampling-1);
        for (int j = 0; j < sampling; ++j) {
            double az = TO_RAD(a_step * j);
            pts(k, 0) = std::cos(az) * std::cos(el);
            pts(k, 1) = std::sin(az) * std::cos(el);
            pts(k, 2) = std::sin(el);
            ++k;
        }
    }
    return pts;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> Ellipsoid::base_ellipsoid_pts = generate_ellipsoid_points(8, 8, 50);

Eigen::Matrix<double, Eigen::Dynamic, 3> Ellipsoid::GeneratePointCloud(int sampling) const {
    Eigen::Matrix<double, Eigen::Dynamic, 3> pts;
    if (sampling == 50) {
        pts = base_ellipsoid_pts;
    } else {
        pts =  generate_ellipsoid_points(8, 8, sampling);
    }
    const Eigen::Vector3d& axes = GetAxes();

    pts.col(0) *= axes[0];
    pts.col(1) *= axes[1];
    pts.col(2) *= axes[2];

    Eigen::Matrix<double, 3, Eigen::Dynamic> pts_transf = (GetOrientation() * pts.transpose());
    pts_transf.colwise() += GetCenter();
    return pts_transf.transpose();
}

}