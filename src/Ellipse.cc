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


#include "Ellipse.h"


namespace ORB_SLAM2
{

    Ellipse::Ellipse(const Eigen::Vector2d& axes, double angle, const Eigen::Vector2d& center) {
        Eigen::Matrix3d C_star = Eigen::Matrix3d(Eigen::Vector3d(std::pow(axes[0], 2), 
                                                                 std::pow(axes[1], 2),
                                                                 -1.0).asDiagonal());
        Eigen::Matrix3d T_center = Eigen::Matrix3d::Identity();
        T_center(0, 2) = center[0];
        T_center(1, 2) = center[1];
        Eigen::Matrix3d Rw_e;
        Rw_e << std::cos(angle), -std::sin(angle), 0.0,
                std::sin(angle),  std::cos(angle), 0.0,
                0.0, 0.0, 1.0;
        Eigen::Matrix3d transf = T_center * Rw_e;
        C_star = transf * C_star * transf.transpose();

        C_ = 0.5 * (C_star + C_star.transpose());
        C_ /= -C_(2, 2);

        axes_ = axes;
        angle_ = angle;
        center_ = center;
        has_changed_ = false;
    }


    void Ellipse::decompose() const {
        // assumes C_(2, 2) == -1
        center_ = -C_.col(2).head(2);
        Eigen::Matrix3d T_c = Eigen::Matrix3d::Identity();
        T_c(0, 2) = -center_[0];
        T_c(1, 2) = -center_[1];
        Eigen::Matrix3d temp = T_c * C_ * T_c.transpose();
        Eigen::Matrix3d C_center = 0.5 * (temp + temp.transpose());

        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(C_center.block<2, 2>(0, 0));
        Eigen::Matrix2d eig_vectors = eigen_solver.eigenvectors();
        Eigen::Vector2d eig_values = eigen_solver.eigenvalues();
        
        if (eig_vectors.determinant() < 0.0) {
            eig_vectors.col(1) *= -1;
        }
        if (eig_vectors(0, 0) < 0) {    // force first axis to be positive in x
            eig_vectors *= -1.0;
        }

        axes_ = eig_values.cwiseAbs().cwiseSqrt();
        angle_ = std::atan2(eig_vectors(1, 0), eig_vectors(0, 0));
        has_changed_ = false;
    }

    BBox2 Ellipse::ComputeBbox() const
    {
        if (has_changed_) {
            decompose();
            has_changed_ = false;
        }
        double c = std::cos(angle_);
        double s = std::sin(angle_);
        double xmax = std::sqrt(std::pow(axes_[0]*c, 2) + std::pow(-axes_[1]*s, 2));
        double ymax = std::sqrt(std::pow(axes_[0]*s, 2) + std::pow( axes_[1]*c, 2));

        BBox2 bb;
        bb << center_[0] - xmax, center_[1] - ymax,
              center_[0] + xmax, center_[1] + ymax;
        return bb;
    }

    Eigen::Matrix3d Ellipse::ComposePrimalMatrix() const
    {
        if (has_changed_) {
            decompose();
            has_changed_ = false;
        }

        Eigen::Matrix3d A;
        A << 1.0 / std::pow(axes_[0], 2), 0.0, 0.0,
            0.0, 1.0 / std::pow(axes_[1], 2), 0.0,
            0.0, 0.0, -1.0;
        Eigen::Matrix3d R;
        R << std::cos(angle_), -std::sin(angle_), 0.0,
            std::sin(angle_), std::cos(angle_), 0.0,
            0.0, 0.0, 1.0;
        Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
        T(0, 2) = -center_[0];
        T(1, 2) = -center_[1];
        Eigen::Matrix3d C = T.transpose() * R * A * R.transpose() * T;
        return C;
    }

    std::ostream& operator <<(std::ostream& os, const Ellipse& ell)
    {
        if (ell.has_changed_) {
            ell.decompose();
        }
        os << "Ellipse: ";
        os << "axes: " << ell.axes_.transpose() << " ";
        os << "angle: " << ell.angle_ << " ";
        os << "center: " << ell.center_.transpose();
        return os;
    }
}