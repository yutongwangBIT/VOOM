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


#include "Reconstruction.h"

#include <Eigen/Dense>
#include "Distance.h"

namespace ORB_SLAM2
{



Eigen::Vector3d TriangulatePoints2(const Eigen::Vector2d& uv1, const Eigen::Vector2d& uv2,
                                  const Matrix34d& P1, const Matrix34d& P2)
{
    Eigen::Matrix4d A(4, 4);
    A.row(0) = uv1[0] * P1.row(2) - P1.row(0);
    A.row(1) = uv1[1] * P1.row(2) - P1.row(1);
    A.row(2) = uv2[0] * P2.row(2) - P2.row(0);
    A.row(3) = uv2[1] * P2.row(2) - P2.row(1);
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d X = svd.matrixV().col(3);
    Eigen::Vector3d center = X.head(3) / X[3];
    return center;
}


Eigen::Vector3d TriangulatePoints(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points,
                                  const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& projections)
{
    const int n = projections.size();
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(2*n, 4);
    for (int i = 0; i < n; ++i) {
        A.row(i*2) = points[i][0] * projections[i].row(2) - projections[i].row(0);
        A.row(i*2+1) = points[i][1] * projections[i].row(2) - projections[i].row(1);
    }
    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 4>> svd(A, Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::Vector4d X = V.col(3);
    Eigen::Vector3d center = X.head(3) / X[3];
    return center;
}

Eigen::Vector3d TriangulatePointsRansac(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points,
                                        const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& projections,
                                        int max_iter)
{
    double max_dist_inliers = 5.0;
    const int n = projections.size();
    int T  = 0.5 * n * (n - 1);
    int a = 0;
    std::vector<std::pair<int, int>> pairs(T, {-1, -1});
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            pairs[a++] = {i, j};
        }
    }
    std::random_shuffle(pairs.begin(), pairs.end());
    int limits = pairs.size();
    if (max_iter > 0)
        limits = std::min(limits, max_iter);

    int max_nb_inliers = 0;
    std::vector<bool> best_inliers;
    for (int k = 0; k < limits; ++k) {
        auto pair = pairs[k];
        int i0 = pair.first;
        int i1 = pair.second;
        Eigen::Vector3d result = TriangulatePoints2(points[i0], points[i1], projections[i0], projections[i1]);
        Eigen::Vector4d result_h = result.homogeneous();
        std::vector<bool> inliers(n, false);
        int nb_inliers = 0;
        for (int i = 0; i < n; ++i) {
            Eigen::Vector3d p = projections[i] * result_h;
            Eigen::Vector3d uvs = p / p[2];
            double d = (uvs.head<2>() - points[i]).norm();
            if (d < max_dist_inliers) {
                inliers[i] = true;
                ++nb_inliers;
            }
        }
        if (nb_inliers > max_nb_inliers) {
            max_nb_inliers = nb_inliers;
            best_inliers = inliers;
        }
    }
    std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>> inliers_projections(max_nb_inliers);
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> inliers_points(max_nb_inliers);
    a = 0;
    for (int i = 0; i < n; ++i) {
        if (best_inliers[i] == true) {
            inliers_projections[a] = projections[i];
            inliers_points[a] = points[i];
            ++a;
        }
    }

    if (max_nb_inliers < 2)
        return TriangulatePoints(points, projections);
    else
        return TriangulatePoints(inliers_points, inliers_projections);
}


std::pair<bool, Ellipsoid>
ReconstructEllipsoidFromCenters(const std::vector<BBox2, Eigen::aligned_allocator<BBox2>>& bboxes,
                                const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& Rts, 
                                const Eigen::Matrix3d& K)
{
    size_t n = bboxes.size();

    std::vector<double> sizes(n);
    std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>> projections(n);
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points2d(n);

    for (size_t i = 0; i < n; ++i) {
        const auto& bb = bboxes[i];
        points2d[i] = bbox_center(bb);
        projections[i] = K * Rts[i];
        sizes[i] = 0.5 * ((bb[2] - bb[0]) / K(0, 0) + (bb[3] - bb[1]) / K(1, 1));
    }
    
    Eigen::Vector3d center = TriangulatePoints(points2d, projections);
    // Eigen::Vector3d center_no_ransac = TriangulatePoints(points2d, projections);
    // Eigen::Vector3d center = TriangulatePointsRansac(points2d, projections, 1000);

    double mean_3D_size = 0.0;
    for (size_t i = 0; i < n; ++i) {
        Eigen::Vector3d X_cam = Rts[i] * center.homogeneous();
        // Reconstruction failed if the object is behind a camera
        if (X_cam.z() < 0) {
            std::cerr << "Reconstruction failed: z is negative" << std::endl;
            return {false, Ellipsoid()};
        }
        Eigen::Vector3d X_img = K * X_cam;
        double u = X_img[0] / X_img[2];
        double v = X_img[1] / X_img[2];
        
        
        if ((points2d[i] - Eigen::Vector2d(u, v)).norm() > 100) {
            std::cerr << "Reconstruction failed: reconstructed center is too far from a detection" << std::endl;
            return {false, Ellipsoid()};
        }
        mean_3D_size += X_cam.z() * sizes[i];
    }

    mean_3D_size /= sizes.size();
    return {true, Ellipsoid(Eigen::Vector3d::Ones() * mean_3D_size * 0.5, Eigen::Matrix3d::Identity(), center)};
}



std::pair<bool, Ellipsoid>
ReconstructEllipsoidFromLandmarks(const std::vector<BBox2, Eigen::aligned_allocator<BBox2>>& bboxes,
                                  const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& Rts, 
                                  const Eigen::Matrix3d& K, const Eigen::Matrix<double, 3, Eigen::Dynamic>& pts)
{
    std::vector<int> count_obs(pts.cols(), 0);
    int min_visibility = static_cast<int>(0.6 * bboxes.size());

    for (size_t i = 0; i < bboxes.size(); ++i) {
        Eigen::Matrix<double, 3, Eigen::Dynamic> uvs =  K * ((Rts[i].block<3, 3>(0, 0) * pts).colwise() + Rts[i].col(3));
        for(int j = 0; j < pts.cols(); j++) {
            float u = uvs(0, j) / uvs(2, j);
            float v =  uvs(1, j) / uvs(2, j);
            count_obs[j] += is_inside_bbox(u, v, bboxes[i]);
        }
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> M;
    Eigen::Vector3d center(0.0, 0.0, 0.0);
    int a = 0;
    for (size_t i = 0; i < count_obs.size(); ++i) {
        if (count_obs[i] >= min_visibility) {
            M.conservativeResize(Eigen::NoChange, M.cols()+1);
            M.col(a) = pts.col(i);
            center += M.col(a);
            ++a;
        }
    }

    if (M.cols() < 5) {
        std::cerr << "PCA failed: need at least 5 points.\n";
        return {false, Ellipsoid()};
    }

    // Reconstruct
    center /= M.cols();
    Eigen::Matrix<double, 3, Eigen::Dynamic> M_centered = M.colwise() - center;
    Eigen::Matrix3d A = (1.0 / M.cols()) * (M_centered * M_centered.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(A);
    Eigen::Matrix3d R = solver.eigenvectors();
    Eigen::Vector3d s = 2 * solver.eigenvalues().cwiseAbs().cwiseSqrt();
    if (R.determinant() < 0) {
        R.col(2) *= -1;
    }
    return {true, Ellipsoid(s, R, center)};
}




Eigen::MatrixXd compute_B_perspective(const Eigen::Matrix<double, 3, 4>& P)
{
    // P is a 3x4 matrix
    double p11 = P(0, 0), p12 = P(0, 1), p13 = P(0, 2), p14 = P(0, 3);
    double p21 = P(1, 0), p22 = P(1, 1), p23 = P(1, 2), p24 = P(1, 3);
    double p31 = P(2, 0), p32 = P(2, 1), p33 = P(2, 2), p34 = P(2, 3);
    double p11_2 = p11*p11, p12_2 = p12 * p12, p13_2 = p13 * p13, p14_2 = p14 * p14;
    double p21_2 = p21*p21, p22_2 = p22 * p22, p23_2 = p23 * p23, p24_2 = p24 * p24;
    double p31_2 = p31*p31, p32_2 = p32 * p32, p33_2 = p33 * p33, p34_2 = p34 * p34;
    Eigen::MatrixXd B(6, 10);
    B << p11_2, 2*p12*p11, 2*p13*p11, 2*p14*p11, p12_2, 2*p13*p12, 2*p14*p12, p13_2, 2*p13*p14, p14_2,
         p21*p11, p21*p12+p22*p11, p23*p11+p21*p13, p24*p11+p21*p14, p22*p12, p22*p13+p23*p12, p22*p14+p24*p12, p23*p13, p23*p14+p24*p13, p24*p14,
         p31*p11, p31*p12+p32*p11, p33*p11+p31*p13, p34*p11+p31*p14, p32*p12, p32*p13+p33*p12, p32*p14+p34*p12, p33*p13, p33*p14+p34*p13, p34*p14,
         p21_2, 2*p22*p21, 2*p23*p21, 2*p24*p21, p22_2, 2*p23*p22, 2*p24*p22, p23_2, 2*p23*p24, p24_2,
         p31*p21, p31*p22+p32*p21, p33*p21+p31*p23, p34*p21+p31*p24, p32*p22, p32*p23+p33*p22, p32*p24+p34*p22, p33*p23, p33*p24+p34*p23, p34*p24,
         p31_2, 2*p32*p31, 2*p33*p31, 2*p34*p31, p32_2, 2*p33*p32, 2*p34*p32, p33_2, 2*p33*p34, p34_2;
    return B;
}

Ellipsoid reconstruct_ellipsoid_lstsq(const std::vector<Ellipse, Eigen::aligned_allocator<Ellipse>>& ellipses,
                                      const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& projections)
{
    int n_views = ellipses.size();
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6 * n_views, 10 + n_views);
    for (int index = 0; index < n_views; ++index) {
        Eigen::Matrix3d C = ellipses[index].AsDual();
        double h = ellipses[index].GetAxes().squaredNorm();
        Eigen::Matrix3d H;
        H.diagonal() << h, h, 1.0;
        H.block<2, 1>(0, 2) = ellipses[index].GetCenter();
        Eigen::Matrix3d H_inv = H.inverse();
        Eigen::Matrix<double, 3, 4> new_P = (projections[index].transpose() * H_inv.transpose()).transpose();
        Eigen::MatrixXd B = compute_B_perspective(new_P);
        Eigen::Matrix3d C_nc = H_inv * C * H_inv.transpose();
        Eigen::Matrix<double, 6, 1> C_nc_vec = sym2vec<double, 3>(C_nc);
        C_nc_vec /= -C_nc_vec[5];

        M.block<6, 10>(6*index, 0) = B;
        M.block<6, 1>(6*index, 10+index) = -C_nc_vec;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::Matrix<double, 10, 1> w = V.block<10, 1>(0, V.cols()-1);
    Eigen::Matrix4d Qadj = vec2sym<double, 4>(w);
    return Ellipsoid(Qadj);
}

std::pair<bool, Ellipsoid>
ReconstructEllipsoidCrocco(const std::vector<BBox2, Eigen::aligned_allocator<BBox2>>& bboxes,
                           const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& Rts, 
                           const Eigen::Matrix3d& K, bool use_two_passes)
{
    const size_t n = Rts.size();
    std::vector<Ellipse, Eigen::aligned_allocator<Ellipse>> ellipses(n);
    std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>> projections(n);
    for (size_t i = 0; i < n; ++i) {
        ellipses[i] = Ellipse::FromBbox(bboxes[i]);
        projections[i] = K * Rts[i];
    }

    Ellipsoid ellipsoid = reconstruct_ellipsoid_lstsq(ellipses, projections);

    if (use_two_passes) {
        Eigen::Vector3d center = ellipsoid.GetCenter();
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 1>(0, 3) = -center;
        for (size_t i = 0; i < projections.size(); ++i) {
            Matrix34d new_P = projections[i] * T;
            projections[i] = new_P;
        }
        Ellipsoid Q = reconstruct_ellipsoid_lstsq(ellipses, projections);
        Eigen::Matrix4d Q_m = T * Q.AsDual() * T.transpose();
        Eigen::Matrix4d Q_sym = 0.5 * (Q_m + Q_m.transpose());
        ellipsoid = Ellipsoid(Q_sym);
    }

    return {true, Ellipsoid(ellipsoid.GetAxes(), ellipsoid.GetOrientation(), ellipsoid.GetCenter())};
}

}