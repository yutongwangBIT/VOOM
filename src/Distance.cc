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

#include "Distance.h"

#include <unsupported/Eigen/MatrixFunctions>
#include "Ellipse.h"

namespace ORB_SLAM2 
{


double gaussian_wasserstein_2d(const Ellipse& ell1, const Ellipse& ell2)
{
    auto [mu1, sigma1] = ell1.AsGaussian();
    auto [mu2, sigma2] = ell2.AsGaussian();

    Eigen::Matrix2d sigma11 = sigma1.sqrt();
    Eigen::Matrix2d s121 = sigma11 * sigma2 * sigma11;
    Eigen::Matrix2d sigma121 = s121.sqrt();

    double d = (mu1-mu2).squaredNorm() + (sigma1 + sigma2 - 2 * sigma121).trace();
    return d;
}

double normalized_gaussian_wasserstein_2d(const Ellipse& ell1, const Ellipse& ell2, const double constant_C)
{
    return exp(-sqrt(gaussian_wasserstein_2d(ell1, ell2))/constant_C);
}


std::vector<std::vector<double>> generate_sampling_points(const Ellipse& ell, int count_az, int count_dist, double scale)
{
    double angle = ell.GetAngle();
    Eigen::Matrix2d R;
    R << std::cos(angle), -std::sin(angle),
         std::sin(angle),  std::cos(angle);

    double d_dist = scale / count_dist;
    double d_az = 2 * M_PI / count_az;
    std::vector<std::vector<double>> points(count_az * count_dist + 1, std::vector<double>(2, 0.0));
    Eigen::Vector2d center = ell.GetCenter();
    Eigen::Vector2d axes = ell.GetAxes();
    points[0][0] = center.x();
    points[0][1] = center.y();
    for (int i = 0; i < count_dist; ++i)
    {
        for (int j = 0; j < count_az; ++j)
        {
            double s = (i+1) * d_dist;
            Eigen::Vector2d v(std::cos(d_az * j) * axes.x() * s, std::sin(d_az * j) * axes.y() * s);
            Eigen::Vector2d pt = center + R * v;
            points[1 + i * count_az + j][0] = pt.x();
            points[1 + i * count_az + j][1] = pt.y();
        }
    }

    return points;
}



std::tuple<double, std::vector<double>, std::vector<double>, std::vector<std::vector<double>>>
 ellipses_sampling_metric_ell(const Ellipse& ell1, const Ellipse& ell2, int N_az, int N_dist, double sampling_scale)
{

    double angle1 = ell1.GetAngle();
    double angle2 = ell2.GetAngle();

    Eigen::Matrix2d R1;
    R1 << std::cos(angle1), -std::sin(angle1),
          std::sin(angle1), std::cos(angle1);
    

    Eigen::Matrix2d R2;
    R2 << std::cos(angle2), -std::sin(angle2),
          std::sin(angle2), std::cos(angle2);
    

    Eigen::Vector2d axes1 = ell1.GetAxes();
    Eigen::Vector2d axes2 = ell2.GetAxes();
    Eigen::Matrix2d A1; 
    A1 << 1.0 / std::pow(axes1[0], 2), 0.0, 
          0.0, 1.0 / std::pow(axes1[1], 2);

    Eigen::Matrix2d A2;
    A2 << 1.0 / std::pow(axes2[0], 2), 0.0, 
          0.0, 1.0 / std::pow(axes2[1], 2);


// std::cout << "A1 = \n" << A1 << "\n";
// std::cout << "A2 = \n" << A2 << "\n";
    /////////////////////////////////////////////////////// scale the ellipses center somewhere !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    Eigen::Matrix2d D1 = R1 * A1 * R1.transpose();
    Eigen::Matrix2d D2 = R2 * A2 * R2.transpose();

    // std::cout << "D1 = \n" << D1 << "\n";
    // std::cout << "D2 = \n" << D2 << "\n";


    
    // std::ofstream fout("debug.txt");
    

    std::vector<std::vector<double>> points = generate_sampling_points(ell1, N_az, N_dist, sampling_scale);
    // std::vector<std::vector<double>> points2 = generate_sampling_points(ell2, N_az, N_dist);
    // points.insert(points.end(), points2.begin(), points2.end());

    Eigen::Vector2d center1 = ell1.GetCenter();
    Eigen::Vector2d center2 = ell2.GetCenter();
    std::vector<double> errors(points.size(), 0.0);
    double total_err = 0.0;
    int i = 0;
    for (const auto& pt : points)
    {
        // std::cout << i*dx << " " << j * dy << " ===> " << ell1_norm.center.transpose() << " " << ell2_norm.center.transpose() << "\n";
        // if (pt[0] < 0 || pt[0] >= 640 || pt[1] < 0 || pt[1] >= 480)
        //     continue;
        Eigen::Vector2d p(pt[0], pt[1]);

        Eigen::Vector2d pt1 = p - center1;
        Eigen::Vector2d pt2 = p - center2;

        double v1 = pt1.transpose() * D1 * pt1;
        double v2 = pt2.transpose() * D2 * pt2;
        // fout << v1 << " " << v2 << "\n";
        errors[i++] = v1 - v2;
        total_err += std::pow(v1-v2, 2);
    }
    errors.resize(i);
    // return std::make_tuple(std::sqrt(total_err/(N_az*N_dist)), errors, std::vector<double>({}), points);
    return std::make_tuple(total_err, errors, std::vector<double>({}), points);
}




double gaussian_bhattacharrya_2d(const Ellipse& ell1, const Ellipse& ell2)
{
    auto [mu1, sigma1] = ell1.AsGaussian();
    auto [mu2, sigma2] = ell2.AsGaussian();

    Eigen::Matrix2d G = (sigma1 + sigma2) * 0.5;
    Eigen::Matrix2d G_inv = G.inverse();
    Eigen::Vector2d mu_diff = mu1 - mu2;
    Eigen::Vector2d temp = G_inv * mu_diff;
    double dist = 0.125 * mu_diff.dot(temp);
    dist += 0.5 * std::log(G.determinant() / std::sqrt(sigma1.determinant() * sigma2.determinant()));
    return dist;
}




std::vector<double> solve_polynomial(double a, double b, double c, bool force_one_solution=false)
{
    double d = b * b - 4 * a * c;
    if (force_one_solution)
        d = 0.0;
    if (std::abs(d) < 1e-8) {
        double res = -0.5 * b / a;
        return {res, res};
    } else if (d < 0) {
        return {};
    } else {
        double sd = std::sqrt(d);
        return {0.5 * (-b-sd) / a,  0.5 * (-b+sd) / a};
    }
}


std::vector<Eigen::Vector2d> find_intersection_ellipse_horiz_line(const Ellipse& ell, double y, bool force_tangency=false)
{
    Eigen::Matrix3d C = ell.ComposePrimalMatrix();
    auto res = solve_polynomial(C(0, 0), 2*C(0, 1)*y + 2*C(0, 2), C(1, 1)*std::pow(y, 2) + 2*C(1, 2)*y + C(2, 2), force_tangency);
    if (res.size() == 0) return {};
    Eigen::Vector2d a(res[0], y);
    Eigen::Vector2d b(res[1], y);
    return {a, b};
}


std::vector<Eigen::Vector2d> find_intersection_ellipse_vert_line(const Ellipse& ell, double x, bool force_tangency=false)
{
    Eigen::Matrix3d C = ell.ComposePrimalMatrix();
    auto res = solve_polynomial(C(1, 1), 2*C(0, 1)*x + 2*C(1, 2), C(0, 0)*std::pow(x, 2) + 2*C(0, 2)*x + C(2, 2), force_tangency);
    if (res.size() == 0) return {};
    Eigen::Vector2d a(x, res[0]);
    Eigen::Vector2d b(x, res[1]);
    return {a, b};
}

std::vector<Eigen::Vector2d> find_intersection_ellipse_bbox(const Ellipse& ell, const BBox2 bbox)
{
    std::vector<Eigen::Vector2d> inter;
    auto pts = find_intersection_ellipse_horiz_line(ell, bbox[1]);
    for (auto& p : pts) {
        if (p[0] >= bbox[0] && p[0] <= bbox[2])
            inter.push_back(p);
    }

    pts = find_intersection_ellipse_vert_line(ell, bbox[0]);
    for (auto& p : pts) {
        if (p[1] >= bbox[1] && p[1] <= bbox[3])
            inter.push_back(p);
    }

    pts = find_intersection_ellipse_horiz_line(ell, bbox[3]);
    for (auto& p : pts) {
        if (p[0] >= bbox[0] && p[0] <= bbox[2])
            inter.push_back(p);
    }

    pts = find_intersection_ellipse_vert_line(ell, bbox[2]);
    for (auto& p : pts) {
        if (p[1] >= bbox[1] && p[1] <= bbox[3])
            inter.push_back(p);
    }
    return inter;
}

std::tuple<bool, BBox2> find_on_image_bbox(const Ellipse& ell, int width, int height)
{
    auto bb = ell.ComputeBbox();
    if (bb[2] < 0 || bb[3] < 0 || bb[0] >= width || bb[1] >= height)
        return {false, bb};

    Eigen::Vector2d p0 = find_intersection_ellipse_vert_line(ell,  bb[0], true)[0];
    Eigen::Vector2d p1 = find_intersection_ellipse_horiz_line(ell, bb[1], true)[0];
    Eigen::Vector2d p2 = find_intersection_ellipse_vert_line(ell,  bb[2], true)[0];
    Eigen::Vector2d p3 = find_intersection_ellipse_horiz_line(ell, bb[3], true)[0];

    std::vector<Eigen::Vector2d> pts = {p0, p1, p2, p3};

    BBox2 image_bb;
    image_bb << 0, 0, width-1, height-1;

    auto intersections = find_intersection_ellipse_bbox(ell, image_bb);
    pts.insert(pts.end(), intersections.begin(), intersections.end());
    double xmin = static_cast<double>(width-1);
    double ymin = static_cast<double>(height-1);
    double xmax = 0.0;
    double ymax = 0.0;
    int count = 0;

    for (auto& p : pts)
    {
        if (p[0] < 0 || p[0] >= width || p[1] < 0 || p[1] >= height)
            continue;
        xmin = std::min(xmin, p[0]);
        ymin = std::min(ymin, p[1]);
        xmax = std::max(xmax, p[0]);
        ymax = std::max(ymax, p[1]);
        count++;
    }
    if (count == 0) {
        return {true, {0, 0, width-1, height-1}};
    }
    return {true, {xmin, ymin, xmax, ymax}};
}

}