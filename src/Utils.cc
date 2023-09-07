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


#include "Utils.h"

#include <fstream>
#include <iomanip>

namespace ORB_SLAM2
{

void writeOBJ(const std::string& filename, const Eigen::Matrix<double, Eigen::Dynamic, 3>& pts,
              const Eigen::Matrix<int, Eigen::Dynamic, 3>& colors)
{
    std::ofstream f(filename);
    f << std::fixed;
    bool with_colors = pts.rows() == colors.rows();
    for (int j = 0; j < pts.rows(); ++j) {
        f << "v " << std::setprecision(7) << " "<< pts(j, 0)
                                          << " " << pts(j, 1)
                                          << " " << pts(j, 2);
        if (with_colors) {
            f << " " << colors(j, 0) << " " << colors(j, 1) << " " << colors(j, 2);
        }
        f << "\n";
    }
    f.close();
}


}