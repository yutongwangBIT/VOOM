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


#include "ColorManager.h"


namespace ORB_SLAM2
{

std::random_device RandomUniformColorGenerator::rd("default");
std::mt19937 RandomUniformColorGenerator::gen(RandomUniformColorGenerator::rd());
std::uniform_int_distribution<int> RandomUniformColorGenerator::distr = std::uniform_int_distribution<int>(0, 255);


CategoryColorsManager *CategoryColorsManager::instance = nullptr;


CategoryColorsManager::CategoryColorsManager()
    : colors_(std::vector<cv::Scalar>(nb_category_colors)) {
    for (int i = 0; i < nb_category_colors; ++i) {
        colors_[i] = RandomUniformColorGenerator::Generate();
    }
}

}
