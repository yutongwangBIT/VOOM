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


#ifndef COLOR_MANAGER_H
#define COLOR_MANAGER_H

#include <random>
#include <iostream>

#include <opencv2/core.hpp>

namespace ORB_SLAM2
{



class RandomUniformColorGenerator
{
public:
    static cv::Scalar Generate() {
        cv::Scalar color;
        color(0) = RandomUniformColorGenerator::distr(RandomUniformColorGenerator::gen);
        color(1) = RandomUniformColorGenerator::distr(RandomUniformColorGenerator::gen);
        color(2) = RandomUniformColorGenerator::distr(RandomUniformColorGenerator::gen);
        return color;
    }

private:
    static std::random_device rd;
    static std::mt19937 gen;
    static std::uniform_int_distribution<int> distr;
};

const int nb_category_colors = 500;


class CategoryColorsManager
{
public:
    static const CategoryColorsManager& GetInstance() {
        if (!instance) {
            instance = new CategoryColorsManager();
        }
        return *instance;
    }

    static void FreeInstance() {
        if (instance)
            delete instance;
    }

    const cv::Scalar& operator[](size_t idx) const {
        return colors_[idx];
    }

private:
    static CategoryColorsManager *instance;

    std::vector<cv::Scalar> colors_;

    CategoryColorsManager();
};


}

#endif // COLOR_MANAGER_H
