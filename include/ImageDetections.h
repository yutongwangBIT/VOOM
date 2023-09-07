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


#ifndef IMAGE_DETECTIONS_H
#define IMAGE_DETECTIONS_H

#include <fstream>
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

#include "Utils.h"
#include "Ellipse.h"
#include <opencv2/opencv.hpp>


namespace ORB_SLAM2
{

class Detection
{
public:
    typedef std::shared_ptr<Detection> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Detection(unsigned int cat, double det_score, const BBox2 bb)
    : category_id(cat), score(det_score), bbox(bb), ell(Ellipse()) {}

    Detection(unsigned int cat, double det_score, const BBox2 bb, Ellipse ell_data)
    : category_id(cat), score(det_score), bbox(bb), ell(ell_data) {}


    friend std::ostream& operator <<(std::ostream& os, const Detection& det);
    unsigned int category_id;
    double score;
    BBox2 bbox;
    Ellipse ell;

private:
    Detection() = delete;
};


class ImageDetectionsManager {
public:

    ImageDetectionsManager() {}
    virtual ~ImageDetectionsManager() {}

    virtual std::vector<Detection::Ptr> detect(const std::string& name) const = 0;
    virtual std::vector<Detection::Ptr> detect(unsigned int idx) const = 0;
    virtual std::vector<Detection::Ptr> detect(cv::Mat img) const = 0;
};


class DetectionsFromFile : public ImageDetectionsManager
{
public:
    DetectionsFromFile(const std::string& filename, const std::vector<int>& cats_to_ignore);

    DetectionsFromFile(const std::string& filename);

    ~DetectionsFromFile() {}

    std::vector<Detection::Ptr> detect(const std::string& name) const;

    std::vector<Detection::Ptr> detect(unsigned int idx) const;

    std::vector<Detection::Ptr> detect(cv::Mat img) const {
        std::cerr << "This function is not available. You should pass the image filename or index as input\n";
        return {};
    }

private:
    std::unordered_map<std::string, std::vector<Detection::Ptr>> detections_;
    std::vector<std::string> frame_names_;
    json data_;
};


#ifdef USE_DNN
class ObjectDetector : public ImageDetectionsManager
{
public:
    ObjectDetector(const std::string& model, const std::vector<int>& cats_to_ignore);
    ~ObjectDetector() {}


    std::vector<Detection::Ptr> detect(cv::Mat img) const;

    std::vector<Detection::Ptr> detect(const std::string& name) const {
        std::cerr << "This function is not available. You should pass an image as input\n";
        return {};
    }
    std::vector<Detection::Ptr> detect(unsigned int idx) const {
        std::cerr << "This function is not available. You should pass an image as input\n";
        return {};
    }

private:
    // cv::dnn::Net network_;
    std::unique_ptr<cv::dnn::Net> network_;
    std::unordered_set<int> ignored_cats_;
};
#else
class ObjectDetector : public ImageDetectionsManager
{
public:
    ObjectDetector(const std::string& model, const std::vector<int>& cats_to_ignore) {
        std::cerr << "Object detection is only available with dnn module of opencv >= 4\n";
    }
    ~ObjectDetector() {}


    std::vector<Detection::Ptr> detect(cv::Mat img) const {
        std::cerr << "Object detection is only available with dnn module of opencv >= 4\n";
        return {};
    }

    std::vector<Detection::Ptr> detect(const std::string& name) const {
        std::cerr << "Object detection is only available with dnn module of opencv >= 4\n";
        return {};
    }
    std::vector<Detection::Ptr> detect(unsigned int idx) const {
        std::cerr << "Object detection is only available with dnn module of opencv >= 4\n";
        return {};
    }
};
#endif


}

#endif