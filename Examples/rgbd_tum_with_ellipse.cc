#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <stdlib.h>     /* srand, rand */

#include<opencv2/core/core.hpp>

#include <ImageDetections.h>
#include <System.h>
#include "Osmap.h"
#include <nlohmann/json.hpp>
#include <experimental/filesystem>
#include "Utils.h"

using json = nlohmann::json;

namespace fs = std::experimental::filesystem;

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);



int main(int argc, char **argv)
{
    srand(time(nullptr));
    std::cout << "C++ version: " << __cplusplus << std::endl;

    if(argc != 8)
    {
        cerr << endl << "Usage:\n"
                        " ./rgbd_diamond_with_ellipse\n"
                        "      vocabulary_file\n"
                        "      camera_file\n"
                        "      path_to_image_sequence (.txt file listing the images or a folder with rgb.txt or 'webcam_id')\n"
                        "      path_to_association \n"
                        "      detections_file (.json file with detections)\n"
                        "      relocalization_mode ('points', 'objects' or 'points+objects' currently on points used)\n"
                        "      output_name \n";
        return 1;
    }

    std::string vocabulary_file = string(argv[1]);
    std::string parameters_file = string(argv[2]);
    string path_to_images = string(argv[3]);
    string strAssociationFilename = string(argv[4]);
    std::string detections_file(argv[5]);
    string reloc_mode = string(argv[6]);
    string output_name = string(argv[7]);

    string output_folder = output_name;
    if (output_folder.back() != '/')
        output_folder += "/";
    fs::create_directories(output_folder);

    // Load object detections
    auto extension = get_file_extension(detections_file);
    std::shared_ptr<ORB_SLAM2::ImageDetectionsManager> detector = nullptr;
    bool detect_from_file = false;
    if (extension == "json") { // load from external detections file
        detector = std::make_shared<ORB_SLAM2::DetectionsFromFile>(detections_file);
        detect_from_file = true;
    } else {
        std::cout << "Invalid detection file. It should be .json\n"
                      "No detections will be obtained.\n";
    }


    // Relocalization mode
    ORB_SLAM2::enumRelocalizationMode relocalization_mode = ORB_SLAM2::RELOC_POINTS;
    if (reloc_mode == string("points"))
        relocalization_mode = ORB_SLAM2::RELOC_POINTS;
    else if (reloc_mode == std::string("objects"))
        relocalization_mode = ORB_SLAM2::RELOC_OBJECTS;
    else if (reloc_mode == std::string("points+objects"))
        relocalization_mode = ORB_SLAM2::RELOC_OBJECTS_POINTS;
    else {
        std::cerr << "Error: Invalid parameter for relocalization mode. "
                     "It should be 'points', 'objects' or 'points+objects'.\n";
        return 1;
    }

    // Load images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create system
    ORB_SLAM2::System SLAM(vocabulary_file, parameters_file, ORB_SLAM2::System::RGBD, true, false);
    SLAM.SetRelocalizationMode(relocalization_mode);

    //nImages = 900;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.reserve(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    ORB_SLAM2::Osmap osmap = ORB_SLAM2::Osmap(SLAM);

    // Main loop
    cv::Mat imRGB, imD;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
    poses.reserve(nImages);
    std::vector<std::string> filenames;
    filenames.reserve(nImages);
    std::vector<double> timestamps;
    timestamps.reserve(nImages);
    int ni = 0;
    while (1)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        std::string filename;
        filename = path_to_images + '/' + vstrImageFilenamesRGB[ni];//path_to_images + vstrImageFilenames[ni];
        //std::cout<<"filename:"<<filename<<std::endl;
        imRGB = cv::imread(filename, cv::IMREAD_UNCHANGED);  // read image from disk
        imD = cv::imread(path_to_images + '/' + vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        
        double tframe = ni < vTimestamps.size() ? vTimestamps[ni] : std::time(nullptr);
        timestamps.push_back(tframe);
        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image: "
                 << filename << endl;
            return 1;
        }
        filenames.push_back(filename);

        // Get object detections
        std::vector<ORB_SLAM2::Detection::Ptr> detections;
        if (detector) {
            if (detect_from_file)
                detections = detector->detect(filename); // from detections file
            else
                detections = detector->detect(imRGB);  // from neural network
        }

        // Pass the image and detections to the SLAM system
        //cv::Mat m = SLAM.TrackMonocular(im, tframe, detections, false);
        cv::Mat m = SLAM.TrackRGBD(imRGB,imD,tframe, detections, false, false);

        if (m.rows && m.cols)
            poses.push_back(ORB_SLAM2::cvToEigenMatrix<double, float, 4, 4>(m));
        else
            poses.push_back(Eigen::Matrix4d::Identity());

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack.push_back(ttrack);
        //std::cout << "time = " << ttrack << "\n";

        if (SLAM.ShouldQuit())
            break;

        ++ni;
        if (ni >= nImages)
            break;
    }

    // Stop all threads
    SLAM.Shutdown();


    // Save camera tracjectory

    // TXT files
    std::ofstream file(output_folder + "camera_poses_" + output_name + ".txt");
    std::ofstream file_tum(output_folder + "camera_poses_" + output_name + "_tum.txt");    // output poses in the TUM RGB-D format
    json json_data;
    for (unsigned int i = 0; i < poses.size(); ++i)
    {
        Eigen::Matrix4d m = poses[i];
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = m.block<3, 3>(0, 0).transpose();
        pose.block<3, 1>(0, 3) = -m.block<3, 3>(0, 0).transpose() * m.block<3, 1>(0, 3);

        file << i << " " << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " " << pose(0, 3) << " "
                  << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " " << pose(1, 3) << " "
                  << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << "\n";


        json R({{m(0, 0), m(0, 1), m(0, 2)},
                {m(1, 0), m(1, 1), m(1, 2)},
                {m(2, 0), m(2, 1), m(2, 2)}});
        json t({m(0, 3), m(1, 3), m(2, 3)});
        json image_data;
        image_data["file_name"] = filenames[i];
        image_data["R"] = R;
        image_data["t"] = t;
        json_data.push_back(image_data);

        auto q = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
        auto p = pose.block<3, 1>(0, 3);
        file_tum << std::fixed << timestamps[i] << " " << p[0] << " " << p[1] << " " << p[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n"; 
    }
    file.close();
    file_tum.close();


    // JSON files
    std::ofstream json_file(output_folder + "camera_poses_" + output_name + ".json");
    json_file << json_data;
    json_file.close();


    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl << endl;

    // Save camera trajectory, points and objects
    SLAM.SaveKeyFrameTrajectoryTUM(output_folder + "keyframes_poses_" + output_name + "_tum.txt");
    SLAM.SaveKeyFrameTrajectoryJSON(output_folder + "keyframes_poses_" + output_name + ".json", filenames);
    SLAM.SaveMapPointsOBJ(output_folder + "map_points_" + output_name + ".obj");
    SLAM.SaveMapObjectsOBJ(output_folder + "map_objects_" + output_name + ".obj");
    SLAM.SaveMapObjectsTXT(output_folder + "map_objects_" + output_name + ".txt");
    std::cout << "\n";

    // Save a reloadable map
    osmap.mapSave(output_folder + "map_" + output_name);

    return 0;
}


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}