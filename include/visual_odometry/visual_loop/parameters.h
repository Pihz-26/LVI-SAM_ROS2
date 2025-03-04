#pragma once

#include <rclcpp/rclcpp.hpp>


#include <eigen3/Eigen/Dense>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <std_msgs/msg/header.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/point_cloud2.h>
// #include <sensor_msgs/image_encodings.h>
#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/path.h>
#include <visualization_msgs/msg/marker.h>
#include <visualization_msgs/msg/marker_array.h>


#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

// #include <tf/LinearMath/Quaternion.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>    
#include <tf2_ros/transform_listener.h>   
#include <tf2_ros/transform_broadcaster.h>  
#include <tf2/transform_datatypes.h>   
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 

 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <cassert>

#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "ThirdParty/DBoW/TemplatedDatabase.h"
#include "ThirdParty/DBoW/TemplatedVocabulary.h"

#include "../visual_feature/camera_models/CameraFactory.h"
#include "../visual_feature/camera_models/CataCamera.h"
#include "../visual_feature/camera_models/PinholeCamera.h"

using namespace std;

extern camodocal::CameraPtr m_camera;

extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;

extern string PROJECT_NAME;
extern string IMAGE_TOPIC;

extern int DEBUG_IMAGE;
extern int LOOP_CLOSURE;
extern double MATCH_IMAGE_SCALE;



class BriefExtractor
{
public:

    DVision::BRIEF m_brief;

    virtual void operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<DVision::BRIEF::bitset> &descriptors) const
    {
        m_brief.compute(im, keys, descriptors);
    }

    BriefExtractor(){};

    BriefExtractor(const std::string &pattern_file)
    {
        // The DVision::BRIEF extractor computes a random pattern by default when
        // the object is created.
        // We load the pattern that we used to build the vocabulary, to make
        // the descriptors compatible with the predefined vocabulary

        // loads the pattern
        cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
        if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

        vector<int> x1, y1, x2, y2;
        fs["x1"] >> x1;
        fs["x2"] >> x2;
        fs["y1"] >> y1;
        fs["y2"] >> y2;

        m_brief.importPairs(x1, y1, x2, y2);
    }
};

extern BriefExtractor briefExtractor;