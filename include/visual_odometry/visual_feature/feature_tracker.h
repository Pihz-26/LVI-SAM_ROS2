#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camera_models/CameraFactory.h"
#include "camera_models/CataCamera.h"
#include "camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp> 

using namespace std;
using namespace camodocal;
using namespace Eigen;


struct PointFeature
{
    PCL_ADD_POINT4D; 
    int id;
    float u;
    float v;
    float velocity_x;
    float velocity_y;
    float depth;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    //PCL宏，确保new操作符对其操作

    PointFeature() : id(0), u(0.0f), v(0.0f), velocity_x(0.0f), velocity_y(0.0f), depth(0.0f){}  // 构造函数，初始化成员变量
}EIGEN_ALIGN16;

// 注册点类型宏
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointFeature,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (int, id, id)
    (float, u, u)
    (float, v, v)
    (float, velocity_x, velocity_x)
    (float, velocity_y, velocity_y)
    (float, depth, depth)
)


bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);




class FeatureTracker
{
public:
    FeatureTracker(const std::shared_ptr<rclcpp::Node> n);

    void readImage(const cv::Mat &_img, double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;
    vector<int> ids;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;

    static int n_id;

    const std::shared_ptr<rclcpp::Node> node_ptr;
};


class DepthRegister: public rclcpp::Node
{
public:
    // publisher for visualization
    // ros::Publisher pub_depth_feature;
    // ros::Publisher pub_depth_image;
    // ros::Publisher pub_depth_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_feature;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       pub_depth_image;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_cloud;

    
    // 缓冲区
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // tf 变换矩阵获取
    std::shared_ptr<tf2_ros::TransformListener> listener;
    geometry_msgs::msg::TransformStamped transform;

    // feature publisher for VINS estimator
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_feature;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      pub_match;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          pub_restart;


    // subscriber to image and lidar
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar;

    // feature tracker variables
    // FeatureTracker trackerData[NUM_OF_CAM];
    std::vector<FeatureTracker> trackerData;
    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;

    // mtx lock for two threads
    std::mutex mtx_lidar;

    // global variable for saving the depthCloud shared between two threads
    pcl::PointCloud<PointType>::Ptr depthCloud;

    // global variables saving the lidar point cloud
    deque<pcl::PointCloud<PointType>> cloudQueue;
    deque<double> timeQueue;


    const int num_bins = 360;
    vector<vector<PointType>> pointsArray;


    DepthRegister(std::string node_name);


    // 两个回调函数
    void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg);


    void get_depth(const rclcpp::Time &stamp_cur, const cv::Mat &imageCur,
                                          const pcl::PointCloud<PointType>::Ptr &depthCloud,
                                          const camodocal::CameraPtr &camera_model,
                                          const pcl::PointCloud<PointFeature>::Ptr features_2d);

    void getColor(float p, float np, float &r, float &g, float &b)
    {
        float inc = 6.0 / np;
        float x = p * inc;
        r = 0.0f;
        g = 0.0f;
        b = 0.0f;
        if ((0 <= x && x <= 1) || (5 <= x && x <= 6))
            r = 1.0f;
        else if (4 <= x && x <= 5)
            r = x - 4;
        else if (1 <= x && x <= 2)
            r = 1.0f - (x - 1);

        if (1 <= x && x <= 3)
            g = 1.0f;
        else if (0 <= x && x <= 1)
            g = x - 0;
        else if (3 <= x && x <= 4)
            g = 1.0f - (x - 3);

        if (3 <= x && x <= 5)
            b = 1.0f;
        else if (2 <= x && x <= 3)
            b = x - 2;
        else if (5 <= x && x <= 6)
            b = 1.0f - (x - 5);
        r *= 255.0;
        g *= 255.0;
        b *= 255.0;
    }


};