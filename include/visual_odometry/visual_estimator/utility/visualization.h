#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/image.h>
// #include <sensor_msgs/msg/image_encodings.h>
#include <nav_msgs/msg/path.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <visualization_msgs/msg/marker.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2/convert.h>
// #include <tf2/LinearMath/Quaternion.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "estimator.h"
#include "parameters.h"
#include <fstream>


geometry_msgs::msg::Transform transformConversion(const geometry_msgs::msg::TransformStamped::SharedPtr t);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, 
    const Eigen::Vector3d &V, std_msgs::msg::Header &header, const int &failureId, 
    const Eigen::Vector3d &t_ic, const Eigen::Quaterniond &q_ic, Estimator* node_ptr);

void printStatistics(Estimator *estimator, double t);

void pubOdometry(Estimator *estimator, std_msgs::msg::Header &header);

void pubInitialGuess(Estimator *estimator, std_msgs::msg::Header &header);

void pubKeyPoses(Estimator *estimator, std_msgs::msg::Header &header);

void pubCameraPose(Estimator *estimator, std_msgs::msg::Header &header);

void pubPointCloud(Estimator *estimator, std_msgs::msg::Header &header);

void pubTF(Estimator *estimator, std_msgs::msg::Header &header);

void pubKeyframe(Estimator *estimator);

void pubRelocalization(Estimator *estimator);