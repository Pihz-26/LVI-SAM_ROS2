#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

// Estimator estimator;








int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Estimator>("vins_loop");
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Estimator Started.\033[0m");
    

    readParameters(node);
    node->setParameter();


    #if IF_OFFICIAL
        odomRegister = new odometryRegister(n);
    #else
        Eigen::Vector3d t_lidar_imu = -R_imu_lidar.transpose() * t_imu_lidar;
        node->odomRegister = new odometryRegister(node, R_imu_lidar.transpose(), t_lidar_imu);
    #endif


    // if (!USE_LIDAR)
    //     sub_odom.shutdown();

    // 多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    // 关闭 ROS2
    rclcpp::shutdown();
    

    return 0;
}