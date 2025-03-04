#include "feature_tracker.h"


#define SHOW_UNDISTORTION 0






int main(int argc, char **argv)
{
    // initialize ROS node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthRegister>("vins_feature_tracker");
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Feature Tracker Started.\033[0m");

    
    // 多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    // 关闭 ROS2
    rclcpp::shutdown();
}

