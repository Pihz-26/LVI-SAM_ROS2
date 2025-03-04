#include "parameters.h"

#include "loop_detection.h"
#include "ament_index_cpp/get_package_share_directory.hpp"





double SKIP_TIME = 0;
double SKIP_DIST = 0;

camodocal::CameraPtr m_camera;



std::string PROJECT_NAME;
std::string IMAGE_TOPIC;

int DEBUG_IMAGE;
int LOOP_CLOSURE;
double MATCH_IMAGE_SCALE;


BriefExtractor briefExtractor;






int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoopDetector>("vins_loop");
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Visual Loop Started.\033[0m");

    // Load params
    std::string config_file;
    node->declare_parameter("vins_config_file", "");
    node->get_parameter("vins_config_file", config_file);


    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    usleep(100);

    // Initialize global params
    fsSettings["project_name"] >> PROJECT_NAME;  
    fsSettings["image_topic"]  >> IMAGE_TOPIC;  
    fsSettings["loop_closure"] >> LOOP_CLOSURE;
    fsSettings["skip_time"]    >> SKIP_TIME;
    fsSettings["skip_dist"]    >> SKIP_DIST;
    fsSettings["debug_image"]  >> DEBUG_IMAGE;
    fsSettings["match_image_scale"] >> MATCH_IMAGE_SCALE;
 

    
    if (LOOP_CLOSURE)
    {
        string pkg_path = ament_index_cpp::get_package_share_directory(PROJECT_NAME);

        // initialize vocabulary
        string vocabulary_file;
        fsSettings["vocabulary_file"] >> vocabulary_file;  
        vocabulary_file = pkg_path + vocabulary_file;
        node->loadVocabulary(vocabulary_file);

        // initialize brief extractor
        string brief_pattern_file;
        fsSettings["brief_pattern_file"] >> brief_pattern_file;  
        brief_pattern_file = pkg_path + brief_pattern_file;
        briefExtractor = BriefExtractor(brief_pattern_file);

        // initialize camera model
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());
    }


    if (!LOOP_CLOSURE)
    {
        rclcpp::shutdown();
    }
    else
    {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }

    
    

    // 关闭 ROS2
    rclcpp::shutdown();


    return 0;
}