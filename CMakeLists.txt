cmake_minimum_required(VERSION 3.5)
project(lvi_sam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


# msg

find_package(std_msgs         REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(nav_msgs         REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(cv_bridge        REQUIRED)
find_package(pcl_conversions  REQUIRED)
find_package(rosidl_default_generators  REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# function dependencies
find_package(tf2    REQUIRED) 
find_package(tf2_ros    REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL    REQUIRED)
find_package(Ceres  REQUIRED)
find_package(GTSAM  REQUIRED)
find_package(Boost  REQUIRED)
find_package(libusb-1.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CloudInfo.msg
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()





# ========================================
# 雷达部分节点定义链接与实现
# ========================================

# imageProjection 节点实现
add_executable(image_projection src/lidar_odometry/utility.cpp src/lidar_odometry/imageProjection.cpp)
rosidl_target_interfaces(image_projection ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(image_projection  ${rclcpp_LIBRARIES}
                                        ${OpenCV_LIBRARIES}
                                        ${cv_bridge_LIBRARIES}
                                        ${PCL_LIBRARIES}
                                        ${PCL_CONVERSION_LIBRARIES}
                                        ${tf2_LIBRARIES}
                                        ${tf2_ros_LIBRARIES}
                                        ${sensor_msgs_LIBRARIES} 
                                        ${nav_msgs_LIBRARIES}
                                        gtsam)

target_include_directories(image_projection PUBLIC include/lidar_odometry)
ament_target_dependencies(image_projection
                          rclcpp 
                          sensor_msgs 
                          nav_msgs
                          geometry_msgs
                          message_filters 
                          tf2 
                          tf2_ros
                          PCL 
                          pcl_conversions
                          cv_bridge 
                          )
install(TARGETS
image_projection
DESTINATION lib/${PROJECT_NAME}
)

# featureExtraction 节点实现
add_executable(feature_extraction src/lidar_odometry/utility.cpp src/lidar_odometry/featureExtraction.cpp)
rosidl_target_interfaces(feature_extraction ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(feature_extraction  ${rclcpp_LIBRARIES}
                                          ${OpenCV_LIBRARIES}
                                          ${cv_bridge_LIBRARIES}
                                          ${PCL_LIBRARIES}
                                          ${PCL_CONVERSION_LIBRARIES}
                                          ${tf2_LIBRARIES}
                                          ${tf2_ros_LIBRARIES}
                                          ${sensor_msgs_LIBRARIES} 
                                          ${nav_msgs_LIBRARIES}
                                          ${geometry_msgs_LIBRARIES}
                                          gtsam)
target_include_directories(feature_extraction PUBLIC include/lidar_odometry)
ament_target_dependencies(feature_extraction
                          rclcpp 
                          sensor_msgs 
                          nav_msgs
                          geometry_msgs
                          message_filters 
                          tf2 
                          tf2_ros
                          PCL 
                          pcl_conversions
                          cv_bridge 
                          )
install(TARGETS
feature_extraction
DESTINATION lib/${PROJECT_NAME}
)


# IMUPreintegration 节点实现
add_executable(imu_preintegration src/lidar_odometry/utility.cpp src/lidar_odometry/imuPreintegration.cpp)
rosidl_target_interfaces(imu_preintegration ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(imu_preintegration  ${rclcpp_LIBRARIES}
                                          ${OpenCV_LIBRARIES}
                                          ${cv_bridge_LIBRARIES}
                                          ${PCL_LIBRARIES}
                                          ${PCL_CONVERSION_LIBRARIES}
                                          ${tf2_LIBRARIES}
                                          ${tf2_ros_LIBRARIES}
                                          ${sensor_msgs_LIBRARIES} 
                                          ${nav_msgs_LIBRARIES}
                                          gtsam
                                          )
target_include_directories(imu_preintegration PUBLIC include/lidar_odometry ${GTSAM_INCLUDE_DIR})
ament_target_dependencies(imu_preintegration
                          rclcpp 
                          sensor_msgs 
                          nav_msgs
                          geometry_msgs
                          message_filters 
                          tf2 
                          tf2_ros
                          PCL 
                          pcl_conversions
                          cv_bridge 
                          )
install(TARGETS
imu_preintegration
DESTINATION lib/${PROJECT_NAME}
)

# mapOptmization节点实现
add_executable(map_optmization src/lidar_odometry/utility.cpp src/lidar_odometry/mapOptmization.cpp)
rosidl_target_interfaces(map_optmization ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(map_optmization  ${rclcpp_LIBRARIES}
                                          ${OpenCV_LIBRARIES}
                                          ${cv_bridge_LIBRARIES}
                                          ${PCL_LIBRARIES}
                                          ${PCL_CONVERSION_LIBRARIES}
                                          ${tf2_LIBRARIES}
                                          ${tf2_ros_LIBRARIES}
                                          ${sensor_msgs_LIBRARIES} 
                                          ${nav_msgs_LIBRARIES}
                                          ${visualization_msgs_LIBRARIES}
                                          ${LIBUSB_1_LIBRARIES}
                                          gtsam
                                          
                                          )
target_include_directories(map_optmization PUBLIC include/lidar_odometry ${GTSAM_INCLUDE_DIR})
ament_target_dependencies(map_optmization
                          rclcpp 
                          sensor_msgs 
                          nav_msgs
                          geometry_msgs
                          message_filters 
                          tf2 
                          tf2_ros
                          PCL 
                          pcl_conversions
                          cv_bridge 
                          )
install(TARGETS
map_optmization
DESTINATION lib/${PROJECT_NAME}
)




# ========================================
# 相机部分节点定义链接与实现
# ========================================

# visual_feature部分实现


file(GLOB camera_model_cpp src/visual_odometry/visual_feature/camera_models/*.cc)
add_library(camera_model SHARED ${camera_model_cpp})
target_include_directories(camera_model PUBLIC include/visual_odometry/visual_feature/camera_models ${glog_INCLUDE_DIRS})
target_link_libraries(camera_model ${OpenCV_LIBRARIES}
                                   ${Eigen3_LIBRARY}
                                   ${glog_LIBRARIES}
                                   glog::glog
                                   )

add_executable(visual_feature src/visual_odometry/visual_feature/feature_tracker_node.cpp
                              src/visual_odometry/visual_feature/feature_tracker.cpp
                              src/visual_odometry/visual_feature/parameters.cpp)

rosidl_target_interfaces(visual_feature ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(visual_feature  ${rclcpp_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      ${cv_bridge_LIBRARIES}
                                      ${PCL_LIBRARIES}
                                      ${PCL_CONVERSION_LIBRARIES}
                                      ${tf2_LIBRARIES}
                                      ${tf2_ros_LIBRARIES}
                                      ${sensor_msgs_LIBRARIES} 
                                      ${nav_msgs_LIBRARIES}
                                      ${glog_LIBRARIES}
                                      camera_model 
                                      )

target_include_directories(visual_feature PUBLIC include/visual_odometry/visual_feature include/visual_odometry/visual_feature/camera_models ${glog_INCLUDE_DIRS})


ament_target_dependencies(visual_feature
                          rclcpp 
                          sensor_msgs 
                          nav_msgs
                          geometry_msgs
                          message_filters 
                          tf2 
                          tf2_ros
                          PCL 
                          pcl_conversions
                          cv_bridge 
                          )
install(TARGETS
visual_feature
DESTINATION lib/${PROJECT_NAME}
)


# visual_estimator部分实现
# 获取所有.cpp文件
file(GLOB visual_estimator_factor_cpp src/visual_odometry/visual_estimator/factor/*.cpp)
file(GLOB visual_estimator_initial_cpp src/visual_odometry/visual_estimator/initial/*.cpp)
file(GLOB visual_estimator_utility_cpp src/visual_odometry/visual_estimator/utility/*.cpp)
file(GLOB visual_estimator_cpp src/visual_odometry/visual_estimator/*.cpp)

add_executable(visual_estimator ${visual_estimator_factor_cpp} ${visual_estimator_initial_cpp} ${visual_estimator_utility_cpp} ${visual_estimator_cpp})

rosidl_target_interfaces(visual_estimator ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(visual_estimator ${rclcpp_LIBRARIES}
                                       ${OpenCV_LIBRARIES}
                                       ${cv_bridge_LIBRARIES}
                                       ${PCL_LIBRARIES}
                                       ${PCL_CONVERSION_LIBRARIES}
                                       ${tf2_LIBRARIES}
                                       ${tf2_ros_LIBRARIES}
                                       ${sensor_msgs_LIBRARIES} 
                                       ${nav_msgs_LIBRARIES}
                                       ${visualization_msgs_LIBRARIES}
                                       ${tf2_geometry_msgs_LIBRARIES}
                                       ${CERES_LIBRARIES})
                                       
                                       

target_include_directories(visual_estimator 
                            PUBLIC 
                            include/visual_odometry/visual_estimator/factor 
                            include/visual_odometry/visual_estimator/initial 
                            include/visual_odometry/visual_estimator/utility 
                            include/visual_odometry/visual_estimator)


ament_target_dependencies(visual_estimator
                          rclcpp 
                          sensor_msgs 
                          nav_msgs
                          geometry_msgs
                          message_filters 
                          tf2 
                          tf2_ros
                          PCL 
                          pcl_conversions
                          cv_bridge 
                          )
install(TARGETS
visual_estimator
DESTINATION lib/${PROJECT_NAME}
)


# visual_loop部分实现
# 获取所有.cpp文件
file(GLOB visual_estimator_dbow_cpp     src/visual_odometry/visual_loop/ThirdParty/DBoW/*.cpp)
file(GLOB visual_estimator_dutils_cpp   src/visual_odometry/visual_loop/ThirdParty/DUtils/*.cpp)
file(GLOB visual_estimator_dvision_cpp  src/visual_odometry/visual_loop/ThirdParty/DVision/*.cpp)
file(GLOB visual_loop_third_cpp         src/visual_odometry/visual_loop/ThirdParty/*.cpp)
file(GLOB visual_loop_cpp               src/visual_odometry/visual_loop/*.cpp)

add_executable(visual_loop ${visual_estimator_dbow_cpp} ${visual_estimator_dutils_cpp} ${visual_estimator_dvision_cpp} ${visual_loop_third_cpp} ${visual_loop_cpp} )

rosidl_target_interfaces(visual_loop ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(visual_loop      ${rclcpp_LIBRARIES}
                                       ${OpenCV_LIBRARIES}
                                       ${cv_bridge_LIBRARIES}
                                       ${PCL_LIBRARIES}
                                       ${PCL_CONVERSION_LIBRARIES}
                                       ${tf2_LIBRARIES}
                                       ${tf2_ros_LIBRARIES}
                                       ${sensor_msgs_LIBRARIES} 
                                       ${nav_msgs_LIBRARIES}
                                       ${visualization_msgs_LIBRARIES}
                                       ${tf2_geometry_msgs_LIBRARIES}
                                       ${CERES_LIBRARIES}
                                       camera_model)
                                       
                                       

target_include_directories( visual_loop 
                            PUBLIC 
                            include/visual_odometry/visual_loop/ThirdParty/DBoW 
                            include/visual_odometry/visual_loop/ThirdParty/DUtils 
                            include/visual_odometry/visual_loop/ThirdParty/DVision 
                            include/visual_odometry/visual_loop/ThirdParty
                            include/visual_odometry/visual_loop
                            include/visual_odometry/visual_feature/camera_models)


ament_target_dependencies(visual_loop
                          rclcpp 
                          sensor_msgs 
                          nav_msgs
                          geometry_msgs
                          message_filters 
                          tf2 
                          tf2_ros
                          PCL 
                          pcl_conversions
                          cv_bridge 
                          )
install(TARGETS
visual_loop
DESTINATION lib/${PROJECT_NAME}
)