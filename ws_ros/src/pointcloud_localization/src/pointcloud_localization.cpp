// pointcloud_localization.cpp
#include "pointcloud_localization/pointcloud_localization.hpp"

namespace pointcloud_localization {

PointCloudLocalization::PointCloudLocalization(const rclcpp::NodeOptions& options)
    : Node("pointcloud_localization_node", options) {
  

    // Set up the transform broadcaster
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  // 初始化参数
  declareParameters();

  // 加载地图点云并预处理
  std::string pcd_path = this->get_parameter("pcd_path").as_string();
  loadMap(pcd_path);

  // 初始化定位算法
  coarse_localizer_ = std::make_unique<NdtLocalizer>(
      this->get_parameter("ndt_resolution").as_double(),
      this->get_parameter("ndt_max_iterations").as_int(),
      this->get_parameter("ndt_transformation_epsilon").as_double(),
      this->get_parameter("ndt_step_size").as_double()
  );
  fine_localizer_ = std::make_unique<IcpLocalizer>(
      this->get_parameter("icp_max_iterations").as_int(),
      this->get_parameter("icp_max_distance").as_double(),
      this->get_parameter("icp_transformation_epsilon").as_double(),
      this->get_parameter("icp_step_size").as_double()
  );

  // 设置目标地图
  coarse_localizer_->setTargetMap(map_cloud_);
  fine_localizer_->setTargetMap(map_cloud_);

  // 订阅点云和初始位姿
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->get_parameter("input_pointcloud_topic").as_string(),
      10,
      std::bind(&PointCloudLocalization::pointcloudCallback, this, std::placeholders::_1)
  );
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",
      10,
      std::bind(&PointCloudLocalization::initialPoseCallback, this, std::placeholders::_1)
  );


  RCLCPP_INFO(this->get_logger(), "Node initialized with NDT+ICP hybrid mode");
}

PointCloudLocalization::~PointCloudLocalization() {
  RCLCPP_INFO(this->get_logger(), "\033[1;32m----> Initial_localization_node END - Shutting down cleanly \033[0m");
}

// 参数声明
void PointCloudLocalization::declareParameters() {
  this->declare_parameter<std::string>("pcd_path", "");
  this->declare_parameter<std::string>("input_pointcloud_topic", "/livox/lidar");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("odom_frame", "init");
  this->declare_parameter<std::string>("lidar_frame", "laser");
  
  // NDT参数
  this->declare_parameter<double>("ndt_resolution", 0.5);
  this->declare_parameter<int>("ndt_max_iterations", 5000);
  this->declare_parameter<double>("ndt_transformation_epsilon", 1e-8);
  this->declare_parameter<double>("ndt_step_size", 0.05);
  
  // ICP参数
  this->declare_parameter<int>("icp_max_iterations", 5000);
  this->declare_parameter<double>("icp_max_distance", 0.3);
  this->declare_parameter<double>("icp_transformation_epsilon", 1e-8);
  this->declare_parameter<double>("icp_euclideanFitness_epsilon", 1e-8);
  this->declare_parameter<double>("icp_step_size", 0.05);
  
  // 预处理参数
  this->declare_parameter<double>("voxel_leaf_size", 0.1);
}

// 加载地图点云
void PointCloudLocalization::loadMap(const std::string& pcd_path) {
  if (pcd_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Empty PCD path!");
    return;
  }

  PointCloudT::Ptr raw_cloud(new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>(pcd_path, *raw_cloud) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_path.c_str());
    return;
  }

  // 降采样和计算法向量
  map_cloud_ = preprocessCloud(raw_cloud);
  RCLCPP_INFO(this->get_logger(), "Loaded map with %ld points", map_cloud_->size());
}

// 点云预处理
PointCloudNormalT::Ptr PointCloudLocalization::preprocessCloud(const PointCloudT::Ptr& cloud) {
  // 降采样
  PointCloudT::Ptr filtered(new PointCloudT);
  voxel_filter_.setLeafSize(
      this->get_parameter("voxel_leaf_size").as_double(),
      this->get_parameter("voxel_leaf_size").as_double(),
      this->get_parameter("voxel_leaf_size").as_double()
  );
  voxel_filter_.setInputCloud(cloud);
  voxel_filter_.filter(*filtered);

  // 计算法向量
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  normal_estimator_.setSearchMethod(tree);
  normal_estimator_.setKSearch(20);
  normal_estimator_.setInputCloud(filtered);

  PointCloudNormalT::Ptr cloud_with_normals(new PointCloudNormalT);
  normal_estimator_.compute(*cloud_with_normals);
  pcl::copyPointCloud(*filtered, *cloud_with_normals);

  return cloud_with_normals;
}

// 点云回调
void PointCloudLocalization::pointcloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg) {
  // RCLCPP_INFO(this->get_logger(), "Received Deskewed PointClouds");
  std::lock_guard<std::mutex> lock(mutex_);
  pcl::fromROSMsg(*msg, *current_cloud_);
  // RCLCPP_INFO(this->get_logger(), "Finished Received Deskewed PointClouds");
  
}

// 初始位姿回调
void PointCloudLocalization::initialPoseCallback(
    const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> msg) {
  RCLCPP_INFO(this->get_logger(), "Received Estimate Pose");

  Eigen::Vector3f pos(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z
  );
  Eigen::Quaternionf q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z
  );

  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  init_guess.block<3, 3>(0, 0) = q.toRotationMatrix();
  init_guess.block<3, 1>(0, 3) = pos;

  // 执行定位
  std::lock_guard<std::mutex> lock(mutex_);
  PointCloudNormalT::Ptr processed_cloud = preprocessCloud(current_cloud_);

  Eigen::Matrix4f result;
  if (localize(processed_cloud, init_guess, result)) {
    publishTransform(result);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Localization failed!");
  }
  
}

// 核心定位逻辑
bool PointCloudLocalization::localize(
    const PointCloudNormalT::Ptr& cloud,
    const Eigen::Matrix4f& init_guess,
    Eigen::Matrix4f& result) {
  
  rclcpp::Time start_time = this->now();
  Eigen::Matrix4f intermediate_result;
  
  // Step 1: NDT粗定位
  if (!coarse_localizer_->align(cloud, init_guess, intermediate_result)) {
    RCLCPP_WARN(this->get_logger(), "NDT coarse alignment failed!");
    return false;
  }
  
  // Step 2: ICP精定位
  if (!fine_localizer_->align(cloud, intermediate_result, result)) {
    RCLCPP_WARN(this->get_logger(), "ICP fine alignment failed!");
    return false;
  }

  if (fine_localizer_->getFitnessScore() < 0.01) {
    RCLCPP_INFO(this->get_logger(), "Final fitness score: %.3f", fine_localizer_->getFitnessScore());
    RCLCPP_INFO(this->get_logger(), "Cost time: %.2fs", 
            (this->now() - start_time).seconds());
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "Final fitness score: %.3f, Please give a better Pose", fine_localizer_->getFitnessScore());
    RCLCPP_INFO(this->get_logger(), "Cost time: %.2fs", 
            (this->now() - start_time).seconds());
    return false;
  }
  

  
}

// 发布TF变换
void PointCloudLocalization::publishTransform(const Eigen::Matrix4f& transform) {
  
  map_to_odom_.header.stamp = this->now();
  map_to_odom_.header.frame_id = this->get_parameter("map_frame").as_string();
  map_to_odom_.child_frame_id = this->get_parameter("odom_frame").as_string();
  
  map_to_odom_.transform.translation.x = transform(0, 3);
  map_to_odom_.transform.translation.y = transform(1, 3);
  map_to_odom_.transform.translation.z = transform(2, 3);
  
  Eigen::Matrix3f rot = transform.block<3, 3>(0, 0);
  Eigen::Quaternionf q(rot);
  map_to_odom_.transform.rotation.w = q.w();
  map_to_odom_.transform.rotation.x = q.x();
  map_to_odom_.transform.rotation.y = q.y();
  map_to_odom_.transform.rotation.z = q.z();
  static_tf_broadcaster_->sendTransform(map_to_odom_);
}




}


int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto PL = std::make_shared<pointcloud_localization::PointCloudLocalization>(options);
    exec.add_node(PL);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> PointCloud Localization Started.\033[0m");



    exec.spin();

    rclcpp::shutdown();

    return 0;
}

