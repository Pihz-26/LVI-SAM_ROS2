// pointcloud_localization.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp> 
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <thread>

namespace pointcloud_localization {

// 定义点云类型 (兼容ICP和NDT)
using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using PointNormalT = pcl::PointNormal;
using PointCloudNormalT = pcl::PointCloud<PointNormalT>;

// 定位算法枚举
enum class LocalizationMethod { ICP, NDT, HYBRID };

/**
 * @brief 基础定位算法接口类
 */
class BaseLocalizer {
public:
  virtual ~BaseLocalizer() = default;
  
  // 设置目标地图点云
  virtual void setTargetMap(const PointCloudNormalT::Ptr& map_cloud) = 0;
  
  // 执行配准
  virtual bool align(const PointCloudNormalT::Ptr& source_cloud, 
                    const Eigen::Matrix4f& init_guess,
                    Eigen::Matrix4f& result) = 0;
  
  // 获取最后一次匹配得分
  virtual float getFitnessScore()  = 0;
};

/**
 * @brief ICP 定位实现
 */
class IcpLocalizer : public BaseLocalizer {
public:
  IcpLocalizer(int max_iterations = 10, float max_correspondence_distance = 0.1, double transformation_epsilon=1e-6, double euclideanFitness_epsilon=1e-6)
    : icp_()
  {
    icp_.setMaximumIterations(max_iterations);
    icp_.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp_.setTransformationEpsilon(transformation_epsilon);
    icp_.setEuclideanFitnessEpsilon(euclideanFitness_epsilon); 
  }

  void setTargetMap(const PointCloudNormalT::Ptr& map_cloud) override {
    icp_.setInputTarget(map_cloud);
  }

  bool align(const PointCloudNormalT::Ptr& source_cloud,
            const Eigen::Matrix4f& init_guess,
            Eigen::Matrix4f& result) override {
    icp_.setInputSource(source_cloud);
    PointCloudNormalT aligned_cloud;
    icp_.align(aligned_cloud, init_guess);
    result = icp_.getFinalTransformation();
    return icp_.hasConverged();
  }

  float getFitnessScore() override{
    return icp_.getFitnessScore();
  }

private:
  pcl::IterativeClosestPoint<PointNormalT, PointNormalT> icp_;
};

/**
 * @brief NDT 定位实现
 */
class NdtLocalizer :public BaseLocalizer {
public:
  NdtLocalizer(float resolution = 1.0, int max_iterations = 30, double transformation_epsilon=1e-6, double step_size=0.1)
    : ndt_()
  {
    ndt_.setTransformationEpsilon(transformation_epsilon);
    ndt_.setStepSize(step_size);
    ndt_.setResolution(resolution);
    ndt_.setMaximumIterations(max_iterations);
  }

  void setTargetMap(const PointCloudNormalT::Ptr& map_cloud) override {
    ndt_.setInputTarget(map_cloud);
  }

  bool align(const PointCloudNormalT::Ptr& source_cloud,
            const Eigen::Matrix4f& init_guess,
            Eigen::Matrix4f& result) override {
    ndt_.setInputSource(source_cloud);
    PointCloudNormalT aligned_cloud;
    ndt_.align(aligned_cloud, init_guess);
    result = ndt_.getFinalTransformation();
    return ndt_.hasConverged();
  }

  float getFitnessScore()  override{
    return ndt_.getFitnessScore();
  }

private:
  pcl::NormalDistributionsTransform<PointNormalT, PointNormalT> ndt_;
};

/**
 * @brief ROS2 节点主类
 */
class PointCloudLocalization : public rclcpp::Node {
public:
  explicit PointCloudLocalization(const rclcpp::NodeOptions& options);
  ~PointCloudLocalization();

private:
  // 参数配置
  void declareParameters();
  void loadMap(const std::string& pcd_path);
  PointCloudNormalT::Ptr preprocessCloud(const PointCloudT::Ptr& cloud);
  
  // 回调函数
  void pointcloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg);
  void initialPoseCallback(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> msg);
  
  // 定位核心逻辑
  bool localize(const PointCloudNormalT::Ptr& cloud, const Eigen::Matrix4f& init_guess, Eigen::Matrix4f& result);
  void publishTransform(const Eigen::Matrix4f& transform);

  // ROS2 相关成员
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::TransformStamped map_to_odom_; 
  
  // PCL 相关成员
  PointCloudNormalT::Ptr map_cloud_;
  PointCloudT::Ptr current_cloud_ = std::make_shared<PointCloudT>();
  pcl::VoxelGrid<PointT> voxel_filter_;
  pcl::NormalEstimation<PointT, PointNormalT> normal_estimator_;
  
  // 定位算法
  std::unique_ptr<BaseLocalizer> coarse_localizer_;  // 粗定位（NDT）
  std::unique_ptr<BaseLocalizer> fine_localizer_;    // 精定位（ICP）
  LocalizationMethod method_ = LocalizationMethod::HYBRID;
  
  // 状态控制
  std::mutex mutex_;
  bool is_ready_ = false;
  bool first_scan_ = true;
  
  // 坐标系参数
  std::string map_frame_;
  std::string odom_frame_;
  std::string lidar_frame_;
};

} // namespace pointcloud_localization
