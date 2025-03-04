#include "utility.hpp"
#include "nav_msgs/msg/path.hpp" 

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class IMUPreintegration : public ParamServer
{
public:

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

    // map -> odom
    tf2::Transform map_to_odom;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfMap2Odom;
    // odom -> base_link
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfOdom2BaseLink;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::msg::Imu> imuQueOpt;
    std::deque<sensor_msgs::msg::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;
    int imuPreintegrationResetId = 0;

#if IF_OFFICIAL
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));
#else
    //? mod: 坐标系定义这里还是有一个细节问题
    //; T_imulidar_lidar，其中旋转一定是单位帧，因为这里的IMU是已经转换到和LiDAR坐标轴xyz完全平行的形式了，所以只剩下旋转了。
    //; 定义旋转到和LiDAR坐标轴完全平行、但是坐标系原点不变的IMU坐标系为imulidar, 那么这里要的其实是T_imulidar_lidar
    //; 而配置文件中我们给的extTrans是T_imu_lidar的平移部分，即t_imu_lidar，所以这里还需要转换一步。
    //; T_imu_imulidar = [R_imu_lidar, 0; 0 1], T_imu_lidar = [R_imu_lidar, t_imu_lidar; 0, 1]
    //; T_imulidar_lidar = [I, R_lidar_imu*t_imu_lidar; 0, 1]
    Eigen::Vector3d t_imulidar_lidar = R_lidar_imu * t_imu_lidar;
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(t_imulidar_lidar.x(), t_imulidar_lidar.y(), t_imulidar_lidar.z()));
    //; T_lidar_imulidar, 同上
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-t_imulidar_lidar.x(), -t_imulidar_lidar.y(), -t_imulidar_lidar.z()));
#endif


    IMUPreintegration(std::string node_name);

    void resetOptimization();

    void resetParams();

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg);
};

