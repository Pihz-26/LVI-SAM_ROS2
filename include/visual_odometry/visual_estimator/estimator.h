#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility.h"
#include "tic_toc.h"
#include "solve_5pts.h"
#include "initial_sfm.h"
#include "initial_alignment.h"
#include "initial_ex_rotation.h"
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/float32.h>
#include "sensor_msgs/msg/imu.hpp"
#include <std_msgs/msg/bool.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "utility/CameraPoseVisualization.h"

#include <ceres/ceres.h>
#include "imu_factor.h"
#include "pose_local_parameterization.h"
#include "projection_factor.h"
#include "projection_td_factor.h"
#include "marginalization_factor.h"
#include "rclcpp/rclcpp.hpp"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>


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


class Estimator: public rclcpp::Node
{
  public:
    // 接收
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         sub_imu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_image;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           sub_restart;

    // 发布
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pub_odometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pub_latest_odometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pub_latest_odometry_ros;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  pub_path;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_point_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_margin_cloud;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr      pub_key_poses;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pub_camera_pose;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_camera_pose_visual;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pub_keyframe_pose;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        pub_keyframe_point;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pub_extrinsic;
    
    CameraPoseVisualization cameraposevisual{0.0, 1.0, 0.0, 1.0};
    CameraPoseVisualization keyframebasevisual{0.0, 0.0, 1.0, 1.0};
    static double sum_of_path;
    static Vector3d last_path;

    Estimator(std::string node_name);

    void setParameter();

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, 
                             const vector<float> &lidar_initialization_info,
                             std_msgs::msg::Header &header);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    // void visualInitialAlignWithDepth();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();

    

    // 回调函数
    void imu_callback    (const sensor_msgs::msg::Imu::SharedPtr         imu_msg);
    void odom_callback   (const nav_msgs::msg::Odometry::SharedPtr       odom_msg);
    void feature_callback(const sensor_msgs::msg::PointCloud2::SharedPtr feature_msg);
    void restart_callback(const std_msgs::msg::Bool::SharedPtr           restart_msg);

    // 处理
    void process();
    void update();

    void predict(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud2::SharedPtr>> getMeasurements();

    
    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::msg::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager feature_manager;
    MotionEstimator motion_estimator;
    InitialEXRotation initial_ex_rotation{shared_from_this()};

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    int failureCount;

    // 缓存区
    std::condition_variable con;
    double current_time = -1;
    queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
    queue<sensor_msgs::msg::PointCloud2::SharedPtr> feature_buf;

    // tf缓冲区
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // tf 变换矩阵监听
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // tf board
    tf2_ros::TransformBroadcaster tf_broad_;

    // saving the lidar odometry 存储雷达里程计
    deque<nav_msgs::msg::Odometry> odomQueue;
    odometryRegister *odomRegister;

    std::mutex m_buf;
    std::mutex m_state;
    std::mutex m_estimator;
    std::mutex m_odom;

    double latest_time;
    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    // Eigen::Vector3d acc_0;
    // Eigen::Vector3d gyr_0;
    bool init_feature = 0;
    bool init_imu = 1;
    double last_imu_t = 0;

};
