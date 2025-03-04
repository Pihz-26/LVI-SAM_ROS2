#include "imuPreintegration.hpp"

using namespace std;

typedef pcl::PointXYZI PointType;

#if IF_OFFICIAL 
#else
    bool ParamServer::if_print_param = false;
#endif

ParamServer::ParamServer(std::string node_name) : Node(node_name)
{
    PROJECT_NAME    = this->declare_parameter<std::string>("/PROJECT_NAME", "sam");

    robot_id        = this->declare_parameter<std::string>("/robot_id", "roboat");

    pointCloudTopic = this->declare_parameter<std::string>(PROJECT_NAME + "/pointCloudTopic", "points_raw");
    imuTopic        = this->declare_parameter<std::string>(PROJECT_NAME + "/imuTopic", "imu_correct");
    odomTopic       = this->declare_parameter<std::string>(PROJECT_NAME + "/odomTopic", "odometry/imu");
    gpsTopic        = this->declare_parameter<std::string>(PROJECT_NAME + "/gpsTopic", "odometry/gps");

    useImuHeadingInitialization = this->declare_parameter<bool>(PROJECT_NAME + "/useImuHeadingInitialization", false);
    useGpsElevation             = this->declare_parameter<bool>(PROJECT_NAME + "/useGpsElevation", false);
    gpsCovThreshold             = this->declare_parameter<float>(PROJECT_NAME + "/gpsCovThreshold", 2.0);
    poseCovThreshold            = this->declare_parameter<float>(PROJECT_NAME + "/poseCovThreshold", 25.0);

    savePCD             = this->declare_parameter<bool>(PROJECT_NAME + "/savePCD", false);
    savePCDDirectory    = this->declare_parameter<std::string>(PROJECT_NAME + "/savePCDDirectory", "/tmp/loam/");

    N_SCAN          = this->declare_parameter<int>(PROJECT_NAME + "/N_SCAN", 16);
    Horizon_SCAN    = this->declare_parameter<int>(PROJECT_NAME + "/Horizon_SCAN", 1800);
    timeField       = this->declare_parameter<std::string>(PROJECT_NAME + "/timeField", "time");
    downsampleRate  = this->declare_parameter<int>(PROJECT_NAME + "/downsampleRate", 1);

    imuAccNoise = this->declare_parameter<float>(PROJECT_NAME + "/imuAccNoise", 0.01);
    imuGyrNoise = this->declare_parameter<float>(PROJECT_NAME + "/imuGyrNoise", 0.001);
    imuAccBiasN = this->declare_parameter<float>(PROJECT_NAME + "/imuAccBiasN", 0.0002);
    imuGyrBiasN = this->declare_parameter<float>(PROJECT_NAME + "/imuGyrBiasN", 0.00003);
    imuGravity  = this->declare_parameter<float>(PROJECT_NAME + "/imuGravity", 9.80511);

#if IF_OFFICIAL    
    extRotV     = this->declare_parameter<vector<double>>(PROJECT_NAME+ "/extrinsicRot", vector<double>());
    extRPYV     = this->declare_parameter<vector<double>>(PROJECT_NAME+ "/extrinsicRPY", vector<double>());
    extTransV   = this->declare_parameter<vector<double>>(PROJECT_NAME+ "/extrinsicTrans", vector<double>());
    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
    extQRPY = Eigen::Quaterniond(extRPY);
#else

    //? mod: 修改外参读取方式
    t_imu_lidar_V = this->declare_parameter<vector<double>>(PROJECT_NAME+ "/extrinsicTranslation",  vector<double>());
    R_imu_lidar_V = this->declare_parameter<vector<double>>(PROJECT_NAME+ "/extrinsicRotation",     vector<double>());
    t_imu_lidar = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(t_imu_lidar_V.data(), 3, 1);
    Eigen::Matrix3d R_tmp = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(R_imu_lidar_V.data(), 3, 3);
    RCUTILS_ASSERT(abs(R_tmp.determinant()) > 0.9);   // 防止配置文件中写错，这里加一个断言判断一下
    R_imu_lidar = Eigen::Quaterniond(R_tmp).normalized().toRotationMatrix();
    R_lidar_imu = R_imu_lidar.transpose();

    //; yaw/pitch/roll的欧拉角绕着哪个轴逆时针旋转，结果为正数。一般来说是绕着+z、+y、+x
    std::string yaw_axis, pitch_axis, roll_axis;   
    yaw_axis   = this->declare_parameter<std::string>(PROJECT_NAME + "/yawAxis", "+z");
    pitch_axis = this->declare_parameter<std::string>(PROJECT_NAME + "/pitchAxis",  "+y");
    roll_axis  = this->declare_parameter<std::string>(PROJECT_NAME + "/rollAxis",  "+x");
    RCUTILS_ASSERT(yaw_axis[0] == '+' || yaw_axis[0] == '-');
    RCUTILS_ASSERT(pitch_axis[0] == '+' || pitch_axis[0] == '-');
    RCUTILS_ASSERT(roll_axis[0] == '+' || roll_axis[0] == '-');
    RCUTILS_ASSERT(yaw_axis[1] != pitch_axis[1] && yaw_axis[1] != roll_axis[1] && pitch_axis[1] != roll_axis[1]);

    //; 旋转的欧拉角坐标系(quat) -> IMU角速度、加速度坐标系(imu) 的旋转
    Eigen::Matrix3d R_imu_quat;   
    std::unordered_map<std::string, Eigen::Vector3d> col_map;
    col_map.insert({"+x", Eigen::Vector3d( 1,  0,  0)}); 
    col_map.insert({"-x", Eigen::Vector3d(-1,  0,  0)});
    col_map.insert({"+y", Eigen::Vector3d( 0,  1,  0)}); 
    col_map.insert({"-y", Eigen::Vector3d( 0, -1,  0)});
    col_map.insert({"+z", Eigen::Vector3d( 0,  0,  1)}); 
    col_map.insert({"-z", Eigen::Vector3d( 0,  0, -1)});
    R_imu_quat.col(2) = col_map[yaw_axis];
    R_imu_quat.col(1) = col_map[pitch_axis];
    R_imu_quat.col(0) = col_map[roll_axis];
    RCUTILS_ASSERT(abs(R_imu_quat.determinant()) > 0.9);  

    //; R_quat_lidar = R_quat_imu * R_imu_lidar
    Eigen::Matrix3d R_quat_lidar = R_imu_quat.transpose() * R_imu_lidar;  
    Q_quat_lidar = Eigen::Quaterniond(R_quat_lidar).normalized();

    if(if_print_param)
    {
        if_print_param = false;
        RCLCPP_WARN_STREAM(this->get_logger(), "=== R_imu_lidar : ===============");
        std::cout << R_imu_lidar << std::endl;
        RCLCPP_WARN_STREAM(this->get_logger(), "=== t_imu_lidar : ===============");
        std::cout << t_imu_lidar << std::endl;

        RCLCPP_WARN_STREAM(this->get_logger(), "=== R_imu_quat  : ===============");
        std::cout << "yawAxis = " << yaw_axis << ", col_map: " << col_map[yaw_axis].transpose()
            << ", pitchAxis = " << pitch_axis << ", col_map: " << col_map[pitch_axis].transpose()
            << ", rollAxis = " << roll_axis << ", col_map: " << col_map[roll_axis].transpose()
            << std::endl;
        std::cout << R_imu_quat << std::endl;

        RCLCPP_WARN_STREAM(this->get_logger(), "=== R_quat_lidar  : ===============");
        std::cout << R_quat_lidar << std::endl;
    }
#endif


    edgeThreshold           = this->declare_parameter<float>(PROJECT_NAME + "/edgeThreshold", 0.1);
    surfThreshold           = this->declare_parameter<float>(PROJECT_NAME + "/surfThreshold", 0.1);
    edgeFeatureMinValidNum  = this->declare_parameter<int>(PROJECT_NAME + "/edgeFeatureMinValidNum", 10);
    surfFeatureMinValidNum  = this->declare_parameter<int>(PROJECT_NAME + "/surfFeatureMinValidNum", 100);

    odometrySurfLeafSize  = this->declare_parameter<float>(PROJECT_NAME + "/odometrySurfLeafSize", 0.2);
    mappingCornerLeafSize = this->declare_parameter<float>(PROJECT_NAME + "/mappingCornerLeafSize", 0.2);
    mappingSurfLeafSize   = this->declare_parameter<float>(PROJECT_NAME + "/mappingSurfLeafSize", 0.2);

    z_tollerance        = this->declare_parameter<float>(PROJECT_NAME + "/z_tollerance", FLT_MAX);
    rotation_tollerance = this->declare_parameter<float>(PROJECT_NAME + "/rotation_tollerance", FLT_MAX);

    numberOfCores           = this->declare_parameter<int>(PROJECT_NAME + "/numberOfCores", 2);
    mappingProcessInterval  = this->declare_parameter<double>(PROJECT_NAME + "/mappingProcessInterval", 0.15);

    surroundingkeyframeAddingDistThreshold  = this->declare_parameter<float>(PROJECT_NAME + "/surroundingkeyframeAddingDistThreshold", 1.0);
    surroundingkeyframeAddingAngleThreshold = this->declare_parameter<float>(PROJECT_NAME + "/surroundingkeyframeAddingAngleThreshold", 0.2);
    surroundingKeyframeDensity              = this->declare_parameter<float>(PROJECT_NAME + "/surroundingKeyframeDensity", 1.0);
    surroundingKeyframeSearchRadius         = this->declare_parameter<float>(PROJECT_NAME + "/surroundingKeyframeSearchRadius", 50.0);

    loopClosureEnableFlag         = this->declare_parameter<bool>(PROJECT_NAME + "/loopClosureEnableFlag", false);
    surroundingKeyframeSize       = this->declare_parameter<int>(PROJECT_NAME + "/surroundingKeyframeSize", 50);
    historyKeyframeSearchRadius   = this->declare_parameter<float>(PROJECT_NAME + "/historyKeyframeSearchRadius", 10.0);
    historyKeyframeSearchTimeDiff = this->declare_parameter<float>(PROJECT_NAME + "/historyKeyframeSearchTimeDiff", 30.0);
    historyKeyframeSearchNum      = this->declare_parameter<int>(PROJECT_NAME + "/historyKeyframeSearchNum", 25);
    historyKeyframeFitnessScore   = this->declare_parameter<float>(PROJECT_NAME + "/historyKeyframeFitnessScore", 0.3);

    globalMapVisualizationSearchRadius = this->declare_parameter<float>(PROJECT_NAME + "/globalMapVisualizationSearchRadius", 1e3);
    globalMapVisualizationPoseDensity  = this->declare_parameter<float>(PROJECT_NAME + "/globalMapVisualizationPoseDensity", 10.0);
    globalMapVisualizationLeafSize     = this->declare_parameter<float>(PROJECT_NAME + "/globalMapVisualizationLeafSize", 1.0);

    usleep(100);
}

sensor_msgs::msg::Imu ParamServer::imuConverter(const sensor_msgs::msg::Imu& imu_in)
{
    sensor_msgs::msg::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
#if IF_OFFICIAL    
    acc = extRot * acc;
#else
    acc = R_lidar_imu * acc;
#endif
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
#if IF_OFFICIAL     
    gyr = extRot * gyr;
#else 
    gyr = R_lidar_imu * gyr;
#endif

    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
#if IF_OFFICIAL    
    Eigen::Quaterniond q_final = q_from * extQRPY;
#else 
    Eigen::Quaterniond q_final = q_from * Q_quat_lidar;
#endif
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
        rclcpp::shutdown();
    }

    return imu_out;
}





float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

