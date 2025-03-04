#include "utility.hpp"
#include "lvi_sam/msg/cloud_info.hpp"

// Velodyne
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;    //  16 字节对齐，提高 CPU 加速计算效率。

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint16_t, ring, ring) (float, time, time)
)

// Ouster
// struct PointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// }EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//     (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
// )

const int queueLength = 500;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    // 接收雷达点云和发送雷达点云
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  pubLaserCloud;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<lvi_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  subOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
    sensor_msgs::msg::PointCloud2 currentCloudMsg;
    
    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lvi_sam::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanNext;
    std_msgs::msg::Header cloudHeader;


public:
    ImageProjection(std::string node_name);

    void resetParameters();

    ~ImageProjection(){};

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg);

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg);

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

    void allocateMemory();

    bool deskewInfo();

    void imuDeskewInfo();

    void odomDeskewInfo();

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur);

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur);

    PointType deskewPoint(PointType *point, double relTime);

    void projectPointCloud();

    void cloudExtraction();

    void publishClouds();

};