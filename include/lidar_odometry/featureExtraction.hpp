#include "utility.hpp"

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{

public:

    rclcpp::Subscription<lvi_sam::msg::CloudInfo>::SharedPtr subLaserCloudInfo;

    rclcpp::Publisher<lvi_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    lvi_sam::msg::CloudInfo cloudInfo;
    std_msgs::msg::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    FeatureExtraction(std::string node_name);

    // 初始化
    void initializationValue();

    void laserCloudInfoHandler(const lvi_sam::msg::CloudInfo::SharedPtr msgIn);

    void calculateSmoothness();

    // 标记点云中可能被遮挡的点以及平行束的点
    void markOccludedPoints();

    // 提取特征 根据平滑度将点云存放到 角点和面点
    void extractFeatures();


    void freeCloudInfoMemory();

    // 发布特征话题
    void publishFeatureCloud();
};


