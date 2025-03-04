#include "utility.hpp"
#include "featureExtraction.hpp"




FeatureExtraction::FeatureExtraction(std::string node_name): ParamServer(node_name)
{
    // imageProjection 发布的 cloud_info 消息，本质只进行了相对于imu的旋转变换，没有位移
    subLaserCloudInfo = this->create_subscription<lvi_sam::msg::CloudInfo>(PROJECT_NAME + "/lidar/deskew/cloud_info", 5, std::bind(&FeatureExtraction::laserCloudInfoHandler, this, std::placeholders::_1));

    // 发布提取特征后的 cloud_info cloud_corner cloud_corner
    pubLaserCloudInfo = this->create_publisher<lvi_sam::msg::CloudInfo> (PROJECT_NAME + "/lidar/feature/cloud_info", 5);
    pubCornerPoints   = this->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/lidar/feature/cloud_corner", 5);
    pubSurfacePoints  = this->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/lidar/feature/cloud_corner", 5);
    
    initializationValue();
}

    // 初始化
void FeatureExtraction::initializationValue()
{
    cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

    downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

    extractedCloud.reset(new pcl::PointCloud<PointType>());
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());

    cloudCurvature = new float[N_SCAN*Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
    cloudLabel = new int[N_SCAN*Horizon_SCAN];
}


void FeatureExtraction::laserCloudInfoHandler(const lvi_sam::msg::CloudInfo::SharedPtr msgIn)
{
    cloudInfo = *msgIn; // new cloud info
    cloudHeader = msgIn->header; // new cloud header
    pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

    calculateSmoothness();

    markOccludedPoints();

    extractFeatures();

    publishFeatureCloud();
}

void FeatureExtraction::calculateSmoothness()
{
    int cloudSize = extractedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++)
    {   
        // 对于前后5点计算平均差，然后平方变成正数方便之后操作
        float diffRange = cloudInfo.point_range[i-5] + cloudInfo.point_range[i-4]
                        + cloudInfo.point_range[i-3] + cloudInfo.point_range[i-2]
                        + cloudInfo.point_range[i-1] - cloudInfo.point_range[i] * 10
                        + cloudInfo.point_range[i+1] + cloudInfo.point_range[i+2]
                        + cloudInfo.point_range[i+3] + cloudInfo.point_range[i+4]
                        + cloudInfo.point_range[i+5];            

        cloudCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
        // cloudSmoothness for sorting 存储平滑度平滑度
        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;
    }
}

    // 标记点云中可能被遮挡的点以及平行束的点
void FeatureExtraction::markOccludedPoints()
{
    int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i)
    {   
        // occluded points  检查有没有被遮挡的点
        float depth1 = cloudInfo.point_range[i];
        float depth2 = cloudInfo.point_range[i+1];
        int columnDiff = std::abs(int(cloudInfo.point_col_ind[i+1] - cloudInfo.point_col_ind[i]));

        if (columnDiff < 10){
            // 10 pixel diff in range image
            if (depth1 - depth2 > 0.3){
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }
        // parallel beam
        float diff1 = std::abs(float(cloudInfo.point_range[i-1] - cloudInfo.point_range[i]));
        float diff2 = std::abs(float(cloudInfo.point_range[i+1] - cloudInfo.point_range[i]));

        // 如果前一个点和后一个点的深度差都大于当前点深度的 2%，认为当前点是平行束上的点，可能不在目标物体的表面上，因此标记为 1（被遮挡）
        if (diff1 > 0.02 * cloudInfo.point_range[i] && diff2 > 0.02 * cloudInfo.point_range[i])
            cloudNeighborPicked[i] = 1;
    }
}

// 提取特征 根据平滑度将点云存放到 角点和面点
void FeatureExtraction::extractFeatures()
{
    cornerCloud->clear();
    surfaceCloud->clear();

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for (int i = 0; i < N_SCAN; i++)
    {
        surfaceCloudScan->clear();

        for (int j = 0; j < 6; j++)
        {

            int sp = (cloudInfo.start_ring_index[i] * (6 - j) + cloudInfo.end_ring_index[i] * j) / 6;
            int ep = (cloudInfo.start_ring_index[i] * (5 - j) + cloudInfo.end_ring_index[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;

            std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                {
                    largestPickedNum++;
                    if (largestPickedNum <= 20){
                        cloudLabel[ind] = 1;
                        cornerCloud->push_back(extractedCloud->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                {

                    cloudLabel[ind] = -1;
                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++) {

                        int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {

                        int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0){
                    surfaceCloudScan->push_back(extractedCloud->points[k]);
                }
            }
        }

        surfaceCloudScanDS->clear();
        downSizeFilter.setInputCloud(surfaceCloudScan);
        downSizeFilter.filter(*surfaceCloudScanDS);

        *surfaceCloud += *surfaceCloudScanDS;
    }
}

void FeatureExtraction::freeCloudInfoMemory()
{
    cloudInfo.start_ring_index.clear();
    cloudInfo.start_ring_index.shrink_to_fit();
    cloudInfo.end_ring_index.clear();
    cloudInfo.end_ring_index.shrink_to_fit();
    cloudInfo.point_col_ind.clear();
    cloudInfo.point_col_ind.shrink_to_fit();
    cloudInfo.point_range.clear();
    cloudInfo.point_range.shrink_to_fit();
}

// 发布特征话题
void FeatureExtraction::publishFeatureCloud()
{
    // free cloud info memory
    freeCloudInfoMemory();
    // save newly extracted features
    cloudInfo.cloud_corner  = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, "base_link");
    cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, "base_link");
    // publish to mapOptimization
    pubLaserCloudInfo->publish(cloudInfo);
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FeatureExtraction>("FeatureExtraction");

    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Lidar Feature Extraction Started.\033[0m");

    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}