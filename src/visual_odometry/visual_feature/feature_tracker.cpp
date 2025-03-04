#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

DepthRegister::DepthRegister(std::string node_name) : Node(node_name)
{   
    depthCloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    readParameters(shared_from_this());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener =  std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // read camera params
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    // load fisheye mask to remove features on the boundry
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                RCLCPP_ERROR(this->get_logger(), "load fisheye mask fail");
            }
            else
                RCLCPP_INFO(this->get_logger(), "load mask success");
        }
    }
    
    for(int i = 0; i< NUM_OF_CAM; i++)
        trackerData.push_back(FeatureTracker(shared_from_this()));


    // messages for RVIZ visualization
    pub_depth_feature = this->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/vins/depth/depth_feature", 5);
    pub_depth_image = this->create_publisher<sensor_msgs::msg::Image>(PROJECT_NAME + "/vins/depth/depth_image", 5);
    pub_depth_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/vins/depth/depth_cloud", 5);

    pub_feature = this->create_publisher<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/vins/feature/feature",    5);
    pub_match   = this->create_publisher<sensor_msgs::msg::Image>     (PROJECT_NAME + "/vins/feature/feature_img", 5);
    pub_restart = this->create_publisher<std_msgs::msg::Bool>         (PROJECT_NAME + "/vins/feature/restart",     5);

    sub_img   = this->create_subscription<sensor_msgs::msg::Image>      (IMAGE_TOPIC, 5, std::bind(&DepthRegister::img_callback,   this, std::placeholders::_1));
    sub_lidar = this->create_subscription<sensor_msgs::msg::PointCloud2>(IMAGE_TOPIC, 5, std::bind(&DepthRegister::lidar_callback, this, std::placeholders::_1));
    pointsArray.resize(num_bins);
    for (int i = 0; i < num_bins; ++i)
        pointsArray[i].resize(num_bins);
}

void DepthRegister::get_depth(const rclcpp::Time &stamp_cur, const cv::Mat &imageCur,
                                const pcl::PointCloud<PointType>::Ptr &depthCloud_,
                                const camodocal::CameraPtr &camera_model,
                                const pcl::PointCloud<PointFeature>::Ptr features_2d
                                )
{
    // 0.1  首先获取特征点的数量
    const int size = features_2d->size();


    // 0.2  check if depthCloud available
    if (depthCloud_->size() == 0)
        return ;

    // 0.3 look up transform at current image time 查看当前图像时间的变换
    try
    {

    #if IF_OFFICIAL
        // listener.waitForTransform("vins_world", "vins_body_ros", stamp_cur, ros::Duration(0.01));
        // listener.lookupTransform("vins_world", "vins_body_ros", stamp_cur, transform);
        transform = tf_buffer_->lookupTransform(
            "vins_world", "vins_body_ros", stamp_cur, tf2::Duration::from_seconds(0.01));
    #else
        //? mod: 直接监听vins_camFLU坐标系在世界坐标系下的表示，这样就把VIO的动态外参包括进去了
        // listener.waitForTransform("vins_world", "vins_cameraFLU", stamp_cur, ros::Duration(0.01));
        // listener.lookupTransform("vins_world", "vins_cameraFLU", stamp_cur, transform);
        transform = tf_buffer_->lookupTransform(
            "vins_world", "vins_cameraFLU", stamp_cur,  rclcpp::Duration::from_seconds(0.01));
    #endif
    }
    catch (tf2::TransformException ex)
    {
        // ROS_ERROR("image no tf");
        RCLCPP_ERROR(this->get_logger(), "image no tf");
        return;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = transform.transform.translation.x;
    yCur = transform.transform.translation.y;
    zCur = transform.transform.translation.z;

    // 提取旋转信息
    // tf::Matrix3x3 m(transform.getRotation());
    // m.getRPY(rollCur, pitchCur, yawCur);
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(rollCur, pitchCur, yawCur);


    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    // 0.4 transform cloud from global frame to camera frame
    pcl::PointCloud<PointType>::Ptr depth_cloud_local(new pcl::PointCloud<PointType>());
    //; 这里就是把vins_world坐标系下的点云，转到了camera的FLU坐标系下
    pcl::transformPointCloud(*depthCloud_, *depth_cloud_local, transNow.inverse());

    // 0.5 project undistorted normalized (z) 2d features onto a unit sphere 将未失真的归一化（z）2D特征投影到一个单位球上
    pcl::PointCloud<PointType>::Ptr features_3d_sphere(new pcl::PointCloud<PointType>());
    PointFeature temp_point;
    for (int i = 0; i < (int)features_2d->size(); ++i)
    {   
        temp_point = features_2d->points[i];
        // normalize 2d feature to a unit sphere
        Eigen::Vector3f feature_cur(temp_point.x, temp_point.y, temp_point.z); // z always equal to 1
        feature_cur.normalize();
        // convert to ROS standard
        PointType p;
        p.x = feature_cur(2);
        p.y = -feature_cur(0);
        p.z = -feature_cur(1);
        p.intensity = -1; // intensity will be used to save depth
        features_3d_sphere->push_back(p);
    }

    // 3. project depth cloud on a range image, filter points satcked in the same region
    float bin_res = 180.0 / (float)num_bins; // currently only cover the space in front of lidar (-90 ~ 90)
    cv::Mat rangeImage = cv::Mat(num_bins, num_bins, CV_32F, cv::Scalar::all(FLT_MAX));

    for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
    {
        PointType p = depth_cloud_local->points[i];
        // filter points not in camera view
        if (p.x < 0 || abs(p.y / p.x) > 10 || abs(p.z / p.x) > 10)
            continue;
        // find row id in range image
        float row_angle = atan2(p.z, sqrt(p.x * p.x + p.y * p.y)) * 180.0 / M_PI + 90.0; // degrees, bottom -> up, 0 -> 360
        int row_id = round(row_angle / bin_res);
        // find column id in range image
        float col_angle = atan2(p.x, p.y) * 180.0 / M_PI; // degrees, left -> right, 0 -> 360
        int col_id = round(col_angle / bin_res);
        // id may be out of boundary
        if (row_id < 0 || row_id >= num_bins || col_id < 0 || col_id >= num_bins)
            continue;
        // only keep points that's closer
        float dist = pointDistance(p);
        if (dist < rangeImage.at<float>(row_id, col_id))
        {
            rangeImage.at<float>(row_id, col_id) = dist;
            pointsArray[row_id][col_id] = p;
        }
    }

    // 4. extract downsampled depth cloud from range image
    pcl::PointCloud<PointType>::Ptr depth_cloud_local_filter2(new pcl::PointCloud<PointType>());
    for (int i = 0; i < num_bins; ++i)
    {
        for (int j = 0; j < num_bins; ++j)
        {
            if (rangeImage.at<float>(i, j) != FLT_MAX)
                depth_cloud_local_filter2->push_back(pointsArray[i][j]);
        }
    }
    *depth_cloud_local = *depth_cloud_local_filter2;

#if IF_OFFICIAL
    publishCloud(pub_depth_cloud, depth_cloud_local, stamp_cur, "vins_body_ros");
#else
    //? mod: 发布点云的坐标系
    publishCloud(pub_depth_cloud, depth_cloud_local, stamp_cur, "vins_cameraFLU");
#endif

    // 5. project depth cloud onto a unit sphere
    pcl::PointCloud<PointType>::Ptr depth_cloud_unit_sphere(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
    {
        PointType p = depth_cloud_local->points[i];
        float range = pointDistance(p);
        p.x /= range;
        p.y /= range;
        p.z /= range;
        p.intensity = range;
        depth_cloud_unit_sphere->push_back(p);
    }
    if (depth_cloud_unit_sphere->size() < 10)
        return;

    // 6. create a kd-tree using the spherical depth cloud
    pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());
    kdtree->setInputCloud(depth_cloud_unit_sphere);

    // 7. find the feature depth using kd-tree
    vector<int> pointSearchInd;
    vector<float> pointSearchSqDis;
    float dist_sq_threshold = pow(sin(bin_res / 180.0 * M_PI) * 5.0, 2);
    for (int i = 0; i < (int)features_3d_sphere->size(); ++i)
    {
        kdtree->nearestKSearch(features_3d_sphere->points[i], 3, pointSearchInd, pointSearchSqDis);
        if (pointSearchInd.size() == 3 && pointSearchSqDis[2] < dist_sq_threshold)
        {
            float r1 = depth_cloud_unit_sphere->points[pointSearchInd[0]].intensity;
            Eigen::Vector3f A(depth_cloud_unit_sphere->points[pointSearchInd[0]].x * r1,
                                depth_cloud_unit_sphere->points[pointSearchInd[0]].y * r1,
                                depth_cloud_unit_sphere->points[pointSearchInd[0]].z * r1);

            float r2 = depth_cloud_unit_sphere->points[pointSearchInd[1]].intensity;
            Eigen::Vector3f B(depth_cloud_unit_sphere->points[pointSearchInd[1]].x * r2,
                                depth_cloud_unit_sphere->points[pointSearchInd[1]].y * r2,
                                depth_cloud_unit_sphere->points[pointSearchInd[1]].z * r2);

            float r3 = depth_cloud_unit_sphere->points[pointSearchInd[2]].intensity;
            Eigen::Vector3f C(depth_cloud_unit_sphere->points[pointSearchInd[2]].x * r3,
                                depth_cloud_unit_sphere->points[pointSearchInd[2]].y * r3,
                                depth_cloud_unit_sphere->points[pointSearchInd[2]].z * r3);

            // https://math.stackexchange.com/questions/100439/determine-where-a-vector-will-intersect-a-plane
            Eigen::Vector3f V(features_3d_sphere->points[i].x,
                                features_3d_sphere->points[i].y,
                                features_3d_sphere->points[i].z);

            Eigen::Vector3f N = (A - B).cross(B - C);
            float s = (N(0) * A(0) + N(1) * A(1) + N(2) * A(2)) / (N(0) * V(0) + N(1) * V(1) + N(2) * V(2));

            float min_depth = min(r1, min(r2, r3));
            float max_depth = max(r1, max(r2, r3));
            if (max_depth - min_depth > 2 || s <= 0.5)
            {
                continue;
            }
            else if (s - max_depth > 0)
            {
                s = max_depth;
            }
            else if (s - min_depth < 0)
            {
                s = min_depth;
            }
            // convert feature into cartesian space if depth is available
            features_3d_sphere->points[i].x *= s;
            features_3d_sphere->points[i].y *= s;
            features_3d_sphere->points[i].z *= s;
            // the obtained depth here is for unit sphere, VINS estimator needs depth for normalized feature (by value z), (lidar x = camera z)
            features_3d_sphere->points[i].intensity = features_3d_sphere->points[i].x;
        }
    }

    // visualize features in cartesian 3d space (including the feature without depth (default 1))
#if IF_OFFICIAL
    publishCloud(&pub_depth_feature, features_3d_sphere, stamp_cur, "vins_body_ros");
#else
    //? mod：发布点云坐标系
    publishCloud(pub_depth_feature, features_3d_sphere, stamp_cur, "vins_cameraFLU");
#endif

    // update depth value for return 更新深度值
    for (int i = 0; i < (int)features_3d_sphere->size(); ++i)
    {
        if (features_3d_sphere->points[i].intensity > 3.0)
           features_2d->points[i].depth = features_3d_sphere->points[i].intensity;
    }

    // visualization project points on image for visualization (it's slow!)
    if (pub_depth_image->get_subscription_count() != 0)
    {
        vector<cv::Point2f> points_2d;
        vector<float> points_distance;

        for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
        {
            // convert points from 3D to 2D
            Eigen::Vector3d p_3d(-depth_cloud_local->points[i].y,
                                    -depth_cloud_local->points[i].z,
                                    depth_cloud_local->points[i].x);
            Eigen::Vector2d p_2d;
            camera_model->spaceToPlane(p_3d, p_2d);

            points_2d.push_back(cv::Point2f(p_2d(0), p_2d(1)));
            points_distance.push_back(pointDistance(depth_cloud_local->points[i]));
        }

        cv::Mat showImage, circleImage;
        cv::cvtColor(imageCur, showImage, cv::COLOR_GRAY2RGB);
        circleImage = showImage.clone();
        for (int i = 0; i < (int)points_2d.size(); ++i)
        {
            float r, g, b;
            getColor(points_distance[i], 50.0, r, g, b);
            cv::circle(circleImage, points_2d[i], 0, cv::Scalar(r, g, b), 5);
        }
        cv::addWeighted(showImage, 1.0, circleImage, 0.7, 0, showImage); // blend camera image and circle image

        cv_bridge::CvImage bridge;
        bridge.image = showImage;
        bridge.encoding = "rgb8";
        sensor_msgs::msg::Image::SharedPtr imageShowPointer = bridge.toImageMsg();
        imageShowPointer->header.stamp = stamp_cur;
        pub_depth_image->publish(*imageShowPointer);
    }

    return;
}


void DepthRegister::img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    double cur_img_time = rclcpp::Time(img_msg->header.stamp).seconds();

    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }
    // detect unstable camera stream
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        RCLCPP_WARN(this->get_logger(), "image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;
    // frequency control
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCLCPP_DEBUG(this->get_logger(), "processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), cur_img_time);
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

        #if SHOW_UNDISTORTION
            trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
        #endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

    
   if (PUB_THIS_FRAME)
   {
        pub_count++;
        auto feature_points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::PointCloud<PointFeature>::Ptr point_feature(new pcl::PointCloud<PointFeature>);
        

        feature_points_msg->header.stamp = img_msg->header.stamp;
        feature_points_msg->header.frame_id = "vins_body";
        
        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    // 创建一个PointFeature点并赋值
                    PointFeature point;
                    point.x = un_pts[j].x;             // x坐标
                    point.y = un_pts[j].y;             // y坐标
                    point.z = 1.0f;                    // z坐标（假设为1.0）
                    point.id = p_id * NUM_OF_CAM + i;   // 点的ID
                    point.u = cur_pts[j].x;            // 图像平面上的u坐标
                    point.v = cur_pts[j].y;            // 图像平面上的v坐标
                    point.velocity_x = pts_velocity[j].x; // x方向上的速度
                    point.velocity_y = pts_velocity[j].y; // y方向上的速度

                    point_feature->push_back(point);
                }
            }
        }


        // get feature depth from lidar point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();

        get_depth(img_msg->header.stamp, show_img, depth_cloud_temp, trackerData[0].m_camera, point_feature);

        
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pcl::toROSMsg(*point_feature, *feature_points_msg);
            pub_feature->publish(*feature_points_msg);

        // publish features in image
        if (pub_match->get_subscription_count() != 0)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                    } else {
                        // depth 
                        if(j < point_feature->size())
                        {
                            if (point_feature->points[j].depth > 0)
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                        }
                    }
                }
            }

            pub_match->publish(*ptr->toImageMsg());
        }
    }
}


void DepthRegister::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg)
{
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP+1) != 0)
        return;

    // 0. listen to transform
    // static tf::TransformListener listener;
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;
#if IF_OFFICIAL
    // static tf::StampedTransform transform;   //; T_vinsworld_camera_FLU
    static geometry_msgs::msg::TransformStamped transform;
#else
    // static tf::StampedTransform transform_world_cFLU;   //; T_vinsworld_camera_FLU
    // static tf::StampedTransform transform_cFLU_imu;    //; T_cameraFLU_imu
    static geometry_msgs::msg::TransformStamped transform_world_cFLU;   //; T_vinsworld_camera_FLU
    static geometry_msgs::msg::TransformStamped transform_cFLU_imu;    //; T_cameraFLU_imu
#endif
    try{
    #if IF_OFFICIAL
        // listener.waitForTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, ros::Duration(0.01));
        // listener.lookupTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, transform);
        transform = tf_buffer_.lookupTransform(
        "vins_world", "vins_body_ros", laser_msg->header.stamp, rclcpp::Duration::from_seconds(0.01));
    #else   
        //? mod: 监听T_vinsworld_cameraFLU 和 T_cameraFLU_imu
        // listener.waitForTransform("vins_world", "vins_cameraFLU", laser_msg->header.stamp, ros::Duration(0.01));
        // listener.lookupTransform("vins_world", "vins_cameraFLU", laser_msg->header.stamp, transform_world_cFLU);
        // listener.waitForTransform("vins_cameraFLU", "vins_body_imuhz", laser_msg->header.stamp, ros::Duration(0.01));
        // listener.lookupTransform("vins_cameraFLU", "vins_body_imuhz", laser_msg->header.stamp, transform_cFLU_imu);
        transform_world_cFLU = tf_buffer_->lookupTransform(
        "vins_world", "vins_cameraFLU", laser_msg->header.stamp, rclcpp::Duration::from_seconds(0.01));
        transform_cFLU_imu = tf_buffer_->lookupTransform(
        "vins_cameraFLU", "vins_body_imuhz", laser_msg->header.stamp, rclcpp::Duration::from_seconds(0.01));
    #endif
    } 
    catch (tf2::TransformException& ex){
        // ROS_ERROR("lidar no tf");
        return;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    #if IF_OFFICIAL
        xCur = transform.transform.translation.x;
        yCur = transform.transform.translation.y;
        zCur = transform.transform.translation.z;
        tf2::Quaternion quat;
        tf2::fromMsg(transform.transform.rotation, quat); 
        tf2::Matrix3x3 m(quat); 
    #else
        xCur = transform_world_cFLU.transform.translation.x;
        yCur = transform_world_cFLU.transform.translation.y;
        zCur = transform_world_cFLU.transform.translation.z;
        tf2::Quaternion quat;
        tf2::fromMsg(transform_world_cFLU.transform.rotation, quat);  // 将 geometry_msgs::msg::Quaternion 转换为 tf2::Quaternion
        tf2::Matrix3x3 m(quat);
    #endif
    m.getRPY(rollCur, pitchCur, yawCur);
    //; T_vinswolrd_cameraFLU
    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    // 1. convert laser cloud message to pcl
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds;

    // 3. 把lidar坐标系下的点云转到相机的FLU坐标系下表示，因为下一步需要使用相机FLU坐标系下的点云进行初步过滤
#if IF_OFFICIAL
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;
#else
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    //; T_cFLU_lidar`
     tf2::Transform transform_cFLU_imu_tf;
    tf2::fromMsg(transform_cFLU_imu.transform, transform_cFLU_imu_tf);
    tf2::Transform transform_cFLU_lidar = transform_cFLU_imu_tf * Transform_imu_lidar;
    double roll, pitch, yaw, x, y, z;
    x = transform_cFLU_lidar.getOrigin().getX();
    y = transform_cFLU_lidar.getOrigin().getY();
    z = transform_cFLU_lidar.getOrigin().getZ();
    tf2::Matrix3x3(transform_cFLU_lidar.getRotation()).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transOffset = pcl::getTransformation(x, y, z, roll, pitch, yaw);
    //; lidar本体坐标系下的点云，转到相机FLU坐标系下表示
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;
#endif

    // 4. filter lidar points (only keep points in camera view)
    //; 根据已经转到相机FLU坐标系下的点云，先排除不在相机FoV内的点云
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
            laser_cloud_in_filter->push_back(p);
    }
    *laser_cloud_in = *laser_cloud_in_filter;

    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    //; cameraFLU坐标系下的点云，转到vinsworld系下表示
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

    // 6. save new cloud
    double timeScanCur = rclcpp::Time(laser_msg->header.stamp).seconds();
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 5.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        } else {
            break;
        }
    }

    std::lock_guard<std::mutex> lock(mtx_lidar);
    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
        *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;
}

FeatureTracker::FeatureTracker(const std::shared_ptr<rclcpp::Node> n)
                                :node_ptr(n) 
{
    RCLCPP_INFO(node_ptr->get_logger(), "FeatureTracker initialized");
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        RCLCPP_DEBUG(node_ptr->get_logger(), "CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        RCLCPP_DEBUG(node_ptr->get_logger(), "temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF();
        RCLCPP_DEBUG(node_ptr->get_logger(), "set mask begins");
        TicToc t_m;
        setMask();
        RCLCPP_DEBUG(node_ptr->get_logger(), "set mask costs %fms", t_m.toc());

        RCLCPP_DEBUG(node_ptr->get_logger(), "detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        RCLCPP_DEBUG(node_ptr->get_logger(),"detect feature costs: %fms", t_t.toc());

        RCLCPP_DEBUG(node_ptr->get_logger(), "add feature begins");
        TicToc t_a;
        addPoints();
        RCLCPP_DEBUG(node_ptr->get_logger(), "selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        RCLCPP_DEBUG(node_ptr->get_logger(), "FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        RCLCPP_DEBUG(node_ptr->get_logger(),"FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        RCLCPP_DEBUG(node_ptr->get_logger(),"FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    RCLCPP_INFO(node_ptr->get_logger(), "reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
