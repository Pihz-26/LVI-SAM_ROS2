#include "visualization.h"

using namespace Eigen;
// ros::Publisher pub_odometry, pub_latest_odometry, pub_latest_odometry_ros;
// ros::Publisher pub_path;
// ros::Publisher pub_point_cloud, pub_margin_cloud;
// ros::Publisher pub_key_poses;
// ros::Publisher pub_camera_pose;
// ros::Publisher pub_camera_pose_visual;
// nav_msgs::Path path;

// ros::Publisher pub_keyframe_pose;
// ros::Publisher pub_keyframe_point;
// ros::Publisher pub_extrinsic;

// CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
// CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);


// 将 tf::StampedTransform 类型的变换信息转换为 tf2::Transform 类型的变换信息
tf2::Transform transformConversion(geometry_msgs::msg::TransformStamped *t)
{
    // double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    // xCur = t.getOrigin().x();
    // yCur = t.getOrigin().y();
    // zCur = t.getOrigin().z();

    double xCur = t->transform.translation.x;
    double yCur = t->transform.translation.y;
    double zCur = t->transform.translation.z;

    // 提取旋转部分并转换为欧拉角
    tf2::Quaternion q;
    tf2::fromMsg(t->transform.rotation, q); // 将 ROS 2 的四元数消息转换为 tf2::Quaternion
    tf2::Matrix3x3 m(q);
    double rollCur, pitchCur, yawCur;
    m.getRPY(rollCur, pitchCur, yawCur); // 从旋转矩阵中提取欧拉角

    tf2::Quaternion q_new;
    q_new.setRPY(rollCur, pitchCur, yawCur); 

    return tf2::Transform(q_new, tf2::Vector3(xCur, yCur, zCur));;
}


void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, 
    const Eigen::Vector3d &V, std_msgs::msg::Header &header, const int &failureId,
    const Eigen::Vector3d &t_ic, const Eigen::Quaterniond &q_ic, Estimator* node_ptr)
{
    // static tf::TransformBroadcaster br;
    // static tf::TransformListener listener;
    static double last_align_time = -1;

    // Quternion not normalized
    if (Q.x() * Q.x() + Q.y() * Q.y() + Q.z() * Q.z() + Q.w() * Q.w() < 0.99)
        return;


    // imu odometry in camera frame
    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.child_frame_id = "vins_body";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    node_ptr->pub_latest_odometry->publish(odometry);


#if IF_OFFICIAL
    // imu odometry in ROS format (change rotation), used for lidar odometry initial guess
    odometry.pose.covariance[0] = double(failureId); // notify lidar odometry failure
    tf::Quaternion q_odom_cam(Q.x(), Q.y(), Q.z(), Q.w());
    tf::Quaternion q_cam_to_lidar(0, 1, 0, 0); // mark: camera - lidar
    tf::Quaternion q_odom_ros = q_odom_cam * q_cam_to_lidar;
    tf::quaternionTFToMsg(q_odom_ros, odometry.pose.pose.orientation);
    pub_latest_odometry_ros.publish(odometry);

    // TF of camera in vins_world in ROS format (change rotation), used for depth registration
    tf::Transform t_w_body = tf::Transform(q_odom_ros, tf::Vector3(P.x(), P.y(), P.z()));
    tf::StampedTransform trans_world_vinsbody_ros = tf::StampedTransform(
        t_w_body, header.stamp, "vins_world", "vins_body_ros");
    br.sendTransform(trans_world_vinsbody_ros);
#else
    // Step 1: 发布T_odom_lidar给LIO后端的scan-to-map位姿初值估计
    odometry.pose.covariance[0] = double(failureId); // notify lidar odometry failure
    //; R_odom_imu
    tf2::Quaternion q_odom_imu(Q.x(), Q.y(), Q.z(), Q.w());   
    Eigen::Quaterniond q_imu_lidar(R_imu_lidar);
    //; R_imu_lidar
    tf2::Quaternion q_imu_lidar_tf(q_imu_lidar.x(), q_imu_lidar.y(), q_imu_lidar.z(), q_imu_lidar.w());   
    //; R_odom_lidar = R_odom_imu * R_imu_lidar
    tf2::Quaternion q_odom_lidar = q_odom_imu * q_imu_lidar_tf;
    //; t_dodom_lidar = R_odom_imu * t_imu_lidar + t_odom_imu
    Eigen::Vector3d t_odom_lidar = Q * t_imu_lidar + P;  
    odometry.pose.pose.position.x = t_odom_lidar.x();
    odometry.pose.pose.position.y = t_odom_lidar.y();
    odometry.pose.pose.position.z = t_odom_lidar.z();
    odometry.pose.pose.orientation = tf2::toMsg(q_odom_lidar);
    node_ptr->pub_latest_odometry_ros->publish(odometry);

    // Step 2: 发布IMU频率下的T_odom_imu的tf位姿变换
    tf2::Transform t_w_body = tf2::Transform(q_odom_imu, tf2::Vector3(P.x(), P.y(), P.z()));
    // tf::StampedTransform trans_world_vinsBody = tf::StampedTransform(
    //     t_w_body, header.stamp, "vins_world", "vins_body_imuhz");
    // br.sendTransform(trans_world_vinsBody);
    geometry_msgs::msg::TransformStamped trans_world_vinsBody;
    trans_world_vinsBody.header.stamp = header.stamp;
    trans_world_vinsBody.header.frame_id = "vins_world";  
    trans_world_vinsBody.child_frame_id = "vins_body_imuhz"; 
    trans_world_vinsBody.transform.translation.x = t_w_body.getOrigin().x();
    trans_world_vinsBody.transform.translation.y = t_w_body.getOrigin().y();
    trans_world_vinsBody.transform.translation.z = t_w_body.getOrigin().z();
    trans_world_vinsBody.transform.rotation = tf2::toMsg(t_w_body.getRotation());

    node_ptr->tf_broad_.sendTransform(trans_world_vinsBody);


    // Step 3: 发布camera 和 IMU之间的外参，这个在动态估计外参的时候会变换，所以前端深度注册也要使用这个动态外参
    //; 另外这里发布外参的时候直接发布了vins相机的FLU坐标系的变换，因为前端深度注册要用相机的FLU坐标系
    // tf2::Transform transform;
    // tf2::Quaternion q;
    //; Eigen::Quaterniond(0.5, 0.5, -0.5, 0.5)对应的旋转矩阵是R_c_cFLU，即相机的
    //; 前左上坐标系 -> 正常的相机右下前坐标系 之间的旋转，结果就是[0, -1, 0; 0, 0, -1; 1, 0, 0]
    //; R_imu_cFLU = R_imu_cam * R_cam_camFLU
    
    // transform.setOrigin(tf2::Vector3(0, 0, 0));
    // q.setW(q_i_cFLU.w());
    // q.setX(q_i_cFLU.x());
    // q.setY(q_i_cFLU.y());
    // q.setZ(q_i_cFLU.z());
    // transform.setRotation(q);
    // node_ptr->tf_broad_.sendTransform(tf::StampedTransform(
    //     transform, header->stamp, "vins_body_imuhz", "vins_cameraFLU"));

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = header.stamp;
    transform_stamped.header.frame_id = "vins_body_imuhz";
    transform_stamped.child_frame_id = "vins_cameraFLU";
    Eigen::Quaterniond q_i_cFLU = q_ic * Eigen::Quaterniond(0.5, 0.5, -0.5, 0.5);
    transform_stamped.transform.translation.x = 0;  // 平移设置为 (0, 0, 0)
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.translation.z = 0;
    transform_stamped.transform.rotation.x = q_i_cFLU.x();  // 四元数赋值
    transform_stamped.transform.rotation.y = q_i_cFLU.y();
    transform_stamped.transform.rotation.z = q_i_cFLU.z();
    transform_stamped.transform.rotation.w = q_i_cFLU.w();
    node_ptr->tf_broad_.sendTransform(transform_stamped);

#endif

    
    if (ALIGN_CAMERA_LIDAR_COORDINATE)
    {
    #if IF_OFFICIAL
        static tf::Transform t_odom_world = tf::Transform(tf::createQuaternionFromRPY(0, 0, M_PI), tf::Vector3(0, 0, 0));
    #else
        //? mod: vins_world坐标系和odom坐标系不再绕着Z轴旋转，而是直接对齐
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        static tf2::Transform t_odom_world(q, tf2::Vector3(0, 0, 0));
    #endif

        if (rclcpp::Time(header.stamp).seconds() - last_align_time > 1.0)
        {
            try
            {
            #if IF_OFFICIAL
                tf::StampedTransform trans_odom_baselink;
                listener.lookupTransform("odom","base_link", ros::Time(0), trans_odom_baselink);
                t_odom_world = transformConversion(trans_odom_baselink) * transformConversion(trans_world_vinsbody_ros).inverse();
                last_align_time = header.stamp.toSec();
            #else
                //; 计算odom坐标系和vins_world之间的变换关系，这个变换是会变化的，因为vins不准确存在漂移，
                //; 这里就用VINS估计的实时位姿和LIO估计的实时位姿对齐，然后把误差分配到odom和vins_world之间的变换上 
                // tf::StampedTransform T_odom_lidar;
                // listener.lookupTransform("odom", "base_link", ros::Time(0), T_odom_lidar);
                geometry_msgs::msg::TransformStamped T_odom_lidar;
                T_odom_lidar = node_ptr->tf_buffer_->lookupTransform("odom", "base_link", rclcpp::Time(0));


                tf2::Transform t_w_lidar = tf2::Transform(q_odom_lidar, tf2::Vector3(t_odom_lidar.x(), t_odom_lidar.y(), t_odom_lidar.z()));
                // tf::StampedTransform T_vinsworld_lidar = tf::StampedTransform(
                //     t_w_lidar, header.stamp, "vinsworld", "lidar");

                geometry_msgs::msg::TransformStamped T_vinsworld_lidar;
                
                    
                T_vinsworld_lidar.header.stamp = header.stamp;
                T_vinsworld_lidar.header.frame_id = "vinsworld";
                T_vinsworld_lidar.child_frame_id = "lidar"; 
                T_vinsworld_lidar.transform.translation.x = t_w_lidar.getOrigin().x();
                T_vinsworld_lidar.transform.translation.y = t_w_lidar.getOrigin().y();
                T_vinsworld_lidar.transform.translation.z = t_w_lidar.getOrigin().z();
                T_vinsworld_lidar.transform.rotation = tf2::toMsg(t_w_lidar.getRotation());

                //; T_odom_vinsworld = T_odom_lidar * T_vinsworld_lidar.inverse()
                t_odom_world = transformConversion(&T_odom_lidar) * transformConversion(&T_vinsworld_lidar).inverse();
                last_align_time = rclcpp::Time(header.stamp).seconds();
            #endif
            }
            catch (tf2::TransformException &ex)
            {
            }
        }
        geometry_msgs::msg::TransformStamped T_odom_word;
        T_odom_word.header.stamp = header.stamp;
        T_odom_word.header.frame_id = "odom";
        T_odom_word.child_frame_id = "vins_world";
        T_odom_word.transform = tf2::toMsg(t_odom_world);
        node_ptr->tf_broad_.sendTransform(T_odom_word);
    }
    else
    {

    #if IF_OFFICIAL
        static geometry_msgs::msg::TransformStamped transform_static;
        static tf2::Quaternion q;
        q.setRPY(0, 0, M_PI); // 设置绕 Z 轴旋转 180 度
        transform_static.transform.rotation.x = q.x();
        transform_static.transform.rotation.y = q.y();
        transform_static.transform.rotation.z = q.z();
        transform_static.transform.rotation.w = q.w();
        transform_static.transform.translation.x = 0;
        transform_static.transform.translation.y = 0;
        transform_static.transform.translation.z = 0;
    #else
        //? mod: vins_world坐标系和odom坐标系不再绕着Z轴旋转，而是直接对齐
        // static tf::Transform t_static = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        geometry_msgs::msg::TransformStamped transform_static;
        transform_static.transform.rotation.w = 1.0; // 单位四元数，无旋转
        transform_static.transform.translation.x = 0;
        transform_static.transform.translation.y = 0;
        transform_static.transform.translation.z = 0;
    #endif
        transform_static.header.stamp = header.stamp; // header 是消息头
        transform_static.header.frame_id = "odom";
        transform_static.child_frame_id = "vins_world";
        node_ptr->tf_broad_.sendTransform(transform_static);
    }
}

void printStatistics(Estimator *estimator, double t)
{
    if (estimator->solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %f, %f, %f\r", estimator->Ps[WINDOW_SIZE].x(), estimator->Ps[WINDOW_SIZE].y(), estimator->Ps[WINDOW_SIZE].z());
    RCLCPP_DEBUG_STREAM(estimator->get_logger(), "position: " << estimator->Ps[WINDOW_SIZE].transpose());
    RCLCPP_DEBUG_STREAM(estimator->get_logger(), "orientation: " << estimator->Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //RCLCPP_DEBUG("calibration result for camera %d", i);
        RCLCPP_DEBUG_STREAM(estimator->get_logger(), "extirnsic tic: " << estimator->tic[i].transpose());
        RCLCPP_DEBUG_STREAM(estimator->get_logger(), "extrinsic ric: " << Utility::R2ypr(estimator->ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator->ric[i];
            eigen_T = estimator->tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    RCLCPP_DEBUG(estimator->get_logger(), "vo solver costs: %f ms", t);
    RCLCPP_DEBUG(estimator->get_logger(), "average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator->Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator->Ps[WINDOW_SIZE];
    RCLCPP_DEBUG(estimator->get_logger(), "sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        RCLCPP_INFO(estimator->get_logger(), "td %f", estimator->td);
}

void pubOdometry(Estimator *estimator, std_msgs::msg::Header &header)
{
    if (estimator->solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.child_frame_id = "vins_world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator->Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator->Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator->Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator->Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator->Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator->Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator->Vs[WINDOW_SIZE].z();
        estimator->pub_odometry->publish(odometry);

        static double path_save_time = -1;
        if (rclcpp::Time(header.stamp).seconds() - path_save_time > 0.5)
        {
            path_save_time = rclcpp::Time(header.stamp).seconds();
            geometry_msgs::msg::PoseStamped pose_stamped;
            nav_msgs::msg::Path             path;
            pose_stamped.header = header;
            pose_stamped.header.frame_id = "vins_world";
            pose_stamped.pose = odometry.pose.pose;
            path.header = header;
            path.header.frame_id = "vins_world";
            path.poses.push_back(pose_stamped);
            estimator->pub_path->publish(path);
        }
    }
}

void pubKeyPoses(Estimator *estimator, std_msgs::msg::Header &header)
{
    if (estimator->pub_key_poses->get_subscription_count() == 0)
        return;

    if (estimator->key_poses.size() == 0)
        return;
    visualization_msgs::msg::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "vins_world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::msg::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = rclcpp::Duration(0.05);
    
    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::msg::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator->key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    estimator->pub_key_poses->publish(key_poses);
}

void pubCameraPose(Estimator *estimator, std_msgs::msg::Header &header)
{
    if (estimator->pub_camera_pose_visual->get_subscription_count() == 0)
        return;

    int idx2 = WINDOW_SIZE - 1;

    if (estimator->solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator->Ps[i] + estimator->Rs[i] * estimator->tic[0];
        Quaterniond R = Quaterniond(estimator->Rs[i] * estimator->ric[0]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        estimator->pub_camera_pose->publish(odometry);

        estimator->cameraposevisual.reset();
        estimator->cameraposevisual.add_pose(P, R);
        estimator->cameraposevisual.publish_by(estimator->pub_camera_pose_visual, odometry.header);
    }
}

void pubPointCloud(Estimator* estimator, std_msgs::msg::Header &header)
{
    if (estimator->pub_point_cloud->get_subscription_count() != 0)
    {
        sensor_msgs::msg::PointCloud2 point_cloud;
        point_cloud.header = header;
        point_cloud.header.frame_id = "vins_world";

        pcl::PointCloud<pcl::PointXYZI> cloud;

        for (auto &it_per_id : estimator->feature_manager.feature)
        {
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
                continue;

            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator->Rs[imu_i] * (estimator->ric[0] * pts_i + estimator->tic[0]) + estimator->Ps[imu_i];

            pcl::PointXYZI p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);

            if (it_per_id.lidar_depth_flag == false)
                p.intensity = 0;
            else
                p.intensity = 1;
            cloud.push_back(p);
            
        }

        cloud.width = cloud.size();
        cloud.height = 1;
        cloud.is_dense = true; 
        pcl::toROSMsg(cloud, point_cloud);
        estimator->pub_point_cloud->publish(point_cloud);
    }

    // pub margined potin
    if (estimator->pub_margin_cloud->get_subscription_count() != 0)
    {
        sensor_msgs::msg::PointCloud2 margin_cloud;
        margin_cloud.header = header;
        margin_cloud.header.frame_id = "vins_world";

        pcl::PointCloud<pcl::PointXYZI> cloud;

        for (auto &it_per_id : estimator->feature_manager.feature)
        {
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 && it_per_id.solve_flag == 1)
            {
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator->Rs[imu_i] * (estimator->ric[0] * pts_i + estimator->tic[0]) + estimator->Ps[imu_i];

                pcl::PointXYZI p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                

                if (it_per_id.lidar_depth_flag == false)
                    p.intensity = 0;
                else
                    p.intensity = 1;
                cloud.push_back(p);

            }
        }

        cloud.width = cloud.size();
        cloud.height = 1;
        cloud.is_dense = true;
        pcl::toROSMsg(cloud, margin_cloud);
        estimator->pub_margin_cloud->publish(margin_cloud);
    }
}

/**
 * @brief 发布估计的TF坐标变换，这个非常重要
 * 
 * @param[in] estimator 
 * @param[in] header 
 */
void pubTF(Estimator *estimator, std_msgs::msg::Header &header)
{
    if (estimator->solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    
    tf2::Transform transform;
    tf2::Quaternion q;
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = header.stamp;
    transform_stamped.header.frame_id = "vins_world"; // 父坐标系
    transform_stamped.child_frame_id = "vins_body";  // 子坐标系


    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator->Ps[WINDOW_SIZE];
    correct_q = estimator->Rs[WINDOW_SIZE];

    transform.setOrigin(tf2::Vector3(correct_t(0), correct_t(1), correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());

    transform_stamped.transform.translation.x = correct_t(0);
    transform_stamped.transform.translation.y = correct_t(1);
    transform_stamped.transform.translation.z = correct_t(2);
    
    transform_stamped.transform.rotation.w = correct_q.w();
    transform_stamped.transform.rotation.x = correct_q.x();
    transform_stamped.transform.rotation.y = correct_q.y();
    transform_stamped.transform.rotation.z = correct_q.z();
    // transform.setRotation(q);
    estimator->tf_broad_.sendTransform(transform_stamped);

    // camera frame
    transform.setOrigin(tf2::Vector3(estimator->tic[0].x(),
                                    estimator->tic[0].y(),
                                    estimator->tic[0].z()));
    q.setW(Quaterniond(estimator->ric[0]).w());
    q.setX(Quaterniond(estimator->ric[0]).x());
    q.setY(Quaterniond(estimator->ric[0]).y());
    q.setZ(Quaterniond(estimator->ric[0]).z());
    transform.setRotation(q);

    geometry_msgs::msg::TransformStamped T_Vbody_Vcamera;
    T_Vbody_Vcamera.header.stamp = header.stamp;
    T_Vbody_Vcamera.header.frame_id = "vins_body";
    T_Vbody_Vcamera.child_frame_id = "vins_camera";
    T_Vbody_Vcamera.transform.translation.x = transform.getOrigin().x();
    T_Vbody_Vcamera.transform.translation.y = transform.getOrigin().y();
    T_Vbody_Vcamera.transform.translation.z = transform.getOrigin().z();

    T_Vbody_Vcamera.transform.rotation.x = transform.getRotation().x();
    T_Vbody_Vcamera.transform.rotation.y = transform.getRotation().y();
    T_Vbody_Vcamera.transform.rotation.z = transform.getRotation().z();
    T_Vbody_Vcamera.transform.rotation.w = transform.getRotation().w();

    estimator->tf_broad_.sendTransform(T_Vbody_Vcamera);

    nav_msgs::msg::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vins_world";
    odometry.pose.pose.position.x = estimator->tic[0].x();
    odometry.pose.pose.position.y = estimator->tic[0].y();
    odometry.pose.pose.position.z = estimator->tic[0].z();
    Quaterniond tmp_q{estimator->ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    estimator->pub_extrinsic->publish(odometry);
}

void pubKeyframe(Estimator* estimator)
{
    if (estimator->pub_keyframe_pose->get_subscription_count() == 0 && estimator->pub_keyframe_point->get_subscription_count() == 0)
        return;

    // pub camera pose, 2D-3D points of keyframe
    if (estimator->solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator->marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator->Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator->Ps[i];
        Quaterniond R = Quaterniond(estimator->Rs[i]);

        nav_msgs::msg::Odometry odometry;
        odometry.header = estimator->Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "vins_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        estimator->pub_keyframe_pose->publish(odometry);

        sensor_msgs::msg::PointCloud2 point_cloud;
        point_cloud.header = estimator->Headers[WINDOW_SIZE - 2];
        pcl::PointCloud<PointFeature> feature_cloud;
        for (auto &it_per_id : estimator->feature_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if (it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator->Rs[imu_i] * (estimator->ric[0] * pts_i + estimator->tic[0]) + estimator->Ps[imu_i];
                // geometry_msgs::msg::Point32 p;
                PointFeature p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;

                p.u = it_per_id.feature_per_frame[imu_j].point.x();
                p.v = it_per_id.feature_per_frame[imu_j].point.y();
                p.velocity_x = it_per_id.feature_per_frame[imu_j].uv.x();
                p.velocity_y = it_per_id.feature_per_frame[imu_j].uv.y();
                p.id = it_per_id.feature_id;
                p.depth = it_per_id.feature_per_frame[imu_j].depth;
                feature_cloud.push_back(p);
            }
        }
        pcl::toROSMsg(feature_cloud, point_cloud);
        estimator->pub_keyframe_point->publish(point_cloud);
    }
}