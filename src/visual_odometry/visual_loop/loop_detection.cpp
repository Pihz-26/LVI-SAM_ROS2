#include "loop_detection.h"

LoopDetector::LoopDetector(std::string node_name): Node(node_name)
{
    pub_match_img = this->create_publisher<sensor_msgs::msg::Image>             (PROJECT_NAME + "/vins/loop/match_image",   3);
    pub_match_msg = this->create_publisher<std_msgs::msg::Float64MultiArray>    (PROJECT_NAME + "/vins/loop/match_frame",   3);
    pub_key_pose  = this->create_publisher<visualization_msgs::msg::MarkerArray>(PROJECT_NAME + "/vins/loop/keyframe_pose", 3);

    sub_image     = this->create_subscription<sensor_msgs::msg::Image>      (IMAGE_TOPIC,                                   30, std::bind(&LoopDetector::image_callback,     this, std::placeholders::_1));
    sub_pose      = this->create_subscription<nav_msgs::msg::Odometry>      (PROJECT_NAME + "/vins/odometry/keyframe_pose",  3, std::bind(&LoopDetector::pose_callback,      this, std::placeholders::_1));
    sub_point     = this->create_subscription<sensor_msgs::msg::PointCloud2>(PROJECT_NAME + "/vins/odometry/keyframe_point", 3, std::bind(&LoopDetector::point_callback,     this, std::placeholders::_1));
    sub_extrinsic = this->create_subscription<nav_msgs::msg::Odometry>      (PROJECT_NAME + "/vins/odometry/extrinsic",      3, std::bind(&LoopDetector::extrinsic_callback, this, std::placeholders::_1));

}


void LoopDetector::image_callback(sensor_msgs::msg::Image::SharedPtr image_msg)
{
    if(!LOOP_CLOSURE)
        return;

    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();

    // detect unstable camera stream
    static double last_image_time = -1;
    if (last_image_time == -1)
        last_image_time = rclcpp::Time(image_msg->header.stamp).seconds();
    else if (rclcpp::Time(image_msg->header.stamp).seconds() - last_image_time > 1.0 || rclcpp::Time(image_msg->header.stamp).seconds() < last_image_time)
    {
        RCLCPP_WARN(this->get_logger(), "image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = rclcpp::Time(image_msg->header.stamp).seconds();
}

void LoopDetector::point_callback(sensor_msgs::msg::PointCloud2::SharedPtr point_msg)
{
    if(!LOOP_CLOSURE)
        return;

    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
}

void LoopDetector::pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
{
    if(!LOOP_CLOSURE)
        return;

    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}

void LoopDetector::extrinsic_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}


void LoopDetector::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void LoopDetector::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
	int loop_index = -1;
    if (flag_detect_loop)
    {
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }

    // check loop if valid using ransan and pnp
	if (loop_index != -1)
	{
        KeyFrame* old_kf = getKeyFrame(loop_index);

        if (cur_kf->findConnection(old_kf))
        {
            std_msgs::msg::Float64MultiArray match_msg;
            match_msg.data.push_back(cur_kf->time_stamp);
            match_msg.data.push_back(old_kf->time_stamp);
            pub_match_msg->publish(match_msg);
        }
	}

    // add keyframe
	keyframelist.push_back(cur_kf);
}

KeyFrame* LoopDetector::getKeyFrame(int index)
{
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

int LoopDetector::detectLoop(KeyFrame* keyframe, int frame_index)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    //first query; then add this frame into database!
    QueryResults ret;
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 200);
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    db.add(keyframe->brief_descriptors);
    //printf("add feature time: %f", t_add.toc());
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result 
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }
    // a good match with its nerghbour
    bool find_loop = false;
    if (ret.size() >= 1 && ret[0].Score > 0.05)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {          
                find_loop = true;
                
                if (DEBUG_IMAGE && 0)
                {
                    int tmp_index = ret[i].Id;
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }

        }
    }
    
    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
    
    if (find_loop && frame_index > 50)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || ((int)ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;

}

void LoopDetector::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;
    }

    db.add(keyframe->brief_descriptors);
}

void LoopDetector::visualizeKeyPoses(double time_cur)
{
    if (keyframelist.empty() || pub_key_pose->get_subscription_count() == 0)
        return;

    visualization_msgs::msg::MarkerArray markerArray;

    int count = 0;
    int count_lim = 10;

    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = "vins_world";
    markerNode.header.stamp = rclcpp::Time(time_cur);
    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerNode.ns = "keyframe_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;

    for (list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin(); rit != keyframelist.rend(); ++rit)
    {
        if (count++ > count_lim)
            break;

        geometry_msgs::msg::Point p;
        p.x = (*rit)->origin_vio_T.x();
        p.y = (*rit)->origin_vio_T.y();
        p.z = (*rit)->origin_vio_T.z();
        markerNode.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    pub_key_pose->publish(markerArray);
}


void LoopDetector::process()
{
    if (!LOOP_CLOSURE)
        return;

    while (rclcpp::ok())
    {
        sensor_msgs::msg::Image::SharedPtr image_msg;
        sensor_msgs::msg::PointCloud2::SharedPtr point_msg;
        nav_msgs::msg::Odometry::SharedPtr pose_msg;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (rclcpp::Time(image_buf.front()->header.stamp).seconds() > rclcpp::Time(pose_buf.front()->header.stamp).seconds())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (rclcpp::Time(image_buf.front()->header.stamp).seconds() > rclcpp::Time(point_buf.front()->header.stamp).seconds())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (rclcpp::Time(image_buf.back()->header.stamp).seconds() >= rclcpp::Time(pose_buf.front()->header.stamp).seconds() 
                && rclcpp::Time(point_buf.back()->header.stamp).seconds() >= rclcpp::Time(pose_buf.front()->header.stamp).seconds())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (rclcpp::Time(image_buf.front()->header.stamp).seconds() < rclcpp::Time(pose_msg->header.stamp).seconds())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (rclcpp::Time(point_buf.front()->header.stamp).seconds() < rclcpp::Time(pose_msg->header.stamp).seconds())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            // skip fisrt few
            static int skip_first_cnt = 0;
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            // limit frequency
            static double last_skip_time = -1;
            if (rclcpp::Time(pose_msg->header.stamp).seconds() - last_skip_time < SKIP_TIME)
                continue;
            else
                last_skip_time = rclcpp::Time(pose_msg->header.stamp).seconds();

            // get keyframe pose
            static Eigen::Vector3d last_t(-1e6, -1e6, -1e6);
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();

            // add keyframe
            if((T - last_t).norm() > SKIP_DIST)
            {
                // convert image
                cv_bridge::CvImageConstPtr ptr;
                if (image_msg->encoding == "8UC1")
                {
                    sensor_msgs::msg::Image img;
                    img.header = image_msg->header;
                    img.height = image_msg->height;
                    img.width = image_msg->width;
                    img.is_bigendian = image_msg->is_bigendian;
                    img.step = image_msg->step;
                    img.data = image_msg->data;
                    img.encoding = "mono8";
                    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
                }
                else
                    ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
                
                cv::Mat image = ptr->image;

                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                pcl::PointCloud<PointFeature> pf_cloud;
                pcl::fromROSMsg(*point_msg, pf_cloud);

                for (const auto& point : pf_cloud)
                {
                    // 提取3D点
                    cv::Point3f p_3d;
                    p_3d.x = point.x;
                    p_3d.y = point.y;
                    p_3d.z = point.z;
                    point_3d.push_back(p_3d);

                    // 提取2D点和ID
                    cv::Point2f p_2d_uv, p_2d_normal;
                    p_2d_normal.x = point.u;        // u和v对应2D点的坐标
                    p_2d_normal.y = point.v;
                    p_2d_uv.x = point.velocity_x;    // velocity_x和velocity_y对应2D点的坐标
                    p_2d_uv.y = point.velocity_y;
                    point_id.push_back(point.id);    // id直接存储

                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                }

                // new keyframe
                static int global_frame_index = 0;
                KeyFrame* keyframe = new KeyFrame(rclcpp::Time(pose_msg->header.stamp).seconds(), global_frame_index, 
                                                  T, R, 
                                                  image,
                                                  point_3d, point_2d_uv, point_2d_normal, point_id);   

                // detect loop
                m_process.lock();
                addKeyFrame(keyframe, 1);
                m_process.unlock();

                visualizeKeyPoses(rclcpp::Time(pose_msg->header.stamp).seconds());

                global_frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
} 

void LoopDetector::new_sequence()
{
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    m_buf.unlock();
}




template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// create keyframe online
KeyFrame::KeyFrame(double _time_stamp, int _index, 
                   Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, 
                   cv::Mat &_image,
		           vector<cv::Point3f> &_point_3d, 
                   vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_norm,
		           vector<double> &_point_id)
{
	time_stamp = _time_stamp;
	index = _index;

	origin_vio_T = _vio_T_w_i;		
	origin_vio_R = _vio_R_w_i;

	image = _image.clone();
    // cv::resize(image, thumbnail, cv::Size(80, 60));
	cv::resize(image, thumbnail, cv::Size(), MATCH_IMAGE_SCALE, MATCH_IMAGE_SCALE);

	point_3d = _point_3d;
	point_2d_uv = _point_2d_uv;
	point_2d_norm = _point_2d_norm;
	point_id = _point_id;

	computeWindowBRIEFPoint();
	computeBRIEFPoint();
	if(!DEBUG_IMAGE)
		image.release();
}

void KeyFrame::computeWindowBRIEFPoint()
{
	for(int i = 0; i < (int)point_2d_uv.size(); i++)
	{
	    cv::KeyPoint key;
	    key.pt = point_2d_uv[i];
	    window_keypoints.push_back(key);
	}
	briefExtractor(image, window_keypoints, window_brief_descriptors);
}

void KeyFrame::computeBRIEFPoint()
{
	const int fast_th = 20; // corner detector response threshold
	if(1)
		cv::FAST(image, keypoints, fast_th, true);
	else
	{
		vector<cv::Point2f> tmp_pts;
		cv::goodFeaturesToTrack(image, tmp_pts, 500, 0.01, 10);
		for(int i = 0; i < (int)tmp_pts.size(); i++)
		{
		    cv::KeyPoint key;
		    key.pt = tmp_pts[i];
		    keypoints.push_back(key);
		}
	}
	briefExtractor(image, keypoints, brief_descriptors);
    
	for (int i = 0; i < (int)keypoints.size(); i++)
	{
		Eigen::Vector3d tmp_p;
		m_camera->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);
		cv::KeyPoint tmp_norm;
		tmp_norm.pt = cv::Point2f(tmp_p.x()/tmp_p.z(), tmp_p.y()/tmp_p.z());
		keypoints_norm.push_back(tmp_norm);
	}
}

bool KeyFrame::searchInAera(const BRIEF::bitset window_descriptor,
                            const std::vector<BRIEF::bitset> &descriptors_old,
                            const std::vector<cv::KeyPoint> &keypoints_old,
                            const std::vector<cv::KeyPoint> &keypoints_old_norm,
                            cv::Point2f &best_match,
                            cv::Point2f &best_match_norm)
{
    cv::Point2f best_pt;
    int bestDist = 128;
    int bestIndex = -1;
    for(int i = 0; i < (int)descriptors_old.size(); i++)
    {

        int dis = HammingDis(window_descriptor, descriptors_old[i]);
        if(dis < bestDist)
        {
            bestDist = dis;
            bestIndex = i;
        }
    }
    //printf("best dist %d", bestDist);
    if (bestIndex != -1 && bestDist < 80)
    {
      best_match = keypoints_old[bestIndex].pt;
      best_match_norm = keypoints_old_norm[bestIndex].pt;
      return true;
    }
    else
      return false;
}

void KeyFrame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
								std::vector<cv::Point2f> &matched_2d_old_norm,
                                std::vector<uchar> &status,
                                const std::vector<BRIEF::bitset> &descriptors_old,
                                const std::vector<cv::KeyPoint> &keypoints_old,
                                const std::vector<cv::KeyPoint> &keypoints_old_norm)
{
    for(int i = 0; i < (int)window_brief_descriptors.size(); i++)
    {
        cv::Point2f pt(0.f, 0.f);
        cv::Point2f pt_norm(0.f, 0.f);
        if (searchInAera(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
          status.push_back(1);
        else
          status.push_back(0);
        matched_2d_old.push_back(pt);
        matched_2d_old_norm.push_back(pt_norm);
    }
}

void KeyFrame::PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                         const std::vector<cv::Point3f> &matched_3d,
                         std::vector<uchar> &status)
{
    cv::Mat r, rvec, t, D, tmp_r;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    Matrix3d R_inital;
    Vector3d P_inital;
    Matrix3d R_w_c = origin_vio_R * node_ptr->qic;
    Vector3d T_w_c = origin_vio_T + origin_vio_R * node_ptr->tic;

    R_inital = R_w_c.inverse();
    P_inital = -(R_inital * T_w_c);

    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    cv::Mat inliers;

    if (CV_MAJOR_VERSION < 3)
        solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 100, inliers);
    else
    {
        if (CV_MINOR_VERSION < 2)
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, sqrt(10.0 / 460.0), 0.99, inliers);
        else
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);
    }

    for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
        status.push_back(0);

    for( int i = 0; i < inliers.rows; i++)
    {
        int n = inliers.at<int>(i);
        status[n] = 1;
    }
}


bool KeyFrame::findConnection(KeyFrame* old_kf)
{
	vector<cv::Point2f> matched_2d_cur, matched_2d_old;
	vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
	vector<cv::Point3f> matched_3d;
	vector<double> matched_id;
	vector<uchar> status;

	matched_3d = point_3d;
	matched_2d_cur = point_2d_uv;
	matched_2d_cur_norm = point_2d_norm;
	matched_id = point_id;

	searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, old_kf->brief_descriptors, old_kf->keypoints, old_kf->keypoints_norm);
	reduceVector(matched_2d_cur, status);
	reduceVector(matched_2d_old, status);
	reduceVector(matched_2d_cur_norm, status);
	reduceVector(matched_2d_old_norm, status);
	reduceVector(matched_3d, status);
	reduceVector(matched_id, status);

	if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
	{
		status.clear();
	    PnPRANSAC(matched_2d_old_norm, matched_3d, status);
	    reduceVector(matched_2d_cur, status);
	    reduceVector(matched_2d_old, status);
	    reduceVector(matched_2d_cur_norm, status);
	    reduceVector(matched_2d_old_norm, status);
	    reduceVector(matched_3d, status);
	    reduceVector(matched_id, status);

        if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
        {
        	if (node_ptr->pub_match_img->get_subscription_count() != 0)
            {
            	int gap = 10;
            	cv::Mat gap_image(thumbnail.size().height, gap, CV_8UC1, cv::Scalar(255, 255, 255));
                cv::Mat gray_img, loop_match_img;
                cv::Mat old_img = old_kf->thumbnail;
                cv::hconcat(thumbnail, gap_image, gap_image);
                cv::hconcat(gap_image, old_img, gray_img);
                cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
                // plot features in current frame
                for(int i = 0; i< (int)matched_2d_cur.size(); i++)
                {
                    cv::Point2f cur_pt = matched_2d_cur[i] * MATCH_IMAGE_SCALE;
                    cv::circle(loop_match_img, cur_pt, 5*MATCH_IMAGE_SCALE, cv::Scalar(0, 255, 0));
                }
                // plot features in previous frame
                for(int i = 0; i< (int)matched_2d_old.size(); i++)
                {
                    cv::Point2f old_pt = matched_2d_old[i] * MATCH_IMAGE_SCALE;
                    old_pt.x += thumbnail.size().width + gap;
                    cv::circle(loop_match_img, old_pt, 5*MATCH_IMAGE_SCALE, cv::Scalar(0, 255, 0));
                }
                // plot lines connecting features
                for (int i = 0; i< (int)matched_2d_cur.size(); i++)
                {
                    cv::Point2f old_pt = matched_2d_old[i] * MATCH_IMAGE_SCALE;
                    old_pt.x += thumbnail.size().width + gap;
                    cv::line(loop_match_img, matched_2d_cur[i] * MATCH_IMAGE_SCALE, old_pt, cv::Scalar(0, 255, 0), 2*MATCH_IMAGE_SCALE, 8, 0);
                }
                // plot text
                int banner_height = (double)100 * MATCH_IMAGE_SCALE;
                cv::Mat notation(banner_height, thumbnail.size().width + gap + thumbnail.size().width, CV_8UC3, cv::Scalar(255, 255, 255));
                putText(notation, "current frame: " + to_string(index), 
                        cv::Point2f(5, banner_height - 5), cv::FONT_HERSHEY_SIMPLEX, 
                        MATCH_IMAGE_SCALE*2, cv::Scalar(255), 2);
                putText(notation, "previous frame: " + to_string(old_kf->index), 
                        cv::Point2f(5 + thumbnail.size().width + gap, banner_height - 5), cv::FONT_HERSHEY_SIMPLEX, 
                        MATCH_IMAGE_SCALE*2, cv::Scalar(255), 2);
                cv::vconcat(notation, loop_match_img, loop_match_img);
                // publish matched image
    	    	sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", loop_match_img).toImageMsg();
                msg->header.stamp = rclcpp::Time(time_stamp);
    	    	node_ptr->pub_match_img->publish(*msg);
            }

            return true;
        }
	}

	return false;
}


int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}