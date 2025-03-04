// #pragma once

#include "parameters.h"
// #include "keyframe.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "CameraFactory.h"
#include "CataCamera.h"
#include "PinholeCamera.h"

#define SKIP_FIRST_CNT 10
using namespace DVision;
using namespace DBoW2;

extern std::string PROJECT_NAME;
extern std::string IMAGE_TOPIC;

extern int DEBUG_IMAGE;
extern int LOOP_CLOSURE;
extern double MATCH_IMAGE_SCALE;


extern BriefExtractor briefExtractor;

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

#pragma once

#include "parameters.h"

#define MIN_LOOP_NUM 25

using namespace Eigen;
using namespace std;
using namespace DVision;

extern double SKIP_TIME;
extern double SKIP_DIST;

class LoopDetector;

class KeyFrame
{
public:

	double time_stamp; 
	int index;

	// Pose in vins_world
	Eigen::Vector3d origin_vio_T;		
	Eigen::Matrix3d origin_vio_R;

	cv::Mat image;
	cv::Mat thumbnail;

	vector<cv::Point3f> point_3d; 
	vector<cv::Point2f> point_2d_uv;
	vector<cv::Point2f> point_2d_norm;
	vector<double> point_id;

	vector<cv::KeyPoint> keypoints;
	vector<cv::KeyPoint> keypoints_norm;
	vector<cv::KeyPoint> window_keypoints;

	vector<BRIEF::bitset> brief_descriptors;
	vector<BRIEF::bitset> window_brief_descriptors;
	LoopDetector* node_ptr;

	KeyFrame(double _time_stamp, int _index, 
			 Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, 
			 cv::Mat &_image,
			 vector<cv::Point3f> &_point_3d, 
			 vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_normal, 
			 vector<double> &_point_id);

	bool findConnection(KeyFrame* old_kf);
	void computeWindowBRIEFPoint();
	void computeBRIEFPoint();

	int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);

	bool searchInAera(const BRIEF::bitset window_descriptor,
	                  const std::vector<BRIEF::bitset> &descriptors_old,
	                  const std::vector<cv::KeyPoint> &keypoints_old,
	                  const std::vector<cv::KeyPoint> &keypoints_old_norm,
	                  cv::Point2f &best_match,
	                  cv::Point2f &best_match_norm);

	void searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
						  std::vector<cv::Point2f> &matched_2d_old_norm,
                          std::vector<uchar> &status,
                          const std::vector<BRIEF::bitset> &descriptors_old,
                          const std::vector<cv::KeyPoint> &keypoints_old,
                          const std::vector<cv::KeyPoint> &keypoints_old_norm);


	void PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
	               const std::vector<cv::Point3f> &matched_3d,
	               std::vector<uchar> &status);
};



class LoopDetector: public rclcpp::Node
{
public:

	BriefDatabase db;
	BriefVocabulary* voc;

	map<int, cv::Mat> image_pool;

	list<KeyFrame*> keyframelist;

	LoopDetector(std::string node_name);
	void loadVocabulary(std::string voc_path);
	
	void addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
	void addKeyFrameIntoVoc(KeyFrame* keyframe);
	KeyFrame* getKeyFrame(int index);

	void visualizeKeyPoses(double time_cur);

	int detectLoop(KeyFrame* keyframe, int frame_index);

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 		  	   pub_match_img;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 	   pub_match_msg;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_key_pose;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr 		 sub_image;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr	     sub_pose;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  	 sub_extrinsic;
	void process();
	void new_sequence();
	Eigen::Vector3d tic;
	Eigen::Matrix3d qic;

private: 

	void image_callback    (const sensor_msgs::msg::Image::SharedPtr 	   image_msg);
	void point_callback	   (const sensor_msgs::msg::PointCloud2::SharedPtr point_msg);
	void pose_callback 	   (const nav_msgs::msg::Odometry::SharedPtr       pose_msg);
	void extrinsic_callback(const nav_msgs::msg::Odometry::SharedPtr 	   pose_msg);
	queue<sensor_msgs::msg::Image::SharedPtr>     image_buf;
	queue<sensor_msgs::msg::PointCloud2::SharedPtr> point_buf;
	queue<nav_msgs::msg::Odometry::SharedPtr>    pose_buf;

	std::mutex m_buf;
	std::mutex m_process;


	

};
