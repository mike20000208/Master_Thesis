#pragma once
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <chrono>
#include <memory>
#include <thread>
#include <filesystem>
#include <mutex>
#include <cmath>

#include "time.h"
#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "librealsense2/rs.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "pcl-1.10/pcl/io/pcd_io.h"
#include "pcl-1.10/pcl/io/ply_io.h"
#include "pcl-1.10/pcl/ModelCoefficients.h"
#include "pcl-1.10/pcl/console/parse.h"
#include "pcl-1.10/pcl/common/io.h"
#include "pcl-1.10/pcl/point_types.h"
#include "pcl-1.10/pcl/point_cloud.h"
#include "pcl-1.10/pcl/sample_consensus/ransac.h"
#include "pcl-1.10/pcl/sample_consensus/sac_model_plane.h"
#include "pcl-1.10/pcl/sample_consensus/method_types.h"
#include "pcl-1.10/pcl/sample_consensus/model_types.h"
// #include "pcl-1.10/pcl/segmentation/sac_segmentation.h"
#include "pcl-1.10/pcl/visualization/pcl_visualizer.h"
#include "pcl-1.10/pcl/visualization/cloud_viewer.h"
#include "pcl-1.10/pcl/filters/passthrough.h"
#include "pcl-1.10/pcl/filters/conditional_removal.h"
// #include "pcl-1.10/pcl/filters/voxel_grid.h"

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace rs2;
using namespace cv;
using namespace pcl;
using namespace Eigen;
using namespace std::chrono;
using namespace std::filesystem;

#define OPENCV_TRAITS_ENABLE_DEPRECATED
#define DEVICE "Intel RealSense D455"
#define REPLAY_FOLDER "/home/mike/DatatoAnalyze/"
#define REPLAY_DATE "2024-03-17-18:26:07"
#define _USE_MATH_DEFINES
#define NANO 1e-9
#define ODO_TOPIC "/my_odo"
#define GPS_TOPIC "/fix"
#define VEL_TOPIC "/capra/remote/direct_velocity"
// #define ODO_TOPIC "/mike/odo"
// #define GPS_TOPIC "/mike/gps"


extern bool TERMINATE;


struct GPS
{
    double timestamp = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
};


struct Odo
{
    double timestamp = 0.0;
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double ox = 0.0;
    double oy = 0.0;
    double oz = 0.0;
    double ow = 0.0;
};


struct Vel
{
    double timestamp = 0.0;
    double lvx = 0.0;
    double lvy = 0.0;
    double lvz = 0.0;
    double avx = 0.0;
    double avy = 0.0;
    double avz = 0.0;
};


struct Img
{
    int number = 0;
    double timestamp = 0.0;
};


struct EulerAngle_
{
    double roll = 0.0;  // about x-axis.
    double yaw = 0.0;  // about z-axis. 
    double pitch = 0.0;  // about y-axis. 
};


struct Quaternion_
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;
};


struct Heading
{
    double x = 0.0;
    double y = 0.0;
    double norm = 0.0;
};


struct Pose
{
    int x_pixel_img = 0;
    int y_pixel_img = 0;
    int x_pixel_map = 0;
    int y_pixel_map = 0;
    double x_meter = 0.0;
    double y_meter = 0.0;
    double roll = 0.0;  // about x-axis.
    double yaw = 0.0;  // about z-axis. 
    double pitch = 0.0;  // about y-axis. 
    Heading heading;  // as a direction vector. 
};

struct Point2D
{
    double x = 0.0;
    double y = 0.0;
};


struct Slice
{
	double score = 0;
	cv::Vec3d centroid = cv::Vec3d(0.0, 0.0, 0.0);
	vector<int> indices;
};


class Mike : public rclcpp::Node
{
public:

    // Node (contains two subscribers)
    Mike() : Node("mike")
    {
        subs_1 = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            GPS_TOPIC,
            10,
            std::bind(
                &Mike::gps_callback,
                this,
                std::placeholders::_1));

        subs_2 = this->create_subscription<nav_msgs::msg::Odometry>(
            ODO_TOPIC,
            10,
            std::bind(
                &Mike::odo_callback,
                this,
                std::placeholders::_1));
        
        pubs_1 = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            VEL_TOPIC, 
            10);

        vel_timer_ = this->create_wall_timer(
            100ms, 
            std::bind(
                &Mike::vel_timer_callback, 
                this));

        createDir();
    }

    // GPD & Odo & Vel data
    GPS gps_data;
    Odo odo_data;
    Vel vel_data;

    // Current log path. 
    string log_path;

    // GPS log method.
    void gpslog();

    // Odo log method.
    void odolog();

    // Vel log method.
    void vellog();

    // Create directories. 
    void createDir();

private:

    // GPS subscriber callback function. 
    void gps_callback(const sensor_msgs::msg::NavSatFix &msg);

    // Odo subscriber callback function. 
    void odo_callback(const nav_msgs::msg::Odometry &msg);

    // Vel publisher callback function.
    void vel_timer_callback();

    // Subscribers. 
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subs_1;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subs_2;

    // Publishers.
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pubs_1;

    // Timer. 
    rclcpp::TimerBase::SharedPtr vel_timer_;
};


class My_Map
{
public:

    // size of map in pixel. 
    int width_pixel;
    int height_pixel;

    // size of map in meter.
    int width_meter;
    int height_meter;

    // resolution of map. [pixel/meter]
    int res;

    // map itself. 
    cv::Mat map_;

    // temporary map for drawing an arrow indicating the heading. 
    cv::Mat tempMap;

    // pose.
    Pose startPoint;
    Pose previousPoint;
    Pose currentPoint;

    // Constructor. 
    My_Map();
    My_Map(int w, int h, int r);

    // Get the orientation. 
    EulerAngle_ Quater2Euler(Quaternion_ q);

    // Extract the heading. 
    Heading getHeading(EulerAngle_ e);

    // Coordinate transformation method.
    void map2img(Pose& p);
    Point2D map2img(Point2D p);

    // Get the current pose of the robot
    Pose getCurrent(double x, double y, EulerAngle_ e);

    // Pose update method. 
    void poseUpdate(int number, double x, double y, Quaternion_ q);

    // Map info update method. 
    void mapUpdate();

};


class Roughness
{
public:
    // Plane model coefficients. 
	double a = 0, b = 0, c = 0, d = 0;

    // the serial numebr of the outliers in the pointcloud. 
	vector<int> outliers;

    // the normalized roughness corresponding to the saved outliers serial number. 
	vector<double> rough;

    // Constructor. 
	Roughness();

    // Constructor with plane model coefficients. 
	Roughness(Eigen::VectorXf& coefficients);

    // Constructor with plane model coefficients. 
	Roughness(pcl::ModelCoefficients& coefficients);

    //
	double get_distance(pcl::PointXYZRGB point);

    //
	void get_Roughness(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
};


class Score
{
public:

	// attributes. 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	double maxX = 0, minX = 0;  // X board
	double x_len = 0;  // length of X
	double stride = 0.3, size = 0.6;  // stride along x-direction, window size
	double search_step = 1.0;  // step along z-direction
	double search_range = 5.0;  // search range within z-direction
	double num_slices = 0;  // number of slice along x-direction
	bool found_boundary = false, found_roi = false;
	double inlier_weight = 0.4, outlier_weight = 1.1, dis_weight = 1.7, angle_weight = 1.1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi;
	vector<Slice> slices;  // slices within specific z range
	cv::Vec3d dir = cv::Vec3d(-1.0, 0.0, 1.0);  // vector pointing the the destination
	vector<Slice> best_paths;  // best slice in each z range

	// methods
	Score(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud);
	void setSearchRange(double z);
	void setSearchStep(double step);
	void setStride(double instride);
	void setSize(double insize);
	void setInlierWeight(double iw);
	void setOutlierWeight(double ow);
	void setDisWeight(double dw);
	void setDir(cv::Vec3d newDir);
	void setAngleWeight(double aw);
	void get_boundary(double z);
	void get_roi(double z);
	void get_slices_1(double z);
	void get_slices_2();
	void get_slices_3(double z);
	double get_angle(cv::Vec3d v);
	static double get_distance(cv::Vec3d v1, cv::Vec3d v2);
	bool get_score(double z, bool have_dst = true);
	bool find_best_path();
	void visualization(pcl::visualization::PCLVisualizer::Ptr viewer, 
		Slice slice);
};


double rad2deg(double rad);


double deg2rad(double deg);


void ProcessDone();


int Communication(std::shared_ptr<Mike> node);


string getTime();


pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points2PCL(const rs2::points& points);


void depth_log(string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


void PCL2PLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string path);


pcl::visualization::PCLVisualizer::Ptr
Visualization(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers,
	Scalar color,
	string window = "3D viewer");


int getFilesNum(string folder);

