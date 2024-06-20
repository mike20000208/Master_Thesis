#pragma once
#include <string>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <chrono>
#include <memory>
#include <thread>
#include <filesystem>
#include <mutex>
#include <cmath>
#include <ctime>
#include <algorithm>

#include <GeographicLib/LambertConformalConic.hpp>
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
#include "pcl-1.10/pcl/segmentation/sac_segmentation.h"
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
using namespace GeographicLib;
namespace fs = std::filesystem;

#define OPENCV_TRAITS_ENABLE_DEPRECATED
#define DEVICE "Intel RealSense D455"
#define LOG_FOLDER "/home/mike/RobotLog/"
#define REPLAY_FOLDER "/home/mike/DatatoAnalyze/"
#define REPLAY_DATE "2024-03-17-18:26:07"
#define DEBUG_FILE "/home/mike/Debug/debug.csv"
#define DEBUG_FOLDER "/home/mike/Debug/"
// #define RECORDING_PATH "/home/mike/Recording/Room005.bag"
#define RECORDING_FOLDER "/home/mike/Recording/"
#define _USE_MATH_DEFINES
#define NANO 1e-9
#define MICRO 1e-6
#define MILLI 1e-3
#define CENTI 1e-2
#define ODO_TOPIC "/my_odo"
#define GPS_TOPIC "/fix"
#define VEL_TOPIC "/capra/remote/direct_velocity"
// #define ODO_TOPIC "/mike/odo"
// #define GPS_TOPIC "/mike/gps"

extern bool TERMINATE;
extern bool isRecording;
extern bool isUseKF;
extern bool isUseWFD;

struct GPS
{
    double timestamp = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    int serial_number = 0;
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
    int serial_number = 0;
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
    int serial_number = 0;
};

struct Img
{
    int number = 0;
    double timestamp = 0.0;
};

struct EulerAngle_
{
    double roll = 0.0;  // about x-axis.
    double yaw = 0.0;   // about z-axis.
    double pitch = 0.0; // about y-axis.
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
    double yaw = 0.0;   // about z-axis.
    double pitch = 0.0; // about y-axis.
    Heading heading;    // as a direction vector.
};

struct Point2D
{
    int x = 0.0;
    int y = 0.0;
};

struct Point3D
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Slice
{
    double score = 0.0;
    cv::Vec3d centroid = cv::Vec3d(0.0, 0.0, 0.0);
    vector<int> indices;
};

struct MyTime
{
    double time;
    bool is_ms;
};

struct Cell
{
    vector<double> height;
    double X = 0.0;
    double Y = 0.0;
    double Z = 0.0;
    int counter = 0;
    bool iaPath = false;
};

struct CellKF
{
    double timestamp = 0.0;
    int iteration = -1;
    double measurement = 0.0;
    double est_state = 0.0;
    double est_cov = 0.0;
    double gain = 0.0;
    double pre_state = 0.0;
    double pre_cov = 0.0;
    bool isPath = false;
};


struct CellAStar
{
    int x = 0;  // col. 
    int y = 0;  // row. 
    double f = numeric_limits<double>::infinity();
    double g = numeric_limits<double>::infinity();  // the cost from the start to the cell. 
    // double f = numeric_limits<double>::infinity();  // the heuristic estimation of the cost from the cell to the goal. 
    CellAStar* parent;
    CellAStar() : x(0), y(0), f(numeric_limits<double>::infinity()), g(numeric_limits<double>::infinity()), parent(nullptr) {}
    CellAStar(int x, int y) : x(x), y(y), f(numeric_limits<double>::infinity()), g(numeric_limits<double>::infinity()), parent(nullptr) {}
    bool operator<(const CellAStar& other) const { return f > other.f; }
    bool operator==(const CellAStar& other) const { return x == other.x && y == other.y; }
};


class TEST
{
public:
    int a = 0;
    int b = 10;
    TEST();
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


class Logging
{
public:
    string main_folder;
    string img_folder;
    string traj_folder;
    string depth_folder;
    string map_folder;

    string info_path;
    string mode_path;
    string bag_path;
    string time_path;
    string detailed_time_path;
    string traj_final_path;
    string map_final_path;

    string traj_suffix;
    string img_suffix;
    string depth_suffix;
    string map_suffix;

    string img_path;
    string traj_path;
    string depth_path;
    string map_path;
    string debug_path;

    map<string, int> commands = {
        {"replay_from_images", 1},
        {"trajectory", 2},
        {"stream_map", 3},
        {"single_frame_map", 4},
        {"None", 5},
        {"debug", 6},
        {"replay_from_odometry", 7},
        {"communication", 8},
        {"map_demo", 9},
        {"delay_test", 10},
        {"field_trip", 11},
        {"recording", 12},
        {"stream_map_from_recording", 13}};

    Logging(std::shared_ptr<Mike> node);
    Logging();

    void createDir(string mode);
};


class GridAnalysis
{
public:
    // Pointcloud to process.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    // Grid size. (in meter)
    double cellSize = 1e-3;

    // X and Z border of the pointcloud. (in meter)
    double maxX = -1e6, minX = 1e6;
    double minZ = -1e6, maxZ = 1e6;

    // Threshold of height to determine the traversability. (in meter)
    double heightThreshold = .10;

    // Weights used fro path planning.
    double weight1 = .58;
    double weight2 = .42;
    double weight3 = .00;

    // Grid to store the information of each cell.
    vector<vector<Cell>> grid;

    // Vector to store the results of potential path. 
    vector<pair<int, int>> cells, bestCells;

    // Constructor.
    GridAnalysis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud);

    // Methods to setup the attributes and parameters.
    void setCellSize(double size);
    void setHeightThreshold(double threshold);
    void setWeight1(double weight);
    void setWeight2(double weight);

    // Render the pointcloud based on the height.
    void rendering();

    // Divide the pointcloud into a grid.
    void divide();

    // Scale to score. 
    void scale(vector<double> &scores, vector<double> data, string mode);

    // Score the cells based on the first criterion.
    vector<double> firstCriterion();

    // Score the cells based on the second criterion.
    vector<double> secondCriterion(int iter);

    // Score the cells based on the third criterion.
    vector<double> thirdCriterion();

    // Find the path on the divided grid.
    void findPath(bool isGlobalInvolved); 
};


class KF
{
    // // Measurement error. (in meter)
    // double sigma_34_normal = .08;
    // double sigma_23_normal = .03;
    // double sigma_12_normal = .01;
    // double sigma_01_normal = .005;

    // double sigma_34_latest = .008;
    // double sigma_23_latest = .003;
    // double sigma_12_latest = .001;
    // double sigma_01_latest = .0005;

public:
    // Constructor.
    KF();

    // Select standard deviation for following variance update.
    static double selectSigma(double z);

    // Select process noise to add to the KF.
    static double selectProcessNoise(double timeSpan);

    // Update the Kalman gain.
    static double updateKalmanGain(double predictedCov, double measuredCov);

    // Update the estimated state.
    static double updateState(double gain, double measurement, double priorState);

    // Update the variance of the estimated state.
    static double updateCov(double gain, double priorCov);
};


class My_Map
{
private:
    // The offsets to check the neighbors. (using 8-connection method)
    int connectivity8 = 8;
    int dx8[8] = {0, 0, 1, 1, 1, -1, -1, -1};
    int dy8[8] = {1, -1, 0, 1, -1, -1, 0, 1};

    int connectivity4 = 4;
    int dx4[4] = {0, 1, 0, -1};
    int dy4[4] = {-1, 0, 1, 0};

public:
    // size of map in pixel.
    int width_pixel;
    int height_pixel;

    // size of map in meter.
    int width_meter;
    int height_meter;

    // resolution of map. [pixel/meter]
    int res;

    // Threshold of height to determine the traversability.
    double height_threshold = .10; // in meter.

    // Map itself.
    cv::Mat map_;

    // Temporary map for drawing an arrow indicating the heading.
    cv::Mat tempMap;

    // Info map that stores the projection info.
    // cv::Mat infoMap;
    vector<vector<CellKF>> infoMap;
    vector<vector<double>> AStarMap;

    // Poses.
    Pose startPoint;
    Pose previousPoint;
    Pose currentPoint;

    /**
     * Centroid of the cell being projected. Will be in camera frame first,
     * then converted to robot frame. follewed by being converted to map frame.
     */
    cv::Vec3d center_cam;
    cv::Vec3d center_map;
    cv::Vec3d center_robot;

    // Flag to check whether the area is already transformed to map frame.
    bool isTransformed = false;

    //
    bool isOriginShown = false;

    //
    bool isHeadingShown = false;

    //
    bool isRendered = false;

    //
    bool isPosShown = false;

    // Flag of is map or trajectory.
    bool isMap = false;

    /**
     * List used in frontier search.
     *
     * Especially, the key of this map is the coordinate of the cell (row, col) in info map or map,
     * and the value is its status.
     */
    set<pair<int, int>> Map_Open;
    set<pair<int, int>> Map_Close;
    set<pair<int, int>> Frontier_Open;
    set<pair<int, int>> Frontier_Close;

    /**
     * Queues or other containers used in frontier search.
     *
     * Especially, each element is the coordinate of the cell (row=first, col=second) in info map or map.
     */
    queue<pair<int, int>> map_queue;
    queue<pair<int, int>> frontier_queue;
    vector<pair<int, int>> new_frontier;
    vector<pair<int, int>> frontiers;  // used to save the centroid of the found frontier. 
    pair<int, int> frontier;
    // pair<int, int> frontierCentroids;

    // Path found by A*.
    vector<CellAStar> cellsCollection;
    // vector<CellAStar> path;
    vector<pair<int, int>> path;

    // Map to reconstruct the path.
    map<pair<int, int>, pair<int, int>> came_from;

    // Score records. 
    map<pair<int, int>, double> gScore, fScore;

    // Constructor.
    My_Map();
    My_Map(int w, int h, int r, bool isMap = false);

    // Initialize the info map. (only the time)
    void initialize(double timestamp);

    // Get the orientation.
    EulerAngle_ Quater2Euler(Quaternion_ q);

    // Extract the heading.
    Heading getHeading(EulerAngle_ e);

    // Coordinate transformation method (map to image).
    void map2img(Pose &p);
    Point2D map2img(Point2D p);

    // Coordinate transformation method (camera to map).
    void cam2map();

    // Project the cell on the map
    double cellProject(double height, double depth, double timestamp);

    // Project the found path on the map.
    void cellProjectForPath();

    // Get the current pose of the robot
    Pose getCurrent(double x, double y, EulerAngle_ e);

    // Pose update method.
    void poseUpdate(int number, double x, double y, Quaternion_ q);

    // Map info update method.
    void mapUpdate(GridAnalysis &G, double timestamp); // the faster method.

    // Path update method
    void pathUpdate(GridAnalysis &G);

    // Reconstruct the path by the end of it. 
    void reconstructPath(pair<int, int> end);
    // vector<CellAStar> reconstructPath(CellAStar end);

    // Calculate the A* distance from the found path. 
    // double getAStarDistance(vector<CellAStar> path);
    double getAStarDistance();

    // Check the bounday violation. 
    bool boundaryCheck(CellAStar cell);

    // Estimate the the heuristic cost from the cell to the goal. 
    double getHeuristic(CellAStar cell, CellAStar goal);

    // Perform the A* alrorithm either to filter the foound frontier or find the path. 
    // bool AStar(CellAStar start, CellAStar goal);
    CellAStar AStar(CellAStar start, CellAStar goal);
    // pair<int, int> AStar(CellAStar start, CellAStar goal);

    // Find the path with the filtered frontier. (haven't done yet. )
    void findPath();

    // Show the heading.
    void headingShow();

    // Show the origin.
    void originShow();

    // Render the map with the info map.
    void renderingFromInfoMap();

    // Show the current location.
    void locShow();

    // Show the map.
    void mapShow();

    // Reset all the flags.
    void flagReset();

    // Determine whether this cell is qualified to be the frontier.
    bool isFrontierCell(pair<int, int> cell);

    // Check if this cell has at least one open-space neighbor.
    bool hasLeastOneOpenSpaceNeighbor(pair<int, int> cell);

    // Find the frontier in the current map.
    void findFrontier();

    // Select the frontier with the minimal A* distance. 
    void selectFrontier();

    // Calculate the centroid (median) of the found frontier. 
    pair<int, int> getCentroid();

    // Show the frontier on the map.
    void frontierShow();
};


double rad2deg(double rad);


double deg2rad(double deg);


void ProcessDone();


int Communication(std::shared_ptr<Mike> node);


string getTime();


pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points2PCL(const rs2::points &points);


void depth_log(string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


void PCL2PLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string path);


pcl::visualization::PCLVisualizer::Ptr
Visualization(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers,
              Scalar color,
              string window = "3D viewer");


int getFilesNum(string folder);


double get_mean(vector<double> data);


double get_median(vector<double> data);


double get_mode(vector<double> data);


double getDistance(vector<double> data);


MyTime getDuration(clock_t start, clock_t end, string path, bool isLast = false);


double getActualDuration(double duration);


void changeTest(TEST &t);
