#include "utils.h"


bool TERMINATE = false;


void ProcessDone()
{
    cout << "\n \n \nProcess Done. \n \n \n";
}


/**
 * @brief Get the current time in the format of YY-MM-DD-time. 
 * @return the current time in string format. 
*/
string getTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);
    return buf;
}


double rad2deg(double rad)
{
    double deg = 0.0;
    deg = 180 / M_PI * rad;
    return deg;
}


double deg2rad(double deg)
{
    double rad = 0.0;
    rad = deg * M_PI / 180;
    return rad;
}


void Mike::gpslog()
{
    std::ofstream f;
    string path = Mike::log_path + "/GPSLog.csv";
    f.open(path, ios::out | ios::app);
    // mut.lock();
    f << to_string(Mike::gps_data.timestamp) << ", " \
    << to_string(Mike::gps_data.latitude) << ", " \
    << to_string(Mike::gps_data.longitude) << ", " \
    << to_string(Mike::gps_data.altitude) << "\n"; 
    // mut.unlock();
    f.close();
}


void Mike::gps_callback(const sensor_msgs::msg::NavSatFix &msg)
{
    RCLCPP_INFO(this->get_logger(), "Receiveing messages from topic [%s].......", GPS_TOPIC);
    std::mutex mut;
    mut.lock();
    Mike::gps_data.timestamp = msg.header.stamp.sec * (1.0) + msg.header.stamp.nanosec * NANO;
    Mike::gps_data.latitude = msg.latitude;
    Mike::gps_data.longitude = msg.longitude;
    Mike::gps_data.altitude = msg.altitude;
    gpslog();
    mut.unlock();
    
    if (TERMINATE)
    {
        rclcpp::shutdown();
    }
}


void Mike::odolog()
{
    std::ofstream f;
    string path = Mike::log_path + "/OdoLog.csv";
    f.open(path, ios::out | ios::app);
    f << to_string(Mike::odo_data.timestamp) << ", " \
    << to_string(Mike::odo_data.px) << ", " << to_string(Mike::odo_data.py) << ", " \
    << to_string(Mike::odo_data.pz) << ", " << to_string(Mike::odo_data.ox) << ", " \
    << to_string(Mike::odo_data.oy) << ", " << to_string(Mike::odo_data.oz) << ", " \
    << to_string(Mike::odo_data.ow) << "\n"; 
    f.close();
}


void Mike::odo_callback(const nav_msgs::msg::Odometry &msg)
{
    RCLCPP_INFO(this->get_logger(), "Receiveing messages from topic [%s].......", ODO_TOPIC);
    std::mutex mut;
    mut.lock();
    Mike::odo_data.timestamp = msg.header.stamp.sec * (1.0) + msg.header.stamp.nanosec * NANO;
    Mike::odo_data.px = msg.pose.pose.position.x;
    Mike::odo_data.py = msg.pose.pose.position.y;
    Mike::odo_data.pz = msg.pose.pose.position.z;
    Mike::odo_data.ox = msg.pose.pose.orientation.x;
    Mike::odo_data.oy = msg.pose.pose.orientation.y;
    Mike::odo_data.oz = msg.pose.pose.orientation.z;
    Mike::odo_data.ow = msg.pose.pose.orientation.w;
    odolog();
    mut.unlock();

    if (TERMINATE)
    {
        rclcpp::shutdown();
    }
}


void Mike::vellog()
{
    std::ofstream f;
    string path = Mike::log_path + "/VelLog.csv";
    f.open(path, ios::out | ios::app);
    f << to_string(Mike::vel_data.timestamp) << ", " \
    << to_string(Mike::vel_data.lvx) << ", " << to_string(Mike::vel_data.lvy) << ", " \
    << to_string(Mike::vel_data.lvz) << ", " << to_string(Mike::vel_data.avx) << ", " \
    << to_string(Mike::vel_data.avy) << ", " << to_string(Mike::vel_data.avz) << "\n";
    f.close();
}


void Mike::vel_timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Publishing messages to topic [%s].......", VEL_TOPIC);
    auto msg = geometry_msgs::msg::TwistStamped();
    double rad = deg2rad(0);
    std::mutex mut;
    mut.lock();
    Mike::vel_data.lvx = 0.0;
    Mike::vel_data.avz = rad;
    // Mike::vel_data.avz = 0.2;
    msg.header.stamp = this->get_clock()->now();
    msg.twist.linear.x = Mike::vel_data.lvx;
    msg.twist.linear.y = Mike::vel_data.lvy;
    msg.twist.linear.z = Mike::vel_data.lvz;
    msg.twist.angular.x = Mike::vel_data.avx;
    msg.twist.angular.y = Mike::vel_data.avy;
    msg.twist.angular.z = Mike::vel_data.avz;
    Mike::vel_data.timestamp = msg.header.stamp.sec * (1.0) + msg.header.stamp.nanosec * NANO;
    pubs_1->publish(msg);
    vellog();
    mut.unlock();

    if (TERMINATE)
    {
        rclcpp::shutdown();
    }
}


void Mike::createDir()
{
    string time = getTime();
    string main_path = "/home/mike/RobotLog/";
    // string GSP_file_name = "GPSLog-" + time + ".csv";
    // string Odo_file_name = "OdoLog-" + time + ".csv";
    string file_path = main_path + time;
    Mike::log_path = file_path;

    if (create_directories(file_path))
    {
        printf("\n\nDirectory [%s] is created. \n\n", file_path.c_str());
    }
    else
    {
        printf("\n\nDirectory creation is failed. \n\n");
    }
}


int Communication(std::shared_ptr<Mike> node)
{
    // rclcpp::init(argc, argv);
    // std::shared_ptr<Mike> node = std::make_shared<Mike>();
    // node->createDir();
    rclcpp::spin(node);
    // rclcpp::spin(make_shared<Mike>());
    rclcpp::shutdown();

    TERMINATE = true;

    return 0;
}


/*
* @brief Constrctor of class Map. 
*/
My_Map::My_Map()
{
    width_meter = 200;
    height_meter = 200;
    res = 5;
    width_pixel = width_meter * res;
    height_pixel = height_meter * res;
    My_Map::map_ = cv::Mat(height_pixel, width_pixel, CV_8UC3, cv::Scalar(0, 0, 0));
}


/**
 * @brief Constrctor of class Map. 
 * @param w width of the map in meter.
 * @param h height of the map in meter.
 * @param r resolution of the map. [pixel/meter]
*/
My_Map::My_Map(int w, int h, int r)
{
    width_meter = w;
    height_meter = h;
    res = r;
    width_pixel = width_meter * res;
    height_pixel = height_meter * res;
    My_Map::map_ = cv::Mat(height_pixel, width_pixel, CV_8UC3, cv::Scalar(0, 0, 0));
}


/**
 * @brief Transform the odometry data into 2D position.
 * @param x x data from odometry in meter.
 * @param y y data from odometry in meter.
 * @return current the current pose.
*/
Pose My_Map::map2img(double x, double y)
{
    Pose current;
    current.x_meter = x;
    current.y_meter = y;
    double dx = x - My_Map::startPoint.x_meter;
    double dy = y - My_Map::startPoint.y_meter;

    if (dx >= 0)
    {
        current.x_pixel_map = ceil(x - My_Map::startPoint.x_meter);
    }
    else
    {
        current.x_pixel_map = floor(x - My_Map::startPoint.x_meter);
    }

    if (dy >= 0)
    {
        current.y_pixel_map = ceil(y - My_Map::startPoint.y_meter);
    }
    else
    {
        current.y_pixel_map = floor(y - My_Map::startPoint.y_meter);
    }

    current.x_pixel_img = -current.y_pixel_map + My_Map::width_pixel / 2;
    current.y_pixel_img = -current.x_pixel_map + My_Map::height_pixel / 2;
    return current;
}


/**
 * @brief Update the pose of the robot based on the odometry. 
 * @param number serial number of scene. 
 * @param x x data from odometry in meter. 
 * @param y y data from odometry in meter. 
*/
void My_Map::poseUpdate(int number, double x, double y)
{
    if (number == 0)
    {
        startPoint.x_meter = x;
        startPoint.y_meter = y;
        startPoint.x_pixel_img = width_pixel / 2;
        startPoint.y_pixel_img = height_pixel / 2;
        startPoint.x_pixel_map = 0;
        startPoint.y_pixel_map = 0;
        previousPoint = startPoint;
        currentPoint = startPoint;
        // m.map.at<cv::Vec3b>(m.startPoint.y_pixel_img, m.startPoint.x_pixel_img).val[0] = 255;  // b
        // m.map.at<cv::Vec3b>(m.startPoint.y_pixel_img, m.startPoint.x_pixel_img).val[1] = 255;  // g
        // m.map.at<cv::Vec3b>(m.startPoint.y_pixel_img, m.startPoint.x_pixel_img).val[2] = 255;  // r
        cv::circle(
            My_Map::map_,
            cv::Point(startPoint.x_pixel_img, startPoint.y_pixel_img),
            2,
            cv::Scalar(255, 255, 255),
            FILLED);
        cv::putText(
            My_Map::map_, 
            "Start Point", 
            cv::Point(startPoint.x_pixel_img, 
            startPoint.y_pixel_img), 
            cv::FONT_HERSHEY_PLAIN, 
            0.5, 
            cv::Scalar(255, 255, 255), 
            1);
    }
    else
    {
        previousPoint = currentPoint;
        currentPoint = My_Map::map2img(x, y);
        cv::line(
            map_, 
            cv::Point(currentPoint.x_pixel_img, currentPoint.y_pixel_img), 
            cv::Point(previousPoint.x_pixel_img, previousPoint.y_pixel_img), 
            cv::Scalar(0, 255, 0), 
            2);
    }
}


/**
 * @brief Convert realsense pointcloud to pcl pointcloud.
 * @param points pointcloud in realsense format.
 * @return cloud pointcloud in pcl format.
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points2PCL(const rs2::points& points)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto vert = points.get_vertices();

	for (auto& p : cloud->points)
	{
		p.x = vert->x * (-1);
		p.y = vert->y * (-1);
		p.z = vert->z;
		p.r = 255;
		p.g = 255;
		p.b = 255;
		vert++;
	}

	return cloud;
}


void depth_log(string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    std::ofstream f;
    f.open(path, ios::app | ios::out);
    
    for (auto& p : cloud->points)
    {
        f << to_string(p.x) << ", " << to_string(p.y) << ", " \
        << to_string(p.z) << ", " << to_string(p.r) << ", " \
        << to_string(p.g) << ", " << to_string(p.b) << "\n";
    }

    f.close();
}


void PCL2PLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string path)
{
	pcl::PLYWriter writer;
	writer.write(path, *cloud);
	printf("\n\nThe *.ply file of 3D point cloud is saved! \n\n");
}