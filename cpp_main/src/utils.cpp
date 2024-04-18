#include "utils.h"


bool TERMINATE = false;

bool isEnableFromFile = false;

bool isRecording = false;


/**
 * @brief Shows that the whole program is done. 
*/
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
    strftime(buf, sizeof(buf), "%Y-%m-%d-%H.%M.%S", &tstruct);
    // strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);
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
	<< to_string(Mike::odo_data.serial_number) << ", " \
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
	Mike::odo_data.serial_number = stoi(msg.header.frame_id);
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


/**
 * @brief Constructor of class Logging.
*/
Logging::Logging()
{
	main_folder = LOG_FOLDER + getTime();
	img_folder = main_folder + "/Images";
    traj_folder = main_folder + "/Trajectories";
    depth_folder = main_folder + "/Depth";
    map_folder = main_folder + "/Maps";

    info_path = main_folder + "/Info.txt";
    bag_path = main_folder + "/record.bag";
    time_path = main_folder + "/TimeLog.csv";

    traj_final_path = main_folder + "/Trajectory_final.png";
    map_final_path = main_folder + "/Map_final.png";
}


/**
 * @brief Constructor of class Logging. 
 * @param node aa ROS node responsible for communicating with the Capra robot. 
*/
Logging::Logging(std::shared_ptr<Mike> node)
{
    std::mutex mut;
    mut.lock();

	main_folder = node->log_path;
	img_folder = node->log_path + "/Images";
    traj_folder = node->log_path + "/Trajectories";
    depth_folder = node->log_path + "/Depth";
    map_folder = node->log_path + "/Maps";

    info_path = node->log_path + "/Info.txt";
    bag_path = node->log_path + "/record.bag";
    time_path = node->log_path + "/TimeLog.csv";

    traj_final_path = node->log_path + "/Trajectory_final.png";
    map_final_path = node->log_path + "/Map_final.png";

    mut.unlock();
}


/**
 * @brief Create folders for logging. 
 * @param mode determine which folders are going to be created. 
*/
void Logging::createDir(string mode)
{
	switch (Logging::commands[mode])
	{
        case 1:
        {
			// replay_from_images. 
            break;
        }

        case 2:
        {
			// trajectory
			if (create_directories(img_folder) && create_directories(traj_folder))
			{
				printf("\n\nDirectories are created. \n\n");
			}
			else
			{
				printf("\n\nDirectory creation is failed. \n\n");
			}
            break;
        }

        case 3:
        {
			// stream_map
			if (create_directories(img_folder) && 
			create_directories(traj_folder) && 
			create_directories(depth_folder) && 
			create_directories(map_folder))
			{
				printf("\n\nDirectories are created. \n\n");
			}
			else
			{
				printf("\n\nDirectory creation is failed. \n\n");
			}
            break;
        }

        case 4:
        {
			// single_frame_map
			if (create_directories(img_folder) && 
			create_directories(traj_folder) && 
			create_directories(depth_folder) && 
			create_directories(map_folder))
			{
				printf("\n\nDirectories are created. \n\n");
			}
			else
			{
				printf("\n\nDirectory creation is failed. \n\n");
			}
            break;
        }

        case 5:
        {
			// None
            break;
        }

        case 6:
        {
			// debug
			if (create_directories(img_folder) && 
			create_directories(traj_folder) && 
			create_directories(depth_folder) && 
			create_directories(map_folder))
			{
				printf("\n\nDirectories are created. \n\n");
			}
			else
			{
				printf("\n\nDirectory creation is failed. \n\n");
			}
            break;
        }

        case 7:
        {
			// replay_from_odometry
            break;
        }

        case 8:
        {
			// coomunication
            break;
        }

        case 9:
        {
			// map_demo
            break;
        }

        case 10:
        {
			// delay_test
			if (
				create_directories(img_folder) && 
				create_directories(traj_folder))
			{
				printf("\n\nDirectories are created. \n\n");
			}
			else
			{
				printf("\n\nDirectory creation is failed. \n\n");
			}

    		debug_path = Logging::main_folder + "/delay_test_scene_and_trajectory.csv";
            break;
        }

        case 11:
        {
			// field_trip
			if (create_directories(img_folder) && 
			create_directories(traj_folder) && 
			create_directories(depth_folder) && 
			create_directories(map_folder))
			{
				printf("\n\nDirectories are created. \n\n");
			}
			else
			{
				printf("\n\nDirectory creation is failed. \n\n");
			}
            break;
        }

		case 12:
		{
			// Only recording. 
			if (create_directories(img_folder))
			{
				printf("\n\nDirectories is created. \n\n");
			}
			else
			{
				printf("\n\nDirectory creation is failed. \n\n");
			}
			break;
		}

        default:
        {
            break;
        }
	}
}



/**
 * @brief Constrctor of class Map. 
 * 
 * There are two modes of the instance: map and trajectory. (the default mode is trajectory)
 * 
 * Map shows the pose of the robot and the relatively detailed information of the environment. 
 * 
 * Trajectory shows the pose of the robot and the trajactory it went through. 
 * 
*/
My_Map::My_Map()
{
    width_meter = 200;
    height_meter = 200;
    res = 5;
    width_pixel = width_meter * res;
    height_pixel = height_meter * res;
    My_Map::map_ = cv::Mat(
		height_pixel, 
		width_pixel, 
		CV_8UC3, 
		cv::Scalar(150, 150, 150));
	My_Map::isMap = false;
}


/**
 * @brief Constrctor of class Map. 
 * @param w width of the map in meter.
 * @param h height of the map in meter.
 * @param r resolution of the map. [pixel/meter]
 * @param isMap determine whether the mode is map or trajectory. 
*/
My_Map::My_Map(int w, int h, int r, bool isMap)
{
    width_meter = w;
    height_meter = h;
    res = r;
    width_pixel = width_meter * res;
    height_pixel = height_meter * res;
    My_Map::map_ = cv::Mat(
		height_pixel, 
		width_pixel, 
		CV_8UC3, 
		cv::Scalar(150, 150, 150));
	if (isMap)
	{
		/*
		Create a mini map that can store the necessary info for projection. (lower resolution)

		There are 4 channels of this mini map. 

		1st means the x coordinates. 

		2nd means the y coordinates. 

		3rd means if this region is explored. 

		4th means the data of this region. (mostly is the height. sometimes the score)
		*/
		My_Map::miniMap = cv::Mat(
			height_pixel / 2, 
			width_pixel / 2, 
			CV_64FC4, 
			cv::Scalar(0.0, 0.0, 0.0, 0.0));
		
		for (int i = 0; i < My_Map::miniMap.rows; i++)
		{
			for (int j = 0; j < My_Map::miniMap.cols; j++)
			{
				My_Map::miniMap.at<cv::Vec4d>(i, j)[0] = (4 * j + 1) / 2.0;
				My_Map::miniMap.at<cv::Vec4d>(i, j)[1] = (4 * i + 1) / 2.0;
			}
		}

		// /*
		// Create a mini map that can store the necessary info for projection. (higher resolution)

		// There are 2 channels of this mini map. 

		// 1st means if this region is explored. 

		// 2nd means the data of this region. (mostly is the height. sometimes the score) 
		// */
		// My_Map::miniMap = cv::Mat(
		// 	height_pixel, 
		// 	width_pixel, 
		// 	CV_64FC2, 
		// 	cv::Scalar(0.0, 0.0));
	}
	My_Map::isMap = isMap;
}


/**
 * @brief Convert the orientation in quaternion representation to Euler angle. 
 * @param q orientation in quaternion format. 
 * @return the Euler angle converted from the quaternion representation. 
*/
EulerAngle_ My_Map::Quater2Euler(Quaternion_ q)
{
    EulerAngle_ e;

    // calculate rotation matrix elements;
    double r11 = 1 - 2 * (pow(q.y, 2) + pow(q.z, 2));
    double r21 = 2 * (q.x * q.y + q.w * q.z);
    double r31 = 2 * (q.x * q.z + q.w * q.y);
    double r32 = 2 * (q.w * q.x + q.y * q.z);
    double r33 = 1 - 2 * (pow(q.x, 2) + pow(q.y, 2));

    // get the Euler angle. 
    e.yaw = atan2(r21, r11);
    e.roll = atan2(r32, r33);
    e.pitch = -asin(r31);

    return e;
}


/**
 * @brief Extract the heading of the robot from the Euler angles. 
 * @param e Euler angles converted form the orientation in quaternion format. 
 * @return the heading of the robot based on the given Euler angles. 
*/
Heading My_Map::getHeading(EulerAngle_ e)
{
	Heading h;
	h.x = cos(e.yaw);  // the x component of direction vector;
    h.y = sin(e.yaw);  // the y component of direction vector;
    h.norm = sqrt(pow(h.x, 2) + pow(h.y, 2));
    h.x /= h.norm;
    h.y /= h.norm;
	h.x *= My_Map::res * 1.5;
	h.y *= My_Map::res * 1.5;
	return h;
}


/**
 * @brief Transform the odometry data into 2D position. (frame map to frame image)
 * @param p .
 * @return current the current pose.
*/
void My_Map::map2img(Pose& p)
{
	p.x_pixel_img = -p.y_pixel_map + ceil(My_Map::width_pixel / 2);
	p.y_pixel_img = -p.x_pixel_map + ceil(My_Map::height_pixel / 2);
}


/**
 * @brief
 * @param p
 * @return 
*/
Point2D My_Map::map2img(Point2D p)
{
	Point2D point;
	point.x = -p.y + ceil(My_Map::width_pixel / 2);
	point.y = -p.x + ceil(My_Map::height_pixel / 2);
	return point;
}


/**
 * @brief Transform the point in camera frame into map frame. 
 * 
 * Especially, transform the class member top_left and bottom_right into map frame. 
*/
void My_Map::cam2map()
{
	// Convert the corners from camera frame to robot frame. (in meter)
	// My_Map::top_left_robot[0] = My_Map::top_left_cam[2];
	// My_Map::top_left_robot[1] = My_Map::top_left_cam[0];
	// My_Map::top_left_robot[2] = My_Map::top_left_cam[1];
	// My_Map::bottom_right_robot[0] = My_Map::bottom_right_cam[2];
	// My_Map::bottom_right_robot[1] = My_Map::bottom_right_cam[0];
	// My_Map::bottom_right_robot[2] = My_Map::bottom_right_cam[1];
	My_Map::center_robot[0] = My_Map::center_cam[2];
	My_Map::center_robot[1] = My_Map::center_cam[0];
	My_Map::center_robot[2] = My_Map::center_cam[1];

	// Convert the corners from robot frame to map frame. (in meter)
	// My_Map::top_left_map[0] = cos(-My_Map::currentPoint.yaw) * top_left_robot[0] + \
	// sin(-My_Map::currentPoint.yaw) * top_left_robot[1] + \
	// (My_Map::currentPoint.x_meter - My_Map::startPoint.x_meter);

	// My_Map::top_left_map[1] = -sin(-My_Map::currentPoint.yaw) * top_left_robot[0] + \
	// cos(-My_Map::currentPoint.yaw) * top_left_robot[1] + \
	// (My_Map::currentPoint.y_meter - My_Map::startPoint.y_meter);

	// My_Map::top_left_map[2] = My_Map::top_left_robot[2];

	// My_Map::bottom_right_map[0] = cos(-My_Map::currentPoint.yaw) * bottom_right_robot[0] + \
	// sin(-My_Map::currentPoint.yaw) * bottom_right_robot[1] + \
	// (My_Map::currentPoint.x_meter - My_Map::startPoint.x_meter);

	// My_Map::bottom_right_map[1] = -sin(-My_Map::currentPoint.yaw) * bottom_right_robot[0] + \
	// cos(-My_Map::currentPoint.yaw) * bottom_right_robot[1] + \
	// (My_Map::currentPoint.y_meter - My_Map::startPoint.y_meter);

	// My_Map::bottom_right_map[2] = My_Map::bottom_right_robot[2];

	My_Map::center_map[0] = cos(-My_Map::currentPoint.yaw) * center_robot[0] + \
	sin(-My_Map::currentPoint.yaw) * center_robot[1] + \
	(My_Map::currentPoint.x_meter - My_Map::startPoint.x_meter);

	My_Map::center_map[1] = -sin(-My_Map::currentPoint.yaw) * center_robot[0] + \
	cos(-My_Map::currentPoint.yaw) * center_robot[1] + \
	(My_Map::currentPoint.y_meter - My_Map::startPoint.y_meter);

	My_Map::center_map[2] = My_Map::center_robot[2];

	// Make the unit consistant. from meter to pixel. 
	// My_Map::top_left_map *= My_Map::res;
	// My_Map::bottom_right_map *= My_Map::res;
	My_Map::center_map *= My_Map::res;

	// // Make it an absolute location in the map frame. 
	// My_Map::top_left_map += cv::Vec3d(
	// 	My_Map::currentPoint.x_pixel_map, 
	// 	My_Map::currentPoint.y_pixel_map, 
	// 	0);
	// My_Map::bottom_right_map += cv::Vec3d(
	// 	My_Map::currentPoint.x_pixel_map, 
	// 	My_Map::currentPoint.y_pixel_map, 
	// 	0);

	// confirm the transformation is done. 
	My_Map::isTransformed = true;
}


// /**
//  * @brief Transform the point in camera frame into map frame. 
//  * @param p a point in the camera frame. 
// */
// cv::Vec3d My_Map::cam2map(cv::Vec3d p)
// {
// 	cv::Vec3d p_map;
// 	p_map[0] = p[2];
// 	p_map[1] = p[0];
// 	p_map[2] = p[1];
// 	return p_map;
// }


/**
 * @brief Project the slice area on the map and show it. 
 * @param S the instance of class Score which is running the scoring currently. 
 * @param index the index of current slice which is going to be projected on the map. 
*/
void My_Map::sliceProject(Score S, int index)
{
	Point2D top_left_map, bottom_right_map, center_map, top_left_img, bottom_right_img, center_img;
	if (My_Map::isTransformed)
	{
		// make sure the numebers will be integers. but still in the map frame  
		// if (My_Map::top_left_map[0] >= 0)
		// {
		// 	top_left_map.x = ceil(My_Map::top_left_map[0]);
		// }
		// else
		// {
		// 	top_left_map.x = floor(My_Map::top_left_map[0]);
		// }

		// if (My_Map::top_left_map[1] >= 0)
		// {
		// 	top_left_map.y = ceil(My_Map::top_left_map[1]);
		// }
		// else
		// {
		// 	top_left_map.y = floor(My_Map::top_left_map[1]);
		// }

		// if (My_Map::bottom_right_map[0] >= 0)
		// {
		// 	bottom_right_map.x = ceil(My_Map::bottom_right_map[0]);
		// }
		// else
		// {
		// 	bottom_right_map.x = floor(My_Map::bottom_right_map[0]);
		// }

		// if (My_Map::bottom_right_map[1] >= 0)
		// {
		// 	bottom_right_map.y = ceil(My_Map::bottom_right_map[1]);
		// }
		// else
		// {
		// 	bottom_right_map.y = floor(My_Map::bottom_right_map[1]);
		// }

		if (My_Map::center_map[0] >= 0)
		{
			center_map.x = ceil(My_Map::center_map[0]);
		}
		else
		{
			center_map.x = floor(My_Map::center_map[0]);
		}

		if (My_Map::center_map[1] >= 0)
		{
			center_map.y = ceil(My_Map::center_map[1]);
		}
		else
		{
			center_map.y = floor(My_Map::center_map[1]);
		}

		// Convert the corners from map frame to image frame. 
		// top_left_img = My_Map::map2img(top_left_map);
		// bottom_right_img = My_Map::map2img(bottom_right_map);
		center_img = My_Map::map2img(center_map);


		// // draw the area as a rectangle on the map. (rendering with the score in green-scale)
		// int lowG = 130;
		// int highG = 255;
		// int g = 0;
		// g = ((S.slices[index].score - S.maxScore) / (S.minScore - S.maxScore)) * (highG - lowG) + lowG;
		// // g = ((S.slices[index].score - S.minScore) / (S.maxScore - S.minScore)) * (highG - lowG) + lowG;
		// cv::rectangle(
		// 	My_Map::map_,
		// 	cv::Point(top_left_img.x, top_left_img.y),
		// 	cv::Point(bottom_right_img.x, bottom_right_img.y),
		// 	cv::Scalar(0, g, 0),
		// 	-1
		// );

		// draw the area as a rectangle on the map. (rendering with the percentage of inliers. above the threshold will be green, red the othe way)
		// double threshold = 0.05;
		cv::Scalar color;
		double dis = 0.0;
		vector<double> coors;
		bool isFound = false;
		// ofstream f;
		// f.open("/home/mike/Debug/center.csv", ios::app | ios::out);

		// if (S.slices[index].score != 0.0)
		// {
		// 	if (S.slices[index].score >= 0.6)
		// 	{
		// 		color = cv::Scalar(0, 127, 0);
		// 	}
		// 	else
		// 	{
		// 		color = cv::Scalar(0, 0, 127);
		// 	}
		// }
		// else
		// {
		// 	color = cv::Scalar(0, 0, 0);
		// }

		// cv::rectangle(
		// 	My_Map::map_,
		// 	cv::Point(top_left_img.x, top_left_img.y),
		// 	cv::Point(bottom_right_img.x, bottom_right_img.y),
		// 	color,
		// 	-1
		// );

		// cv::circle(
		// 	My_Map::map_,
		// 	cv::Point(
		// 		(top_left_img.x + bottom_right_img.x) / 2, 
		// 		(top_left_img.y + bottom_right_img.y) / 2),
		// 	1,
		// 	color,
		// 	-1
		// );

		// Project the slice on the mini map, avoiding the overlap problem. 
		// Lower resolution case. 
		for (int i = 0; i < My_Map::miniMap.rows; i++)
		{
			for (int j = 0; j < My_Map::miniMap.cols; j++)
			{
				coors.push_back(center_img.x);
				coors.push_back(center_img.y);
				coors.push_back(My_Map::miniMap.at<cv::Vec4d>(i, j)[0]);
				coors.push_back(My_Map::miniMap.at<cv::Vec4d>(i, j)[1]);
				dis = getDistance(coors);
				coors.clear();

				// f << to_string(i) << ", " << to_string(j) << ", " \
				// << to_string(center_img.x) << ", " \
				// << to_string(center_img.y) << ", " \
				// << to_string(My_Map::miniMap.at<cv::Vec4d>(i, j)[0]) << ", " \
				// << to_string(My_Map::miniMap.at<cv::Vec4d>(i, j)[1]) << ", " \
				// << to_string(dis) << "\n";

				
				if (dis <= (sqrt(2) + 1e-3))  // in pixel
				{
					
					if (My_Map::miniMap.at<cv::Vec4d>(i, j)[2] == 0.0)
					{
						My_Map::miniMap.at<cv::Vec4d>(i, j)[2] = 1.0;
					}
					// else
					// {
					// 	My_Map::miniMap.at<cv::Vec4d>(i, j)[3] += S.slices[index].score;
					// 	My_Map::miniMap.at<cv::Vec4d>(i, j)[3] /= 2;
					// }
					My_Map::miniMap.at<cv::Vec4d>(i, j)[3] = S.slices[index].score;
					isFound = true;
					break;
				}
			}

			if (isFound)
			{
				break;
			}
		}

		// // Higher resolution case. 
		// if (My_Map::miniMap.at<cv::Vec2d>(center_img.y, center_img.x)[0] == 0.0)
		// {
		// 	My_Map::miniMap.at<cv::Vec4d>(center_img.y, center_img.x)[0] = 1.0;
		// }

		// My_Map::miniMap.at<cv::Vec2d>(center_img.y, center_img.x)[1] = S.slices[index].score;

		// reset the flag. 
		My_Map::isTransformed = false;

		// f.close();
	}
	else
	{
		cerr << "\n\nThe location of area needs to be transformed to map frame first! \n\n";
		exit(-1);
	}
}


/**
 * @brief Project the slice area on the map and show it. (only for debug)
 * 
 * The pointcloud is rendered in colors based on the distance interval, and 
 * 
 * this will project the location and color of that slice on the map. 
 * 
 * @param colors
 * @param c
*/
void My_Map::sliceProject(vector<cv::Vec3i> colors, int c)
{
	Point2D top_left_map, bottom_right_map, top_left_img, bottom_right_img;
	if (My_Map::isTransformed)
	{
		// make sure the numebers will be integers. but still in the map frame  
		if (My_Map::top_left_map[0] >= 0)
		{
			top_left_map.x = ceil(My_Map::top_left_map[0]);
		}
		else
		{
			top_left_map.x = floor(My_Map::top_left_map[0]);
		}

		if (My_Map::top_left_map[1] >= 0)
		{
			top_left_map.y = ceil(My_Map::top_left_map[1]);
		}
		else
		{
			top_left_map.y = floor(My_Map::top_left_map[1]);
		}

		if (My_Map::bottom_right_map[0] >= 0)
		{
			bottom_right_map.x = ceil(My_Map::bottom_right_map[0]);
		}
		else
		{
			bottom_right_map.x = floor(My_Map::bottom_right_map[0]);
		}

		if (My_Map::bottom_right_map[1] >= 0)
		{
			bottom_right_map.y = ceil(My_Map::bottom_right_map[1]);
		}
		else
		{
			bottom_right_map.y = floor(My_Map::bottom_right_map[1]);
		}

		// transform the coordinates of the corners to image frame. 
		top_left_img = My_Map::map2img(top_left_map);
		bottom_right_img = My_Map::map2img(bottom_right_map);

		// draw the area as a rectangle on the map. 
		cv::rectangle(
			My_Map::map_,
			cv::Point(top_left_img.x, top_left_img.y),
			cv::Point(bottom_right_img.x, bottom_right_img.y),
			cv::Scalar(colors[c][2], colors[c][1], colors[c][0]),
			-1
		);

		// reset the flag. 
		My_Map::isTransformed = false;
	}
	else
	{
		cerr << "\n\nThe location of area needs to be transformed to map frame first! \n\n";
		exit(-1);
	}
}


/**
 * @brief Project the slice area on the map and show it. (only for debug)
 * 
 * The pointcloud is rendered in colors based on the distance interval, and 
 * 
 * this will project the location, color, and number of that slice on the map. 
 * 
 * @param colors
 * @param c
 * @param i
*/
void My_Map::sliceProject(vector<cv::Vec3i> colors, int c, int i)
{
	Point2D top_left_map, bottom_right_map, top_left_img, bottom_right_img;
	if (My_Map::isTransformed)
	{
		// make sure the numebers will be integers. but still in the map frame  
		if (My_Map::top_left_map[0] >= 0)
		{
			top_left_map.x = ceil(My_Map::top_left_map[0]);
		}
		else
		{
			top_left_map.x = floor(My_Map::top_left_map[0]);
		}

		if (My_Map::top_left_map[1] >= 0)
		{
			top_left_map.y = ceil(My_Map::top_left_map[1]);
		}
		else
		{
			top_left_map.y = floor(My_Map::top_left_map[1]);
		}

		if (My_Map::bottom_right_map[0] >= 0)
		{
			bottom_right_map.x = ceil(My_Map::bottom_right_map[0]);
		}
		else
		{
			bottom_right_map.x = floor(My_Map::bottom_right_map[0]);
		}

		if (My_Map::bottom_right_map[1] >= 0)
		{
			bottom_right_map.y = ceil(My_Map::bottom_right_map[1]);
		}
		else
		{
			bottom_right_map.y = floor(My_Map::bottom_right_map[1]);
		}

		// transform the coordinates of the corners to image frame. 
		top_left_img = My_Map::map2img(top_left_map);
		bottom_right_img = My_Map::map2img(bottom_right_map);

		// draw the area as a rectangle on the map. 
		cv::rectangle(
			My_Map::map_,
			cv::Point(top_left_img.x, top_left_img.y),
			cv::Point(bottom_right_img.x, bottom_right_img.y),
			cv::Scalar(colors[c][2], colors[c][1], colors[c][0]),
			-1
		);

		// print the number of slice on the map. 
		cv::putText(
			My_Map::map_,
			to_string(i),
			cv::Point(
				((top_left_img.x + bottom_right_img.x) / 2), 
				((top_left_img.y + bottom_right_img.y) / 2)),
			FONT_HERSHEY_COMPLEX_SMALL,
			0.01,
			cv::Scalar(0, 0, 0)
		);

		// put a circle on the center of the slice. 
		cv::circle(
			My_Map::map_,
			cv::Point(
				((top_left_img.x + bottom_right_img.x) / 2), 
				((top_left_img.y + bottom_right_img.y) / 2)),
			1,
			cv::Scalar(0, 0, 0),
			-1
		);

		// reset the flag. 
		My_Map::isTransformed = false;
	}
	else
	{
		cerr << "\n\nThe location of area needs to be transformed to map frame first! \n\n";
		exit(-1);
	}
}


/**
 * @brief Get the current pose and heading of the robot. 
 * @param x x data from odometry in meter.
 * @param y y data from odometry in meter.
 * @param e Euler angle extracted from the orientation in quaternion format. 
 * @return current the current pose.
*/
Pose My_Map::getCurrent(double x, double y, EulerAngle_ e)
{
	Pose current;
    current.x_meter = x;
    current.y_meter = y;
    current.roll = e.roll;
    current.pitch = e.pitch;
    current.yaw = e.yaw;
	current.heading = My_Map::getHeading(e);
    double dx = x - My_Map::startPoint.x_meter;
    double dy = y - My_Map::startPoint.y_meter;

    if (dx >= 0)
    {
        current.x_pixel_map = ceil(dx * My_Map::res);
    }
    else
    {
        current.x_pixel_map = floor(dx * My_Map::res);
    }

    if (dy >= 0)
    {
        current.y_pixel_map = ceil(dy * My_Map::res);
    }
    else
    {
        current.y_pixel_map = floor(dy * My_Map::res);
    }
	
	My_Map::map2img(current);
	return current;
}


/**
 * @brief Update the pose of the robot based on the odometry. 
 * @param number serial number of scene. 
 * @param x x data from odometry in meter. 
 * @param y y data from odometry in meter. 
*/
void My_Map::poseUpdate(int number, double x, double y, Quaternion_ q)
{
    // get the current Euler angles 
    EulerAngle_ e = My_Map::Quater2Euler(q);

	// get the corresponding heading. 
	Heading h = My_Map::getHeading(e);

    if (number == 0)
    {
        startPoint.x_meter = x;
        startPoint.y_meter = y;
        startPoint.x_pixel_img = ceil(width_pixel / 2);
        startPoint.y_pixel_img = ceil(height_pixel / 2);
        startPoint.x_pixel_map = 0;
        startPoint.y_pixel_map = 0;
        startPoint.roll = e.roll;
        startPoint.pitch = e.pitch;
        startPoint.yaw = e.yaw;
		startPoint.heading = h;
        previousPoint = startPoint;
        currentPoint = startPoint;
        // m.map.at<cv::Vec3b>(m.startPoint.y_pixel_img, m.startPoint.x_pixel_img).val[0] = 255;  // b
        // m.map.at<cv::Vec3b>(m.startPoint.y_pixel_img, m.startPoint.x_pixel_img).val[1] = 255;  // g
        // m.map.at<cv::Vec3b>(m.startPoint.y_pixel_img, m.startPoint.x_pixel_img).val[2] = 255;  // r
        // cv::circle(
        //     My_Map::map_,
        //     cv::Point(startPoint.x_pixel_img, startPoint.y_pixel_img),
        //     (int)(0.6 * My_Map::res),
        //     cv::Scalar(255, 0, 0),
        //     FILLED);
        // cv::putText(
        //     My_Map::map_, 
        //     "Start Point", 
        //     cv::Point(startPoint.x_pixel_img, 
        //     startPoint.y_pixel_img), 
        //     cv::FONT_HERSHEY_PLAIN, 
        //     1.0, 
        //     cv::Scalar(0, 0, 0), 
        //     1);
    }
    else
    {
        previousPoint = currentPoint;
        currentPoint = My_Map::getCurrent(x, y, e);

		// show the trajcectory if it's trajectory mode.
		if (!My_Map::isMap)
		{
			cv::line(
				My_Map::map_, 
            	cv::Point(currentPoint.x_pixel_img, currentPoint.y_pixel_img), 
            	cv::Point(previousPoint.x_pixel_img, previousPoint.y_pixel_img), 
            	cv::Scalar(0, 150, 0), 
            	2);
		}
    }
}


/**
 * @brief Update the map with the info from camera. 
 * @param S an object contains the information to update the map. 
*/
void My_Map::mapUpdate(Score S)
{
	double scale = 1.0;
    for (int i = 0; i < S.slices.size(); i++)
	{
		// Still in camera frame. 
		My_Map::center_cam = S.slices[i].centroid;
		// My_Map::top_left_cam = S.slices[i].centroid + cv::Vec3d((S.size / scale), 0.0, (S.search_step / scale));
		// My_Map::bottom_right_cam = S.slices[i].centroid + cv::Vec3d(-(S.size / scale), 0.0, -(S.search_step / scale));

		// Convert to map frame. 
		My_Map::cam2map();

		// Convert to image frame. 
		My_Map::sliceProject(S, i);
	}
}


/**
 * @brief Update the map with the info from camera. (only for debug)
 * @param S an object contains the information to update the map. 
 * @param colors the colors that can divide the pointcloud based on distance. 
 * @param c the index to decide the color. 
*/
void My_Map::mapUpdate(Score S, vector<cv::Vec3i> colors, int c)
{
	double scale = 1.5;
    for (int i = 0; i < S.slices.size(); i++)
	{
		My_Map::top_left_cam = S.slices[i].centroid + cv::Vec3d((S.size / scale), 0.0, (S.search_step / scale));
		My_Map::bottom_right_cam = S.slices[i].centroid + cv::Vec3d(-(S.size / scale), 0.0, -(S.search_step / scale));
		My_Map::cam2map();
		// My_Map::sliceProject(colors, c, i);
		My_Map::sliceProject(colors, c);
	}
}


/**
 * @brief Visualize the heading on the map. 
*/
void My_Map::headingShow()
{
	My_Map::isHeadingShown = true;

	if (!My_Map::isOriginShown && !My_Map::isRendered)
	{
		My_Map::tempMap = My_Map::map_.clone();
	}

	// // only for debug
	// printf(
	// 	"The heading of current scene = %f [degree], and the corresponding vector = [%.2f, %.2f]", 
	// 	currentPoint.yaw,
	// 	currentPoint.heading.x,
	// 	currentPoint.heading.y);

	double endX = currentPoint.x_pixel_map + currentPoint.heading.x;
	double endY = currentPoint.y_pixel_map + currentPoint.heading.y;
	int endX_int, endY_int;

	if (endX >= 0)
	{
		endX_int = ceil(endX);
	}
	else
	{
		endX_int = floor(endX);
	}

	if (endY >= 0)
	{
		endY_int = ceil(endY);
	}
	else
	{
		endY_int = floor(endY);
	}

	Point2D point_map, point_img;
	point_map.x = endX_int;
	point_map.y = endY_int;
	point_img = My_Map::map2img(point_map);
    cv::arrowedLine(
        My_Map::tempMap,
        cv::Point(currentPoint.x_pixel_img, currentPoint.y_pixel_img),
        cv::Point(point_img.x, point_img.y),
        cv::Scalar(255, 0, 0),
		2,
		8,
		0,
		0.3
    );
}

void My_Map::originShow()
{
	My_Map::isOriginShown = true;

	if (!My_Map::isHeadingShown && !My_Map::isRendered)
	{
		My_Map::tempMap = My_Map::map_.clone();
	}
	
	cv::circle(
		My_Map::tempMap,
		cv::Point(startPoint.x_pixel_img, startPoint.y_pixel_img),
		1,
		cv::Scalar(255, 0, 0),
		FILLED);

	// cv::circle(
	// 	My_Map::tempMap,
	// 	cv::Point(startPoint.x_pixel_img, startPoint.y_pixel_img),
	// 	(int)(0.6 * My_Map::res * 0.5),
	// 	cv::Scalar(255, 0, 0),
	// 	FILLED);
}


/**
 * @brief
*/
void My_Map::renderingFromMiniMap()
{
	My_Map::isRendered = true;
	cv::Scalar color;
	// ofstream f;
	// f.open(DEBUG_FILE, ios::out);

	if (!My_Map::isHeadingShown && !My_Map::isOriginShown)
	{
		My_Map::tempMap = My_Map::map_.clone();
	}

	for (int i = 0; i < My_Map::miniMap.rows; i++)
	{
		for (int j = 0; j < My_Map::miniMap.rows; j++)
		{
			// // Debug
			// f << to_string(i) << ", " << to_string(j) << ", " \
			// << to_string(My_Map::miniMap.at<cv::Vec4d>(i, j)[0]) << ", " \
			// << to_string(My_Map::miniMap.at<cv::Vec4d>(i, j)[1]) << ", " \
			// << to_string(My_Map::miniMap.at<cv::Vec4d>(i, j)[2]) << ", " \
			// << to_string(My_Map::miniMap.at<cv::Vec4d>(i, j)[3]) << "\n";
			
			// Lower resolution case. 
			if (My_Map::miniMap.at<cv::Vec4d>(i, j)[2] == 0.0)
			{
				continue;
			}
			else
			{
				// lower solution case
				if (My_Map::miniMap.at<cv::Vec4d>(i, j)[3] >= -0.10 && 
				My_Map::miniMap.at<cv::Vec4d>(i, j)[3] <= 0.10)
				// if (My_Map::miniMap.at<cv::Vec4d>(i, j)[3] >= 0.6)
				{
					color = cv::Scalar(0, 127, 0);
				}
				else
				{
					color = cv::Scalar(0, 0, 127);
				}
				My_Map::tempMap.at<cv::Vec3b>(2 * i, 2 * j)[0] = color[0];
				My_Map::tempMap.at<cv::Vec3b>(2 * i, 2 * j)[1] = color[1];
				My_Map::tempMap.at<cv::Vec3b>(2 * i, 2 * j)[2] = color[2];

				My_Map::tempMap.at<cv::Vec3b>(2 * i, 2 * j + 1)[0] = color[0];
				My_Map::tempMap.at<cv::Vec3b>(2 * i, 2 * j + 1)[1] = color[1];
				My_Map::tempMap.at<cv::Vec3b>(2 * i, 2 * j + 1)[2] = color[2];

				My_Map::tempMap.at<cv::Vec3b>(2 * i + 1, 2 * j)[0] = color[0];
				My_Map::tempMap.at<cv::Vec3b>(2 * i + 1, 2 * j)[1] = color[1];
				My_Map::tempMap.at<cv::Vec3b>(2 * i + 1, 2 * j)[2] = color[2];

				My_Map::tempMap.at<cv::Vec3b>(2 * i + 1, 2 * j + 1)[0] = color[0];
				My_Map::tempMap.at<cv::Vec3b>(2 * i + 1, 2 * j + 1)[1] = color[1];
				My_Map::tempMap.at<cv::Vec3b>(2 * i + 1, 2 * j + 1)[2] = color[2];
			}

			// // Higher resolution case.
			// if (My_Map::miniMap.at<cv::Vec2d>(i, j)[0] == 0.0)
			// {
			// 	continue;
			// }
			// else
			// {
			// 	// lower solution case
			// 	if (My_Map::miniMap.at<cv::Vec2d>(i, j)[1] >= -0.10 && 
			// 	My_Map::miniMap.at<cv::Vec2d>(i, j)[1] <= 0.10)
			// 	// if (My_Map::miniMap.at<cv::Vec4d>(i, j)[3] >= 0.6)
			// 	{
			// 		color = cv::Scalar(0, 127, 0);
			// 	}
			// 	else
			// 	{
			// 		color = cv::Scalar(0, 0, 127);
			// 	}
			// 	My_Map::tempMap.at<cv::Vec3b>(i, j)[0] = color[0];
			// 	My_Map::tempMap.at<cv::Vec3b>(i, j)[1] = color[1];
			// 	My_Map::tempMap.at<cv::Vec3b>(i, j)[2] = color[2];
			// }
		}
	}
	// f.close();
}


/**
 * @brief
*/
void My_Map::posShow()
{
	if (!My_Map::isHeadingShown && 
	!My_Map::isOriginShown && 
	!My_Map::isRendered)
	{
		My_Map::tempMap = My_Map::map_.clone();
	}

	cv::circle(
		My_Map::tempMap,
		cv::Point(My_Map::currentPoint.x_pixel_img, My_Map::currentPoint.y_pixel_img),
		1,
		cv::Scalar(0, 0, 0),
		-1
	);
}




/**
 * @brief Copy the map information to the one meant to display. 
*/
void My_Map::mapShow()
{
	if (!My_Map::isHeadingShown && 
	!My_Map::isOriginShown && 
	!My_Map::isRendered && 
	!My_Map::isRendered)
	{
		My_Map::tempMap = My_Map::map_.clone();
	}
}


/**
 * @brief 
*/
void My_Map::flagReset()
{
    isOriginShown = false;
    isHeadingShown = false;
    isRendered = false;
	isPosShown = false;
}



/**
 * @brief Constructor of class Roughness.
*/
Roughness::Roughness()
{
	a = 0;
	b = 0;
	c = 0;
	d = 0;
}


/**
 * @brief Constructor of class Roughness.
 * @param coefficients plane model coefficients. 
*/
Roughness::Roughness(Eigen::VectorXf& coefficients)
{
	a = coefficients[0];
	b = coefficients[1];
	c = coefficients[2];
	d = coefficients[3];
}

/**
 * @brief Constructor of class Roughness.
 * @param coefficients plane model coefficients.
*/
Roughness::Roughness(pcl::ModelCoefficients& coefficients)
{
	a = coefficients.values[0];
	b = coefficients.values[1];
	c = coefficients.values[2];
	d = coefficients.values[3];
}


/**
 * @brief Calculate the distance between a point and plane.
 * @param point one point from outliers. 
 * @return distance distance between point and plane.
*/
double Roughness::get_distance(pcl::PointXYZRGB point)
{
	double distance = 0;
	distance = (abs(a * point.x + b * point.y + c * point.z + d)) / (sqrt(pow(a, 2.0) + pow(b, 2.0) + pow(c, 2.0)));
	return distance;
}


/**
 * @brief Calculate the roughness of outliers of single frame. 
 * @param cloud pointcloud with inliers in green. 
*/
void Roughness::get_Roughness(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
	vector<double> temp;
	outliers.clear();
	rough.clear();
	double d = 0;

	for (int i = 0; i < cloud.points.size(); i++)
	{
		if (cloud.points[i].r == 255 && cloud.points[i].b == 255)  // outliers
		{
			d = get_distance(cloud.points[i]);
			temp.push_back(d);
			outliers.push_back(i);
		}
	}
	normalize(temp, rough, 50, 255, cv::NORM_MINMAX, CV_64F);
	// normalize(temp, rough, 0, 255, cv::NORM_MINMAX, CV_64F);
}


/**
 * @brief Constructor of class Score. 
*/
Score::Score(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud)
{
	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*incloud, *cloud);

	// Calculate the height statistics. 
	for (auto& p : (*cloud).points)
	{
		Score::height.push_back(p.y);
	}

	Score::height_mean = Score::get_mean(Score::height);
	Score::height_median = Score::get_median(Score::height);
	Score::height_mode = Score::get_mode(Score::height);

	// Initialize the maximum and minimum score of the whole scanned region. 
	Score::minScore = 999999;
	Score::maxScore = -999999;
}


/**
 * @brief Set the start value along z-axis for the path planner. 
 * @param z start value. 
*/
void Score::setStartZ(double z)
{
	Score::start_z = z;
}


/**
 * @brief Set the searching range for the path planner. 
 * @param z searching range. 
*/
void Score::setSearchRange(double range)
{
	Score::search_range = range;
}


/**
 * @brief Set the searching step for the path planner.
 * @param step searching step.
*/
void Score::setSearchStep(double step)
{
	Score::search_step = step;
}


/**
 * @brief Set the moving stride for the path planner.
 * @param instride stride for slicing.
*/
void Score::setStride(double instride)
{
	Score::stride = instride;
}


/**
 * @brief Set the size of slice for the path planner.
 * @param instride size of slice.
*/
void Score::setSize(double insize)
{
	Score::size = insize;
}


/**
 * @brief Set the inlier-weight parameter of path planning. 
 * @param iw inlier-weight parameter. 
*/
void Score::setInlierWeight(double iw)
{
	Score::inlier_weight = iw;
}


/**
 * @brief Set the outlier-weight parameter of path planning.
 * @param ow outlier-weight parameter.
*/
void Score::setOutlierWeight(double ow)
{
	Score::outlier_weight = ow;
}


/**
 * @brief Set the distance-weight parameter of path planning.
 * @param dw distance-weight parameter.
*/
void Score::setDisWeight(double dw)
{
	Score::dis_weight = dw;
}

/**
 * @brief Set the direction of destinaiton. 
 * @param dir direction of destination. 
*/
void Score::setDir(cv::Vec3d newDir)
{
	Score::dir = newDir;
}


/**
 * @brief Set the angle-weight parameter of path planning.
 * @param aw angle-weight parameter.
*/
void Score::setAngleWeight(double aw)
{
	Score::angle_weight = aw;
}


/**
 * @brief Get the boundary of roi for both get_slice_1() and get_slice_2(). 
 * @param z lower limit of z value.
*/
void Score::get_boundary(double z)
{
	// use the conventional iteration way.
	vector<double> X;

	for (int i = 0; i < Score::cloud->points.size(); i++)
	{
		if ((Score::cloud->points[i].z >= z) && 
		(Score::cloud->points[i].z < (z + Score::search_step)))
		{
			X.push_back(Score::cloud->points[i].x);
		}
	}
	
	cv::minMaxLoc(X, &(Score::minX), &(Score::maxX));
	Score::x_len = Score::maxX - Score::minX;
	Score::num_slices = floor(Score::x_len / Score::stride) - 1;

		// // debug 
		// std::ofstream f;
		// f.open(DEBUG_FILE, ios::out | ios::app);
		// f << to_string(Score::cloud->size()) << "\n";
		// f << to_string(X.size()) << "\n";
		// f.close();

	// set the flag
	Score::found_boundary = true;
}


/**
 * @brief Get the slice along x direction within specified z range. (by conventional iteration method)
*/
void Score::get_slices(double z)
{
	// check the flag. 
	if (!Score::found_boundary)
	{
		cerr << "\n\nNeeds to find the boundary first! \n\n";
		exit(-1);
	}
	else
	{
		Score::found_boundary = false;
	}

	// reset attributes. 
	Score::slices.clear();

	// prepare some variables. 
	vector<int> X, Y, Z;

	// slicing
	for (int i = 0; i < num_slices; i++)
	{
		// prepare some variables. 
		Slice slice;

		// set the condition to pick out the points. 
		for (int k = 0; k < Score::cloud->points.size(); k++)
		{
			if ((Score::cloud->points[k].z >= z) && 
				(Score::cloud->points[k].z < (z + Score::search_step)))
			{
				if ((Score::cloud->points[k].x >= (Score::minX + i * Score::stride)) &&
					(Score::cloud->points[k].x < (Score::minX + i * Score::stride + Score::size)))
				{
					slice.indices.push_back(k);
					X.push_back(Score::cloud->points[k].x);
					Y.push_back(Score::cloud->points[k].y);
					Z.push_back(Score::cloud->points[k].z);
				}
			}
		}

		// check slice is empty or not. 
		if (slice.indices.size() == 0)
		{
			continue;
		}

		// save the slice. 
		slice.score = 0.0;
		cv::Scalar cx = cv::sum(X);
		cv::Scalar cy = cv::sum(Y);
		cv::Scalar cz = cv::sum(Z);
		double len = X.size();
		slice.centroid = cv::Vec3d(double(cx[0] / len), double(cy[0] / len), double(cz[0] / len));
		Score::slices.push_back(slice);

		// reset. 
		X.clear();
		Y.clear();
		Z.clear();
	}
}


/**
 * @brief Calculate the angle between the given angle and desired direction. 
 * @param v the vector pointing the centroid of a slice from the origin. 
 * @return angle the angle between the given angle and desired direction in degree.
*/
double Score::get_angle(cv::Vec3d v)
{
	double angle = 0;
	double norm_v = sqrt(v.ddot(v));
	double norm_dir = sqrt(Score::dir.ddot(Score::dir));
	angle = acos((v.ddot(Score::dir)) / (norm_v * norm_dir));
	angle = angle * 180 / M_PI;
	return angle;
}


/**
 * @brief Calculate the distance between two points. 
 * @param v1 the first point. 
 * @param v2 the second point. 
 * @return distance distance between two points.
*/
double Score::get_distance(cv::Vec3d v1, cv::Vec3d v2)
{
	double distance = 0;
	cv::Vec3d v = v1 - v2;
	distance = sqrt(v.ddot(v));
	return distance;
}


/**
 * @brief Calculate the score for each slice in second_slices. 
 * @param z lower limit of z value. 
 * @param have_dst whether to take the destination into account. 
*/
bool Score::get_score(double z, bool have_dst)
{
	bool is_Zero = false;
	int num_inliers = 0, num_outliers = 0;
	double percentage = 0.0;

	// // want to log the score data. 
	// std::ofstream f;
	// f.open(DEBUG_FILE, ios::out | ios::app);

	if (Score::slices.size() != 0)
	{
		for (int i = 0; i < Score::slices.size(); i++)  // for each slice in this z-range. 
		{
			for (int j = 0; j < Score::slices[i].indices.size(); j++)  // for each point in this slice. 
			{
				if ((*Score::cloud).points[Score::slices[i].indices[j]].g == 127)  // inliers
				{
					// Score::slices[i].score += Score::inlier_weight;

					// test new score system. 
					num_inliers++;
				}
				else if ((*Score::cloud).points[Score::slices[i].indices[j]].g == 0 &&
					(*Score::cloud).points[Score::slices[i].indices[j]].b == 0)  // outliers
				{
					// // the oldest version. 
					// Score::slices[i].score -= 0.3;

					// Score::slices[i].score +=
					// 	((255 - (*Score::cloud).points[Score::slices[i].indices[j]].r) / 255) * Score::outlier_weight;
					
					// test new score system. 
					num_outliers++;
				}
				else
				{
					// Score::slices[i].score = -100000.0;
					continue;
				}
			}

			// check the percentage of inliers. 
			percentage = (num_inliers * (1.0)) / ((num_inliers + num_outliers) * (1.0));
			Score::slices[i].score = percentage;
			num_inliers = 0;
			num_outliers = 0;
			percentage = 0.0;

			// // ckeck if it's the first iteration in z range. 
			// if (z == Score::start_z + Score::search_step)
			// {
			// 	if (Score::slices[i].centroid[0] == 0 && Score::slices[i].centroid[1] == 0)
			// 	{
			// 		Score::slices[i].score += 0.5;
			// 	}
			// }

			// // normalize the score by the number of points. 
			// Score::slices[i].score = (Score::slices[i].score) / (Score::slices[i].indices.size());

			// // check the distance between currrnt slice and the best slice from the last search. 
			// if (Score::best_paths.size() != 0)
			// {
			// 	double distance = get_distance(Score::best_paths[Score::best_paths.size() - 1].centroid, Score::slices[i].centroid);
			// 	double maxD = sqrt(pow(Score::search_step, 2) + pow(Score::x_len, 2));
			// 	double minD = Score::search_step;
			// 	distance = (distance - minD) / (maxD - minD);
			// 	Score::slices[i].score -= distance * Score::dis_weight;
			// }

			// // if have destination to go, then take it into consideration when calculating the score. 
			// if (have_dst)
			// {
			// 	double angle = get_angle(Score::slices[i].centroid);
			// 	Score::slices[i].score -= (abs(angle) / 180) * Score::angle_weight;
			// }

			// // update the minScore and maxScore. 
			// if (Score::slices[i].score > Score::maxScore)
			// {
			// 	Score::maxScore = Score::slices[i].score;
			// }

			// if (Score::slices[i].score < Score::minScore)
			// {
			// 	Score::minScore = Score::slices[i].score;
			// }

			// // logging the score data to debug. 
			// f << to_string(z) << ", " \
			// << to_string(Score::minX) << ", " \
			// << to_string(Score::maxX) << ", " \
			// << to_string(i) << ", " \
			// << to_string(Score::slices[i].centroid[0]) << ", " \
			// << to_string(Score::slices[i].centroid[1]) << ", " \
			// << to_string(Score::slices[i].centroid[2]) << ", " \
			// << to_string(Score::slices[i].score) << ", " \
			// << to_string(Score::slices[i].indices.size()) << "\n";
		}
		// f.close();
		return is_Zero;
	}
	else
	{
		return !is_Zero;
	}
}


/**
 * @brief Get the roughness of each slice to provide the info to the map. 
 * @param z the operating range along z-axis. 
 * @return is this range void or not. 
*/
bool Score::get_roughness(double z)
{
	bool is_Zero = false;
	int num_inliers = 0, num_outliers = 0;
	double percentage = 0.0;

	// want to log the score data. 
	std::ofstream f;
	f.open(DEBUG_FILE, ios::out | ios::app);
	f << "\n\n";

	if (Score::slices.size() != 0)
	{
		for (int i = 0; i < Score::slices.size(); i++)  // for each slice in this z-range. 
		{
			for (int j = 0; j < Score::slices[i].indices.size(); j++)  // for each point in this slice. 
			{
				if ((*Score::cloud).points[Score::slices[i].indices[j]].g == 127)  // inliers
				{
					num_inliers++;
				}
				else if ((*Score::cloud).points[Score::slices[i].indices[j]].g == 0 &&
					(*Score::cloud).points[Score::slices[i].indices[j]].b == 0)  // outliers
				{
					num_outliers++;
				}
				else
				{
					continue;
				}
			}

			// check the percentage of inliers. 
			percentage = (num_inliers * (1.0)) / ((num_inliers + num_outliers) * (1.0));
			Score::slices[i].score = percentage;

			// update the minScore and maxScore. 
			if (Score::slices[i].score > Score::maxScore)
			{
				Score::maxScore = Score::slices[i].score;
			}

			if (Score::slices[i].score < Score::minScore)
			{
				Score::minScore = Score::slices[i].score;
			}

			// logging the score data to debug. 
			f << to_string(z) << ", " \
			<< to_string(Score::minX) << ", " \
			<< to_string(Score::maxX) << ", " \
			<< to_string(i) << ", " \
			<< to_string(Score::slices[i].centroid[0]) << ", " \
			<< to_string(Score::slices[i].centroid[1]) << ", " \
			<< to_string(Score::slices[i].centroid[2]) << ", " \
			<< to_string(Score::slices[i].score) << ", " \
			<< to_string(Score::slices[i].indices.size()) << "\n";
			num_inliers = 0;
			num_outliers = 0;
			percentage = 0.0;
		}
		// f.close();
		return is_Zero;
	}
	else
	{
		return !is_Zero;
	}
}


/**
 * @brief 
 * @param z
*/
bool Score::get_height(double z)
{
	bool is_Zero = false;
	vector<double> height;
	double median = 0.0;

	// ofstream f;
	// f.open(DEBUG_FILE, ios::app | ios::out);

	if (Score::slices.size() != 0)
	{
		for (int i = 0; i < Score::slices.size(); i++)  // for each slice in this z-range. 
		{
			for (int j = 0; j < Score::slices[i].indices.size(); j++)  // for each point in this slice. 
			{
				// height += (*Score::cloud).points[Score::slices[i].indices[j]].y;
				height.push_back((*Score::cloud).points[Score::slices[i].indices[j]].y);
			}
			median = Score::get_median(height);
			Score::slices[i].score = median;
			height.clear();

			// update the minScore and maxScore. 
			if (Score::slices[i].score > Score::maxScore)
			{
				Score::maxScore = Score::slices[i].score;
			}

			if (Score::slices[i].score < Score::minScore)
			{
				Score::minScore = Score::slices[i].score;
			}

			// // logging the score data to debug. 
			// f << to_string(z) << ", " \
			// << to_string(Score::minX) << ", " \
			// << to_string(Score::maxX) << ", " \
			// << to_string(i) << ", " \
			// << to_string(Score::slices[i].centroid[0]) << ", " \
			// << to_string(Score::slices[i].centroid[1]) << ", " \
			// << to_string(Score::slices[i].centroid[2]) << ", " \
			// << to_string(Score::slices[i].score) << ", " \
			// << to_string(Score::slices[i].indices.size()) << "\n";
		}
		// f.close();
		return is_Zero;
	}
	else
	{
		return !is_Zero;
	}
}


/**
 * @brief Find the mean of a set of data saved in a vector. 
 * @param
*/
double Score::get_mean(vector<double> data)
{
	double mean = 0.0;
	double len = static_cast<double>(data.size());
	double sum = reduce(data.begin(), data.end(), 0.0);
	mean = sum / len;
	return mean;
}


/**
 * @brief Find the median of a set of data saved in a vector. 
 * @param
*/
double Score::get_median(vector<double> data)
{
	double median = 0.0;
	int len = static_cast<int>(data.size());
	sort(data.begin(), data.end());

	if (len % 2 == 0)
	{
		median = (data[len / 2] + data[len / 2 + 1]) / 2.0;
	}
	else
	{
		median = data[len / 2 + 1];
	}

	return median;
}


/**
 * @brief Find the mode of a set of data saved in a vector. 
 * @param
*/
double Score::get_mode(vector<double> data)
{
	double mode = 0.0;
	unordered_map<double, int> frequency;
	int maxFrequency = 0;

	for (int i = 0; i < data.size(); i++)
	{
		frequency[data[i]]++;
	}

	for (auto& pair : frequency)
	{
		if (pair.second > maxFrequency)
		{
			maxFrequency = pair.second;
			mode = pair.first;
		}
	}

	return mode;
}


/**
 * @brief Find the max and min height of the pointcloud. 
*/
void Score::get_maxMin()
{
	sort(Score::height.begin(), Score::height.end());
	Score::maxHeight = height[height.size() - 1];
	Score::minHeight = height[0];
}


/**
 * @brief Find the max and min height of the pointcloud. 
*/
void Score::rendering()
{
	for (auto& p : (*Score::cloud).points)
	{
		if (p.y >= -0.10 && p.y <= 0.10)
		{
			p.r = 0;
			p.g = 127;
			p.b = 0;
		}
		else
		{
			p.r = 127;
			p.g = 0;
			p.b = 0;
		}
	}
}


/**
 * @brief Find the best path in the second slices. 
*/
bool Score::find_best_path()
{
	// prepare some vsriables. 
	bool found = true;
	double best_score = -9e9;
	int index = 0;
	Slice best_path;

	if (Score::slices.size() == 0)
	{
		return !found;
	}
	else
	{
		for (int i = 0; i < Score::slices.size(); i++)
		{
			if (Score::slices[i].score >= best_score)
			{
				best_score = Score::slices[i].score;
				index = i;
			}
		}

		best_path.score = best_score;
		best_path.centroid = Score::slices[index].centroid;
		best_path.indices = Score::slices[index].indices;
		Score::best_paths.push_back(best_path);
		return found;
	}
}


/**
 * @brief Visualize the slice to debug. 
 * @param slice the slice or the roi that we want to see. 
*/
void Score::visualization(pcl::visualization::PCLVisualizer::Ptr viewer, 
	Slice slice)
{
	// reset
	(*viewer).removeAllPointClouds();
	
	// create the pointcloud in roi. 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		roi(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud, slice.indices, *roi);

	// visualization. 
	(*viewer).addPointCloud(roi, "debug");
	(*viewer).setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "debug");

	while (!(*viewer).wasStopped())
	{
		(*viewer).spinOnce(1000);
		std::this_thread::sleep_for(100ms);

        int c = cv::waitKey(10);

        if (c == 32 || c ==13)
		{
			printf("\n\nPress Space or Enter to see next slice. \n\n");
			break;
		}

		// if (GetAsyncKeyState(32) || GetAsyncKeyState(13))
		// {
		// 	printf("\n\nPress Space or Enter to see next slice. \n\n");
		// 	break;
		// }
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
	double theta = deg2rad(14.42);  // inclination of camera. 
	double temp_x = 0.0, temp_y = 0.0, temp_z = 0.0;  // temp values for coordinates transformation. 

	// // Debug
	// ofstream f;

	for (auto& p : cloud->points)
	{
		// f.open(DEBUG_FILE, ios::app | ios::out);
		p.x = vert->x * (-1);
		p.y = vert->y * (-1);
		p.z = vert->z;

		temp_x = p.x;
		temp_y = p.y;
		temp_z = p.z;

		// p.x = sin(theta) * temp_y + cos(theta) * temp_z + 15e-3;
		// p.y = temp_x;
		// p.z = cos(theta) * temp_y - sin(theta) * temp_z + 27e-3;

		p.y = cos(theta) * temp_y - sin(theta) * temp_z;
		p.z = sin(theta) * temp_y + cos(theta) * temp_z;

		p.y += 65 * CENTI;
		p.z += 15 * CENTI;

		// f << to_string(p.x) << ", " \
		// << to_string(p.y) << ", " \
		// << to_string(p.z) << "\n";

		// f << to_string(temp_x) << ", " \
		// << to_string(temp_y) << ", " \
		// << to_string(temp_z) << ", " \
		// << to_string(p.x) << ", " \
		// << to_string(p.y) << ", " \
		// << to_string(p.z) << "\n";

		// p.r = 0;
		// p.g = 127;
		// p.b = 0;
		p.r = 255;
		p.g = 255;
		p.b = 255;
		vert++;
		// f.close();
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


/**
 * @brief Save the pointcloud in ply format. 
 * @param cloud
 * @param path
*/
void PCL2PLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string path)
{
	pcl::PLYWriter writer;
	writer.write(path, *cloud);
	printf("\n\nThe *.ply file of 3D point cloud is saved! \n\n");
}


/**
 * @brief Create a simple pointcloud viewer.
 * @param layers multiple pointcloud layers you want to display.
 * @param color color of the viewer's background. 
 * @param window name of the window.
 * @return viewer pointcloud viewer
*/
pcl::visualization::PCLVisualizer::Ptr
Visualization(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> layers,
	Scalar color,
	string window)
{
	pcl::visualization::PCLVisualizer::Ptr
		viewer(new pcl::visualization::PCLVisualizer(window));
	viewer->setBackgroundColor(color[0], color[1], color[2]);
	viewer->setPosition(0, 0);
	// viewer->setPosition(50, 70);
	//viewer->addCoordinateSystem(max(cloud->width, cloud->height), "global");
	//viewer->addCoordinateSystem(10000, "global");
	viewer->addCoordinateSystem(5, "global");
	for (int i = 0; i < layers.size(); i++)
	{
		viewer->addPointCloud(layers[i], to_string(i));
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			4, to_string(i));
	}

	viewer->initCameraParameters();
	return viewer;
}


/**
 * @brief Count the numebr of files in the given folder. 
 * @param folder the path to the folder that you want to count how many files there are. 
 * @return the numebr of files in the given folder. 
*/
int getFilesNum(string folder)
{
    int num_files = 0;
    std::filesystem::path P {folder};

    for (auto& p : std::filesystem::directory_iterator(P))
    {
        num_files++;
    }

    return num_files;
}


/**
 * @brief Get distance between two points. 
 * @param data a vector saving the coordinates of two points. 
 * 
 * 1st element is x1. 
 * 2nd element is y1. 
 * 3rd element is x2. 
 * 4th element is y2. 
 * 
*/
double getDistance(vector<double> data)
{
	double dis = 0.0;
	dis = sqrt(pow((data[0] - data[2]), 2) + pow((data[1] - data[3]), 2));
	return dis;
}



