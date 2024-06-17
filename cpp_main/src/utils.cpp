#include "utils.h"


bool TERMINATE = false;

bool isRecording = false;

bool isUseKF = true;

bool isUseWFD = false;


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
    std::fstream f;
    string path = Mike::log_path + "/GPSLog.csv";
    f.open(path, ios::out | ios::app);
    f << to_string(Mike::gps_data.timestamp) << ", " \
    << to_string(Mike::gps_data.latitude) << ", " \
    << to_string(Mike::gps_data.longitude) << ", " \
    << to_string(Mike::gps_data.altitude) << "\n"; 
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
    std::fstream f;
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
    std::fstream f;
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


/**
 * @brief Create the main folder to save all the logging data. 
*/
void Mike::createDir()
{
    string time = getTime();
    string main_path = "/home/mike/RobotLog/";
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


/**
 * @brief A main function to start a ROS node and perform the communication with the Capra robot. 
*/
int Communication(std::shared_ptr<Mike> node)
{
    rclcpp::spin(node);
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
	mode_path = main_folder + "/Mode.txt";
    bag_path = main_folder + "/record.bag";
    time_path = main_folder + "/TimeLog.csv";
	detailed_time_path = main_folder + "/Time_for_each_component.csv";

    traj_final_path = main_folder + "/Trajectory_final.png";
    map_final_path = main_folder + "/Map_final.png";
}


/**
 * @brief Constructor of class Logging. 
 * @param node A ROS node responsible for communicating with the Capra robot. 
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
	mode_path = node->log_path + "/Mode.txt";
    bag_path = node->log_path + "/record.bag";
    time_path = node->log_path + "/TimeLog.csv";
	detailed_time_path = node->log_path + "/Time_for_each_component.csv";

    traj_final_path = node->log_path + "/Trajectory_final.png";
    map_final_path = node->log_path + "/Map_final.png";

    mut.unlock();
}


/**
 * @brief Create folders for logging. 
 * @param mode Determine which folders are going to be created. 
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

		case 13: 
		{		
			// steaam_map_from_recording. 
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

        default:
        {
            break;
        }
	}
}


/**
 * @brief Constructor of class KF (Kalman filter). 
*/
KF::KF()
{
	;
}


/**
 * @brief Select standard deviation for following variance update. 
 * @param z the z component of the centroid of this cell. (depth, in meter)
 * @param timeSpan the time between now and the last time this cell was updated. (in second)
 * @return measurement error in this depth range. 
*/
double KF::selectSigma(double z)
{
	double sigma = 0.0;

	// Measurement error. (in meter)
    double sigma_34_normal = .08; 
    double sigma_23_normal = .03; 
    double sigma_12_normal = .01; 
    double sigma_01_normal = .005; 

	if (z >= 3.0 && z < 4.0)
	{
		sigma = sigma_34_normal;
	}
	else if (z >= 2.0 && z < 3.0)
	{
		sigma = sigma_23_normal;
	}
	else if (z >= 1.0 && z < 2.0)
	{
		sigma = sigma_12_normal;
	}
	else if (z < 1.0)
	{
		sigma = sigma_01_normal;
	}
	else
	{
		cerr << "\n\nInvalid coordinate of this cell\n\n";
		exit(-1);
	}

	return sigma;
}


// /**
//  * @brief Select standard deviation for following variance update. 
//  * @param z the z component of the centroid of this cell. (depth, in meter)
//  * @param timeSpan the time between now and the last time this cell was updated. (in second)
//  * @return measurement error in this depth range. 
// */
// double KF::selectSigma(double z, double timeSpan)
// {
// 	double sigma = 0.0;

// 	// Measurement error. (in meter)
//     double sigma_34_normal = .08; 
//     double sigma_23_normal = .03; 
//     double sigma_12_normal = .01; 
//     double sigma_01_normal = .005; 

// 	double sigma_34_latest = .008; 
//     double sigma_23_latest = .003; 
//     double sigma_12_latest = .001; 
//     double sigma_01_latest = .0005; 

// 	if (timeSpan > 3)
// 	{
// 		if (z >= 3.0 && z < 4.0)
// 		{
// 			sigma = sigma_34_latest;
// 		}
// 		else if (z >= 2.0 && z < 3.0)
// 		{
// 			sigma = sigma_23_latest;
// 		}
// 		else if (z >= 1.0 && z < 2.0)
// 		{
// 			sigma = sigma_12_latest;
// 		}
// 		else if (z < 1.0)
// 		{
// 			sigma = sigma_01_latest;
// 		}
// 		else
// 		{
// 			cerr << "\n\nInvalid coordinate of this cell\n\n";
// 			exit(-1);
// 		}
// 	}
// 	else
// 	{
// 		if (z >= 3.0 && z < 4.0)
// 		{
// 			sigma = sigma_34_normal;
// 		}
// 		else if (z >= 2.0 && z < 3.0)
// 		{
// 			sigma = sigma_23_normal;
// 		}
// 		else if (z >= 1.0 && z < 2.0)
// 		{
// 			sigma = sigma_12_normal;
// 		}
// 		else if (z < 1.0)
// 		{
// 			sigma = sigma_01_normal;
// 		}
// 		else
// 		{
// 			cerr << "\n\nInvalid coordinate of this cell\n\n";
// 			exit(-1);
// 		}
// 	}

// 	return sigma;
// }


/**
 * @brief Select process noise to add to the KF. 
 * @param timeSpan the time between now and the last time this cell was updated or initialized. (in second)
 * @return a process noise determined by a saturattion function. 
*/
double KF::selectProcessNoise(double timeSpan)
{
	// Define the parameters of the saturation function. 
	double q = 0.0;
	double q_lower_bound = pow(.001, 2);  // square meter. 
	double q_upper_bound = pow(.03, 2);  // square meter 
	double t_lower_bound = 2.0;
	double t_upper_bound = 10.0;
	// double slope = (q_upper_bound - q_lower_bound) / (t_upper_bound - t_lower_bound);
	double slope = pow(.001, 2) * 10 * 5;
	
	// // Saturation function 
	// if (timeSpan < t_lower_bound)
	// {
	// 	q = q_lower_bound;
	// }
	// else if (timeSpan >= t_lower_bound && timeSpan <= t_upper_bound)
	// {
	// 	q = slope * (timeSpan - t_upper_bound) + q_upper_bound;
	// }
	// else
	// {
	// 	q = q_upper_bound;
	// }

	// Piecewise function 
	if (timeSpan <= t_lower_bound)
	{
		q = q_lower_bound;
	}
	else
	{
		q = slope * (timeSpan - t_lower_bound) + q_lower_bound;
	}


	return q;
}


/**
 * @brief Update the Kalman gain. 
 * @param predictedCov
 * @param measuredCov
 * @return updated Kalman gain. 
*/
double KF::updateKalmanGain(double predictedCov, double measuredCov)
{
	double gain = 0.0;
	gain = (predictedCov) / (predictedCov + measuredCov);
	return gain;
}


/**
 * @brief Update the estimated state. 
 * @param gain
 * @param measuremet
 * @param priorState
*/
double KF::updateState(double gain, double measurement, double priorState)
{
	double state = 0.0;
	state = priorState + gain * (measurement - priorState);
	return state;
}


/**
 * @brief Update the variance of the estimated state. 
*/
double KF::updateCov(double gain, double priorCov)
{
	double cov = 0.0;
	cov = (1 - gain) * priorCov;
	return cov;
}


/**
 * @brief Constrctor of class My_Map. 
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
 * @brief Constrctor of class My_Map. 
 * 
 * There are two modes of the instance: map and trajectory. (the default mode is trajectory)
 * 
 * Map shows the pose of the robot and the relatively detailed information of the environment. 
 * 
 * Trajectory shows the pose of the robot and the trajactory it went through. 
 * 
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

	// Create the info map if it's in map mode. Also resize it so that it can share the same size as the map. 
	if (isMap)
	{
		// /**
		//  * Create an info map that can store the necessary info for projection. 
		//  * 
		//  * There are 4 channels of this info map:
		//  * 
		//  * 1st holds the number of exploration of this cell, also indicates which iteration it has gone through. 
		//  * 
		//  * 2nd holds the estimated state, which will be used to represent the height data of this cell. 
		//  * 
		//  * 3rd holds the predicted state, which will be used in the next iteration. 
		//  * 
		//  * 4th holds the predicted covariance. (uncertainty of the measurement (measurement error, in meter, which is sigma).)
		// */
		// My_Map::infoMap = cv::Mat(
		// 	height_pixel, 
		// 	width_pixel, 
		// 	CV_64FC4, 
		// 	cv::Scalar(0.0));	
	

		/**
		 * Create an info map that can store the necessary info for projection. (Kalman filter applied)
		 * 
		 * Each cell in the info map contains the following information: 
		 * 
		 * iteration: holds the number indicating that which iteration it has gone through. 
		 * 
		 * measurement: holds the latest measurement from the camera. (height in my case)
		 * 
		 * est_state: holds the estimated state. (height in my case)
		 * 
		 * est_cov: holds the estimated covariance. 
		 * 
		 * gain: updated Kalman gain. 
		 * 
		 * pre_state: holds the predicted state, which will be used in the next iteration. 
		 * 
		 * pre_cov: holds the predicted covariance. 
		*/
		for (auto& row : My_Map::infoMap)
		{
			row.resize(My_Map::width_pixel);
		}

		My_Map::infoMap.resize(My_Map::height_pixel, vector<CellKF>(My_Map::width_pixel));
	}

	My_Map::isMap = isMap;
}


/**
 * @brief Initialize the info map. 
 * @param timestamp the time for the initialization of the info map. 
*/
void My_Map::initialize(double timestamp)
{
	for (int i = 0; i < My_Map::infoMap.size(); i++)
	{
		for (int j = 0; j < My_Map::infoMap[0].size(); j++)
		{
			My_Map::infoMap[i][j].timestamp = timestamp;
		}
	}
}


/**
 * @brief Convert the orientation in quaternion representation to Euler angle. 
 * @param q orientation in quaternion format. 
 * @return the Euler angle converted from the quaternion representation. 
*/
EulerAngle_ My_Map::Quater2Euler(Quaternion_ q)
{
    EulerAngle_ e;

    // Calculate rotation matrix elements;
    double r11 = 1 - 2 * (pow(q.y, 2) + pow(q.z, 2));
    double r21 = 2 * (q.x * q.y + q.w * q.z);
    double r31 = 2 * (q.x * q.z + q.w * q.y);
    double r32 = 2 * (q.w * q.x + q.y * q.z);
    double r33 = 1 - 2 * (pow(q.x, 2) + pow(q.y, 2));

    // Get the Euler angle. 
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
	h.x *= My_Map::res;
	h.y *= My_Map::res;
	// h.x *= My_Map::res * 1.5;
	// h.y *= My_Map::res * 1.5;
	return h;
}


/**
 * @brief Transform the odometry data into 2D position. (map frame to image frame)
 * @param p the pose the hasn't had the coordinates in image frame. 
*/
void My_Map::map2img(Pose& p)
{
	p.x_pixel_img = -p.y_pixel_map + round(My_Map::width_pixel / 2);
	p.y_pixel_img = -p.x_pixel_map + round(My_Map::height_pixel / 2);
}


/**
 * @brief Perform the transformation from map frame to image frame. (the image used to display the map)
 * @param p the 2D point in map frame. 
 * @return the transformed 2D point. (in image frame)
*/
Point2D My_Map::map2img(Point2D p)
{
	Point2D point;
	point.x = -p.y + round(My_Map::width_pixel / 2);
	point.y = -p.x + round(My_Map::height_pixel / 2);
	return point;
}


/**
 * @brief Transform the point in camera frame into map frame. 
*/
void My_Map::cam2map()
{
	// Convert the center from camera frame to robot frame. (in meter)
	My_Map::center_robot[0] = My_Map::center_cam[2];
	My_Map::center_robot[1] = My_Map::center_cam[0];
	My_Map::center_robot[2] = My_Map::center_cam[1];

	// Convert the corners from robot frame to map frame. (in meter)
	My_Map::center_map[0] = cos(-My_Map::currentPoint.yaw) * center_robot[0] + \
	sin(-My_Map::currentPoint.yaw) * center_robot[1] + \
	(My_Map::currentPoint.x_meter - My_Map::startPoint.x_meter);

	My_Map::center_map[1] = -sin(-My_Map::currentPoint.yaw) * center_robot[0] + \
	cos(-My_Map::currentPoint.yaw) * center_robot[1] + \
	(My_Map::currentPoint.y_meter - My_Map::startPoint.y_meter);

	My_Map::center_map[2] = My_Map::center_robot[2];

	// Make the unit consistant. from meter to pixel. 
	My_Map::center_map *= My_Map::res;

	// confirm the transformation is done. 
	My_Map::isTransformed = true;
}


// /**
//  * @brief Project one the cells in the grid made from class GridAnalysis on the map. 
//  * @param height the height data of this cell.
// */
// void My_Map::cellProject(double height)
// {
// 	// Initialize center point in map frame and image frame. 
// 	Point2D center_map, center_img;

// 	if (My_Map::isTransformed)
// 	{
// 		// Make sure the numebers will be integers. but still in the map frame  
// 		center_map.x = round(My_Map::center_map[0]);
// 		center_map.y = round(My_Map::center_map[1]);

// 		// Convert the corners from map frame to image frame. 
// 		center_img = My_Map::map2img(center_map);

// 		// // Mark the center of the slice on the map. (for debug)
// 		// cv::circle(
// 		// 	My_Map::map_,
// 		// 	cv::Point(center_img.x, center_img.y),
// 		// 	1,
// 		// 	cv::Scalar(255, 255, 255),
// 		// 	-1
// 		// );

// 		// Project the cell on the info map. 
// 		if (My_Map::infoMap.at<cv::Vec4d>(center_img.y, center_img.x)[0] == 0.0)
// 		{
// 			My_Map::infoMap.at<cv::Vec4d>(center_img.y, center_img.x)[0] = 1.0;
// 		}

// 		My_Map::infoMap.at<cv::Vec4d>(center_img.y, center_img.x)[1] = height;			

// 		// Reset the flag. 
// 		My_Map::isTransformed = false;
// 	}
// 	else
// 	{
// 		cerr << "\n\nThe location of area needs to be transformed to map frame first! \n\n";
// 		exit(-1);
// 	}
// }


/**
 * @brief Project one of the cells in the grid made from class GridAnalysis on the info map. 
 * 
 * Especially, a one-dimensional Kalman filter is applied. The filter takes the measurement
 * 
 * from the camera (depth and height) as inputs and outputs the examinated height. 
 * 
 * @param height the height data of this cell. (as the measurement in the KF)
 * @param depth the depth data of this cell. (relative to the robot in the camera frame) (used to determine the variance)
 * @param timestamp the time when this cell is updated. (in second)
*/
double My_Map::cellProject(double height, double depth, double timestamp)
{
	// Initialize center point in map frame and image frame. 
	Point2D center_map, center_img;

	// Initialize general variables. 
	double sigma = 0.0, variance = 0.0, gain = 0.0, state = 0.0, cov = 0.0, timeSpan = 0.0, noise = 0.0;

	if (My_Map::isTransformed)
	{
		// Make sure the numebers will be integers. but still in the map frame  
		center_map.x = round(My_Map::center_map[0]);
		center_map.y = round(My_Map::center_map[1]);

		// Convert the corners from map frame to image frame. 
		center_img = My_Map::map2img(center_map);

		// // Mark the center of the slice on the map. (for debug)
		// cv::circle(
		// 	My_Map::map_,
		// 	cv::Point(center_img.x, center_img.y),
		// 	1,
		// 	cv::Scalar(255, 255, 255),
		// 	-1
		// );

		// Project the cell on the info map. 
		if (!isUseKF)
		{
			// Method without KF. 
			My_Map::infoMap[center_img.y][center_img.x].iteration += 1;
			My_Map::infoMap[center_img.y][center_img.x].est_state = height;
		}
		else
		{
			// Method with KF. 
			My_Map::infoMap[center_img.y][center_img.x].iteration += 1;

			// Check the time span. 
			timeSpan = timestamp - infoMap[center_img.y][center_img.x].timestamp;  // in second. 
			infoMap[center_img.y][center_img.x].timestamp = timestamp;

			// Measurement. 
			My_Map::infoMap[center_img.y][center_img.x].measurement = height;
			sigma = KF::selectSigma(depth);
			// sigma = KF::selectSigma(depth, timeSpan);
			variance = pow(sigma, 2);
			noise = KF::selectProcessNoise(timeSpan);

			if (My_Map::infoMap[center_img.y][center_img.x].iteration == 0)  // initialization of KF
			{
				// Initialize the estmated state and its variance with the measurement directly. 
				My_Map::infoMap[center_img.y][center_img.x].est_state = height;
				My_Map::infoMap[center_img.y][center_img.x].est_cov = variance;

				// Predict. 
				My_Map::infoMap[center_img.y][center_img.x].pre_state = height;
				My_Map::infoMap[center_img.y][center_img.x].pre_cov = variance + noise;
			}
			else  // normal iteration. 
			{
				// Update. 
				gain = KF::updateKalmanGain(
					My_Map::infoMap[center_img.y][center_img.x].pre_cov, 
					variance);
				state = KF::updateState(
					gain, 
					height, 
					My_Map::infoMap[center_img.y][center_img.x].pre_state);
				cov = KF::updateCov(
					gain, 
					My_Map::infoMap[center_img.y][center_img.x].pre_cov);
				My_Map::infoMap[center_img.y][center_img.x].gain = gain;
				My_Map::infoMap[center_img.y][center_img.x].est_state = state;
				My_Map::infoMap[center_img.y][center_img.x].est_cov = cov;
				
				// Predict. 	
				My_Map::infoMap[center_img.y][center_img.x].pre_state = state;
				My_Map::infoMap[center_img.y][center_img.x].pre_cov = cov + noise;
			}
		}

		// Reset the flag. 
		My_Map::isTransformed = false;
	}
	else
	{
		cerr << "\n\nThe location of area needs to be transformed to map frame first! \n\n";
		exit(-1);
	}

	return My_Map::infoMap[center_img.y][center_img.x].est_state;
}


/**
 * @brief Project the cells in best path on the map.
 */
void My_Map::cellProjectForPath()
{
	// Initialize center point in map frame and image frame. 
	Point2D center_map, center_img;

	if (My_Map::isTransformed)
	{
		// Make sure the numebers will be integers. but still in the map frame  
		center_map.x = round(My_Map::center_map[0]);
		center_map.y = round(My_Map::center_map[1]);

		// Convert the center from map frame to image frame. 
		center_img = My_Map::map2img(center_map);

		// Mark the corresponding cell as best path.
		My_Map::infoMap[center_img.y][center_img.x].isPath = true;

		// Reset the flag. 
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
 * @return the current pose.
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

	// Convert the coordinate from meter to pixel. but still in map frame. 
    double dx = x - My_Map::startPoint.x_meter;
    double dy = y - My_Map::startPoint.y_meter;
	current.x_pixel_map = round(dx * My_Map::res);
	current.y_pixel_map = round(dy * My_Map::res);

	// Transformation from map frame to image frame. (the image used to display the map)
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
    // Get the current Euler angles 
    EulerAngle_ e = My_Map::Quater2Euler(q);

	// Get the corresponding heading. 
	Heading h = My_Map::getHeading(e);

    if (number == 0)
    {
        startPoint.x_meter = x;
        startPoint.y_meter = y;
        startPoint.x_pixel_img = round(width_pixel / 2);
        startPoint.y_pixel_img = round(height_pixel / 2);
        startPoint.x_pixel_map = 0;
        startPoint.y_pixel_map = 0;
        startPoint.roll = e.roll;
        startPoint.pitch = e.pitch;
        startPoint.yaw = e.yaw;
		startPoint.heading = h;
        previousPoint = startPoint;
        currentPoint = startPoint;
    }
    else
    {
        previousPoint = currentPoint;
        currentPoint = My_Map::getCurrent(x, y, e);

		// Show the trajcectory if it's in trajectory mode.
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
 * @param G an instance of the class GridAnalysis that contains the information to update the map. 
 * @param timestamp the time when the info map is updated. (in second)
*/
void My_Map::mapUpdate(GridAnalysis &G, double timestamp)
{
	// Initialzie general variables
	double newHeight = 0.0;
	
	// Assign the height threshold. 
	My_Map::height_threshold = G.heightThreshold;

	// Project the grid in class GridAnalysis to the info map. 
	for (int i = 0; i < G.grid.size(); i++)
	{
		for (int j = 0; j < G.grid[0].size(); j++)
		{
			// Check if the current cell is empty. 
			if (G.grid[i][j].counter == 0 || 
			cv::Vec3d(G.grid[i][j].X, G.grid[i][j].Y, G.grid[i][j].Z) == cv::Vec3d(0.0, 0.0, 0.0))
			{
				continue;
			}
			else
			{
				// Get the coordinates of a cell, but still in camera frame. 
				My_Map::center_cam = cv::Vec3d(
					G.grid[i][j].X, 
					G.grid[i][j].Y, 
					G.grid[i][j].Z);

				// Transformation from camera frame to map frame.
				My_Map::cam2map();

				// Transformation from map frame to image frame. (the image used to display the map)
				// My_Map::cellProject(G.grid[i][j].Y);  // the older method, and it's verified working. 
				newHeight = My_Map::cellProject(G.grid[i][j].Y, G.grid[i][j].Z, timestamp);  // the newer method, but still in test. 
				G.grid[i][j].Y = newHeight;
			}
		}
	}
}


/**
 * Update the path found in class GridAnalysis. 
 */
void My_Map::pathUpdate(GridAnalysis &G)
{
	int len = static_cast<int>(G.bestCells.size());

	for (int i = 0; i < len; i++)
	{
		// Get the coordinates of a cell, but still in camera frame. 
		My_Map::center_cam = cv::Vec3d(
			G.grid[G.bestCells[i].first][G.bestCells[i].second].X, 
			G.grid[G.bestCells[i].first][G.bestCells[i].second].Y, 
			G.grid[G.bestCells[i].first][G.bestCells[i].second].Z);

		// Transformation from camera frame to map frame.
		My_Map::cam2map();

		// Transformation from map frame to image frame. (the image used to display the map)
		My_Map::cellProjectForPath();
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

	// // Only for debug
	// printf(
	// 	"The heading of current scene = %f [degree], and the corresponding vector = [%.2f, %.2f]", 
	// 	currentPoint.yaw,
	// 	currentPoint.heading.x,
	// 	currentPoint.heading.y);

	// Get the tip of the arrowed line, but still in map frame. 
	double endX = currentPoint.x_pixel_map + currentPoint.heading.x;
	double endY = currentPoint.y_pixel_map + currentPoint.heading.y;

	// Round the coordinates of the tip in order to display on the image. 
	int endX_int, endY_int;
	endX_int = round(endX);
	endY_int = round(endY);

	// Transformation from map frame to image frame. 
	Point2D point_map, point_img;
	point_map.x = endX_int;
	point_map.y = endY_int;
	point_img = My_Map::map2img(point_map);

	// Display it on the map. 
    cv::arrowedLine(
        My_Map::tempMap,
        cv::Point(currentPoint.x_pixel_img, currentPoint.y_pixel_img),
        cv::Point(point_img.x, point_img.y),
        cv::Scalar(255, 0, 0),
		2,
		8,
		0,
		0.2
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
 * @brief Render the image with the info map to display the map. 
*/
void My_Map::renderingFromInfoMap()
{
	My_Map::isRendered = true;
	cv::Scalar color;
	// fstream f;
	// f.open(string(DEBUG_FOLDER) + string("KF.csv"), ios::out | ios::app);

	if (!My_Map::isHeadingShown && !My_Map::isOriginShown)
	{
		My_Map::tempMap = My_Map::map_.clone();
	}

	// // Method without KF
	// for (int i = 0; i < My_Map::infoMap.rows; i++)
	// {
	// 	for (int j = 0; j < My_Map::infoMap.rows; j++)
	// 	{
	// 		// // Debug
	// 		// f << to_string(i) << ", " << to_string(j) << ", " \
	// 		// << to_string(My_Map::infoMap.at<cv::Vec4d>(i, j)[0]) << ", " \
	// 		// << to_string(My_Map::infoMap.at<cv::Vec4d>(i, j)[1]) << ", " \
	// 		// << to_string(My_Map::infoMap.at<cv::Vec4d>(i, j)[2]) << ", " \
	// 		// << to_string(My_Map::infoMap.at<cv::Vec4d>(i, j)[3]) << "\n";
			
	// 		if (My_Map::infoMap.at<cv::Vec2d>(i, j)[0] == 0.0)
	// 		{
	// 			continue;
	// 		}
	// 		else
	// 		{
	// 			if (My_Map::infoMap.at<cv::Vec2d>(i, j)[1] >= -My_Map::height_threshold && 
	// 			My_Map::infoMap.at<cv::Vec2d>(i, j)[1] <= My_Map::height_threshold)
	// 			{
	// 				color = cv::Scalar(0, 127, 0);
	// 			}
	// 			else
	// 			{
	// 				color = cv::Scalar(0, 0, 127);
	// 			}

	// 			My_Map::tempMap.at<cv::Vec3b>(i, j)[0] = color[0];
	// 			My_Map::tempMap.at<cv::Vec3b>(i, j)[1] = color[1];
	// 			My_Map::tempMap.at<cv::Vec3b>(i, j)[2] = color[2];
	// 		}				
	// 	}
	// }

	// Method with KF. 
	for (int i = 0; i < My_Map::infoMap.size(); i++)
	{
		for (int j = 0; j < My_Map::infoMap[0].size(); j++)
		{
			// // Debug of Kalman filter.
			// f << to_string(i) << ", " << to_string(j) << ", " \
			// << to_string(infoMap[i][j].est_state) << ", " \
			// << to_string(infoMap[i][j].measurement) << ", " \
			// << to_string(infoMap[i][j].gain) << "\n";
			
			if (My_Map::infoMap[i][j].iteration == -1)  // this cell hasn't been explored. 
			{
				continue;
			}
			else
			{
				// Render the map based on the height data. 
				if (My_Map::infoMap[i][j].est_state >= -My_Map::height_threshold && 
				My_Map::infoMap[i][j].est_state <= My_Map::height_threshold)
				{
					color = cv::Scalar(0, 127, 0);
				}
				else
				{
					color = cv::Scalar(0, 0, 127);
				}

				My_Map::tempMap.at<cv::Vec3b>(i, j)[0] = color[0];
				My_Map::tempMap.at<cv::Vec3b>(i, j)[1] = color[1];
				My_Map::tempMap.at<cv::Vec3b>(i, j)[2] = color[2];

				// Render the map based on the path flag. 
				if (My_Map::infoMap[i][j].isPath)
				{
					color = cv::Scalar(168, 50, 168);
					My_Map::tempMap.at<cv::Vec3b>(i, j)[0] = color[0];
					My_Map::tempMap.at<cv::Vec3b>(i, j)[1] = color[1];
					My_Map::tempMap.at<cv::Vec3b>(i, j)[2] = color[2];
				}
			}
		}
	}

	// f.close();
}


/**
 * @brief Show the current location of the robot. 
*/
void My_Map::locShow()
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
 * @brief Reset all the flags inside the class My_Map. 
*/
void My_Map::flagReset()
{
    isOriginShown = false;
    isHeadingShown = false;
    isRendered = false;
	isPosShown = false;

	// Reset the path attribute in the infomap. 
	for (int i = 0; i < static_cast<int>(My_Map::infoMap.size()); i++)
	{
		for (int j = 0; j < static_cast<int>(My_Map::infoMap[0].size()); j++)
		{
			My_Map::infoMap[i][j].isPath = false;
		}
	}
}


/**
 * @brief Determine whether this cell is qualified to be a part of the frontier. 
 * 
 * Especially, a cell can be seen as a part of the frontier only it's unexplored and 
 * 
 * has at least one open-space neighbor. Open-space means a region is explored and obstacle-free.  
 * 
 * Besides, 8-connection method is used to check it's surroundings. 
 * 
 * @param cell the cell is going to be examinated if it's qualified to be the frontier.
 * @return can this cell be seen as a frontier cell. 
*/
bool My_Map::isFrontierCell(pair<int, int> cell)
{
	bool is = false;

	if (My_Map::infoMap[cell.first][cell.second].iteration == -1)  // an unknown cell. 
	{
		for (int i = 0; i < connectivity; i++)
		{
			// Check if the adjacent is open-space. (explored and obstacle-free)
			if (My_Map::infoMap[cell.first + dy[i]][cell.second + dx[i]].iteration != -1)
			{
				if (My_Map::infoMap[cell.first + dy[i]][cell.second + dx[i]].est_state >= -My_Map::height_threshold && 
				My_Map::infoMap[cell.first + dy[i]][cell.second + dx[i]].est_state <= My_Map::height_threshold)
				{
					is = true;
					return is;
				}			
			}
		}
	}
	else
	{
		return is;
	}

	return is;
}


/**
 * @brief Check if this cell has at least one open-space neighbor.
 * @param cell
 * @return does this cell have at least one open-space neighbor. 
*/
bool My_Map::hasLeastOneOpenSpaceNeighbor(pair<int, int> cell)
{
	bool doesHave = false;

	for (int i = 0; i < connectivity; i++)
	{
		if (My_Map::infoMap[cell.first + dy[i]][cell.second + dx[i]].iteration != -1)
		{
			if (My_Map::infoMap[cell.first + dy[i]][cell.second + dx[i]].est_state >= -My_Map::height_threshold && 
			My_Map::infoMap[cell.first + dy[i]][cell.second + dx[i]].est_state <= My_Map::height_threshold)
			{
				doesHave = true;
				return doesHave;
			}			
		}
	}

	return doesHave;
}


// /**
//  * @brief Check the type of a cell.
//  * @param cell
//  * @param types
//  * @param mode
//  * @return 
// */
// bool My_Map::checkCellType(pair<int, int> cell, vector<CellType> types, string mode)
// {
// 	bool isMatched = true;

// 	if (mode == "is")
// 	{
// 		for (int i = 0; i < types.size(); i++)
// 		{
// 			if (!My_Map::cellTypeList[cell][types[i]])
// 			{
// 				isMatched = false;
// 				return isMatched;
// 			}
// 		}
// 	}
// 	else if (mode == "is not")
// 	{
// 		for (int i = 0; i < types.size(); i++)
// 		{
// 			if (My_Map::cellTypeList[cell][types[i]])
// 			{
// 				isMatched = false;
// 				return isMatched;
// 			}
// 		}
// 	}
// 	else
// 	{
// 		cerr << "\n\nInvalid mode! \n\n";
// 		exit(-1);
// 	}

// 	return isMatched;
// }


/**
 * @brief Apply WFD algorithm to find the frontier in the current info map or map. 
 * 
 * Especially, run the BSF twice to find it. Specifically, the first 
 * 
 * BFS runs on the entire grid (info map or map) to get the frontier points. 
 * 
 * Next, another BFS is run on the collected frontier points to obtain the 
 * 
 * final frontier. 
 * 
*/
void My_Map::findFrontier()
{	
	// Initialize some general vairables. 
	pair<int, int> pose, f_cell, frontier, neighbor, median;
	vector<double> rows, cols;
	int centroid_row = 0, centroid_col = 0;
	
	// Line 1. Initialize the map queue.
	My_Map::map_queue = {};
	My_Map::Map_Open.clear();
	My_Map::Map_Close.clear();
	My_Map::Frontier_Open.clear();
	My_Map::Frontier_Close.clear();
	My_Map::frontier.clear();

	// Line 2 - 3. Push the current point into the queue. Also mark it as Map_Open. 
	pose.first = My_Map::currentPoint.y_pixel_img;
	pose.second = My_Map::currentPoint.x_pixel_img;
	My_Map::map_queue.push(pose);
	My_Map::Map_Open.insert(pose);

	// Line 4. Perform the first BFS. (the outer one)
	while(!My_Map::map_queue.empty())
	{
		// Line 5. 
		f_cell = My_Map::map_queue.front();
		My_Map::map_queue.pop();

		// Line 6. 
		if (My_Map::Map_Close.count(f_cell) == 1)
		{
			continue;
		}

		// Line 8. Check if this cell is a frontier cell. 
		if (My_Map::isFrontierCell(f_cell))
		{
			// Line 9 - 12. 
			My_Map::frontier_queue = {};
			My_Map::new_frontier.clear();
			My_Map::frontier_queue.push(f_cell);
			My_Map::Frontier_Open.insert(f_cell);

			// Line 13. Perform the second BFS. (the inner one)
			while (!My_Map::frontier_queue.empty())
			{
				// Line 14. 
				frontier = My_Map::frontier_queue.front();
				My_Map::frontier_queue.pop();

				// Line 15. 
				if (My_Map::Map_Close.count(frontier) == 1 && 
				My_Map::Frontier_Close.count(frontier) == 1)
				{
					continue;
				}

				// Line 17. 
				if (My_Map::isFrontierCell(frontier))
				{
					// Line 18. 
					My_Map::new_frontier.push_back(frontier);

					// Check all the neighbors of this cell. 
					for (int i = 0; i < connectivity; i++)
					{
						neighbor.first = frontier.first + dy[i];
						neighbor.second = frontier.second + dx[i];

						// Line 20. Only the cells are not a part of Frontier_Open, 
						// Frontier_Close, or Map_Close can be added to the frontier queue. 
						if (My_Map::Frontier_Open.count(neighbor) == 0 && 
						My_Map::Frontier_Close.count(neighbor) == 0 && 
						My_Map::Map_Close.count(neighbor) == 0)
						{
							My_Map::frontier_queue.push(neighbor);
							My_Map::Frontier_Open.insert(neighbor);
						}
					}
				}

				// Line 23. 
				My_Map::Frontier_Close.insert(frontier);
			}

			// Line 24. Save data of new frontier. 
			// This line could be a problem. 
			// Not save the whole frontier. Instead, store the centroid of the frontier. 
			median = My_Map::getCentroid();
			My_Map::frontier.push_back(median);

			// line 25. 
			for (int k = 0; k < static_cast<int>(My_Map::new_frontier.size()); k++)
			{
				My_Map::Map_Close.insert(My_Map::new_frontier[k]);
			}
		}

		// Line 26. Check all the neighbors of this cell. 
		for (int j = 0; j < connectivity; j++)
		{
			neighbor.first = f_cell.first + dy[j];
			neighbor.second = f_cell.second + dx[j];

			// Line 27. at least has one open-space neighbor. 
			if (My_Map::Map_Open.count(neighbor) == 0 && 
			My_Map::Map_Close.count(neighbor) == 0)
			{
				if (My_Map::hasLeastOneOpenSpaceNeighbor(neighbor))
				// if (My_Map::isFrontierCell(neighbor))
				{
					My_Map::map_queue.push(neighbor);
					My_Map::Map_Open.insert(neighbor);
				}
			}
		}

		// Line 30. 
		My_Map::Map_Close.insert(f_cell);
	}

	// // Calculate the centroid of the found frontier. 
	// for (int i = 0; i < My_Map::frontier.size(); i++)
	// {
	// 	rows.push_back(double(My_Map::frontier[i].first));
	// 	cols.push_back(double(My_Map::frontier[i].second));
	// }

	// centroid_row = int(get_median(rows));
	// centroid_col = int(get_median(cols));
	// My_Map::frontierCentroid.first = centroid_row;
	// My_Map::frontierCentroid.second = centroid_col;
}


/**
 * @brief Calculate the centroid (median) of the found frontier. 
 * @return the median of the found frontier. 
 */
pair<int, int> My_Map::getCentroid()
{
	pair<int, int> median;
	vector<double> rows, cols;

	if (!My_Map::new_frontier.empty())
	{
		for (int i = 0; i < static_cast<int>(My_Map::new_frontier.size()); i++)
		{
			rows.push_back(My_Map::new_frontier[i].first);
			cols.push_back(My_Map::new_frontier[i].second);
		}

		median.first = int(get_median(rows));
		median.second = int(get_median(cols));
	}
	else
	{
		median.first = 0;
		median.second = 0;
	}

	return median;
}


/**
 * @brief Show the frontier on the map.
*/
void My_Map::frontierShow()
{
	vector<double> rows, cols, diss;
	vector<double> data;
	double dis = 0;
	map<double, pair<int, int>> medians;
	int length = 3;
	// int centroid_row = 0, centroid_col = 0;

	// Collect all the medians. 
	printf("\n\nCollecting all the medians\n\n");
	if (!My_Map::frontier.empty())
	{
		for (int i = 0; i < static_cast<int>(My_Map::frontier.size()); i++)
		{
			// rows.push_back(My_Map::frontier[i].first);
			// cols.push_back(My_Map::frontier[i].second);

			// My_Map::tempMap.at<Vec3b>(
			// 	My_Map::frontier[i].first, 
			// 	My_Map::frontier[i].second)[0] = 200;

			// My_Map::tempMap.at<Vec3b>(
			// 	My_Map::frontier[i].first, 
			// 	My_Map::frontier[i].second)[1] = 0;

			// My_Map::tempMap.at<Vec3b>(
			// 	My_Map::frontier[i].first, 
			// 	My_Map::frontier[i].second)[2] = 0;

			// cv::circle(
			// 	My_Map::tempMap,
			// 	cv::Point(My_Map::new_frontier[i].second, My_Map::new_frontier[i].first),
			// 	1,
			// 	cv::Scalar(200, 0, 0),
			// 	-1
			// );
			data.clear();
			data.push_back(My_Map::frontier[i].second);
			data.push_back(My_Map::frontier[i].first);
			data.push_back(My_Map::currentPoint.x_pixel_img);
			data.push_back(My_Map::currentPoint.y_pixel_img);
			dis = getDistance(data);

			diss.push_back(dis);
			medians.insert(pair<double, pair<int, int>>(dis, My_Map::frontier[i]));

			// if (dis > 15)
			// {
			// 	cv::circle(
			// 		My_Map::tempMap,
			// 		cv::Point(
			// 			round(My_Map::frontier[i].second), 
			// 			round(My_Map::frontier[i].first)),
			// 		1,
			// 		cv::Scalar(168, 50, 168),
			// 		-1);	
			// }
		}

		// Mark the centroid of the frontier. 
		// centroid_row = get_median(rows);
		// centroid_col = get_median(cols);

		// My_Map::tempMap.at<Vec3b>(
		// 		centroid_row, 
		// 		centroid_col)[0] = 150;

		// My_Map::tempMap.at<Vec3b>(
		// 		centroid_row, 
		// 		centroid_col)[1] = 0;

		// My_Map::tempMap.at<Vec3b>(
		// 		centroid_row, 
		// 		centroid_col)[2] = 0;

		// cv::circle(
		// 	My_Map::tempMap,
		// 	cv::Point(centroid_col, centroid_row),
		// 	1,
		// 	cv::Scalar(168, 50, 168),
		// 	-1
		// );		

		// cv::circle(
		// 	My_Map::tempMap,
		// 	cv::Point(
		// 		round(My_Map::frontierCentroid.second), 
		// 		round(My_Map::frontierCentroid.first)),
		// 	1,
		// 	cv::Scalar(168, 50, 168),
		// 	-1
		// );		
	}

	// Pick three medians with minimum distance. 
	printf("\n\nSorting. \n\n");
	if (!My_Map::frontier.empty())
	{
		printf("\n\nDetermine the length. \n\n");
		if (static_cast<int>(My_Map::frontier.size()) < length)
		{
			length = 1;
			// length = static_cast<int>(My_Map::frontier.size()) - 1;
		}

		partial_sort(diss.begin(), diss.begin() + length, diss.end());
		vector<double> extracted(diss.begin(), diss.begin() + length);

		// Display the filtered medians.
		for (int i = 0; i < static_cast<int>(diss.size()); i++)
		{
			cv::circle(
				My_Map::tempMap,
				cv::Point(
					round(medians[extracted[i]].second), 
					round(medians[extracted[i]].first)),
				2,
				cv::Scalar(168, 50, 168),
				-1);	
		}
	}
}


/**
 * @brief Constructor of class Score. 
 * @param incloud a filtered pointcloud which covers the ROI. 
*/
Score::Score(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud)
{
	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*incloud, *cloud);
	vector<double> height;

	// Calculate the height statistics. 
	for (auto& p : (*cloud).points)
	{
		height.push_back(p.y);
	}

	// Get the basic statistics of the pointcloud. 
	sort(height.begin(), height.end());
	Score::maxHeight = height.back();
	Score::minHeight = height.front();
	Score::height_mean = get_mean(height);
	Score::height_median = get_median(height);
	Score::height_mode = get_mode(height);

	// Initialize the maximum and minimum score of the whole scanned region. 
	Score::minScore = 999999;
	Score::maxScore = -999999;

	// // Debug
	// fstream f;
	// f.open(DEBUG_FILE, ios::app | ios::out);
	// f << to_string(Score::minX) << ", " \
	// << to_string(Score::maxX) << ", " \
	// << to_string(Score::minZ) << ", " \
	// << to_string(Score::maxZ) << "\n";
	// f.close();
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
 * @brief Set the height threshold to determine the traversability. 
 * @param ht the height threshold. 
*/
void Score::setHeightThreshold(double ht)
{
	Score::height_threshold = ht;
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
	// Use the conventional iteration way.
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

	// // Debug 
	// std::fstream f;
	// f.open(DEBUG_FILE, ios::out | ios::app);
	// f << to_string(Score::cloud->size()) << "\n";
	// f << to_string(X.size()) << "\n";
	// f.close();

	// Set the flag
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

	// Reset attributes. 
	Score::slices.clear();

	// Prepare variables for calculating centroid coordinates. 
	vector<double> X, Y, Z;

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

		// Check slice is empty or not. 
		if (slice.indices.size() == 0)
		{
			continue;
		}
		else
		{
			// Save the slice. 
			slice.score = 0.0;
			cv::Scalar cx = cv::sum(X);
			cv::Scalar cy = cv::sum(Y);
			cv::Scalar cz = cv::sum(Z);
			double len = X.size();
			slice.centroid = cv::Vec3d(double(cx[0] / len), double(cy[0] / len), double(cz[0] / len));
			Score::slices.push_back(slice);

			// Reset. 
			X.clear();
			Y.clear();
			Z.clear();
		}
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
	// std::fstream f;
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
	std::fstream f;
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
 * @brief Get the height of slices (grid or region). 
 * 
 * Especially, the median value is taken as the height of that region. 
 * 
 * @param z the specified range along the z-axis. (in camera frame)
*/
bool Score::get_height(double z)
{
	bool is_Zero = false;
	vector<double> heights;
	double height = 0.0;

	// // Debug. 
	// fstream f;
	// f.open(DEBUG_FILE, ios::app | ios::out);

	if (Score::slices.size() != 0)
	{
		for (int i = 0; i < Score::slices.size(); i++)  // for each slice in this z-range. 
		{
			for (int j = 0; j < Score::slices[i].indices.size(); j++)  // for each point in this slice. 
			{
				// height += (*Score::cloud).points[Score::slices[i].indices[j]].y;
				heights.push_back((*Score::cloud).points[Score::slices[i].indices[j]].y);
			}

			// height = get_mean(heights);
			height = get_median(heights);
			Score::slices[i].score = height;
			heights.clear();

			// Update the minScore and maxScore. 
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
 * @brief Render the pointcloud based on the height. 
*/
void Score::rendering()
{
	for (auto& p : (*Score::cloud).points)
	{
		if (p.y >= -Score::height_threshold && p.y <= Score::height_threshold)
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
 * @brief Constructor of class GridAnalysis. 
 * @param incloud the target pointcloud. 
*/
GridAnalysis::GridAnalysis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud)
{
	// Initialize and assign the pointcloud. 
	GridAnalysis::cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*incloud, *(GridAnalysis::cloud));

	// Do the statistics. 
	vector<double> Z, X;

	for (auto& p : GridAnalysis::cloud->points)
	{
		Z.push_back(p.z);
		X.push_back(p.x);
	}

	sort(Z.begin(), Z.end());
	sort(X.begin(), X.end());
	GridAnalysis::maxX = X.back();
	GridAnalysis::minX = X.front();
	GridAnalysis::maxZ = Z.back();
	GridAnalysis::minZ = Z.front();
}


/**
 * @brief Set the size of a cell. 
 * @param size size of a cellthat divide the pointcloud. 
*/
void GridAnalysis::setCellSize(double size)
{
	GridAnalysis::cellSize = size;
}


/**
 * @brief Set the height threshold to determine the traversability. 
 * @param threshold the height threshold. 
*/
void GridAnalysis::setHeightThreshold(double threshold)
{
	GridAnalysis::heightThreshold = threshold;
}


/**
 * @brief Set the weight of the score calculated based on the first criterion, which is the traversability. 
 * @param weight the weight for the traversability. 
 */
void GridAnalysis::setWeight1(double weight)
{
	GridAnalysis::weight1 = weight;
}


/**
 * @brief Set the weight of the score calculated based on the second criterion, which is the continuity. 
 * @param weight the weight for the continuity. 
 */
void GridAnalysis::setWeight2(double weight)
{
	GridAnalysis::weight2 = weight;
}


/**
 * @brief Render the pointcloud based on the height. 
*/
void GridAnalysis::rendering()
{
	for (auto& p : GridAnalysis::cloud->points)
	{
		if (p.y >= -GridAnalysis::heightThreshold && p.y <= GridAnalysis::heightThreshold)
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
 * @brief Divide the pointcloud into a grid and create an info map to store all the necessary data. 
 * 
 * Especially, the info map is a 2D matrix of a user-defined data type: Grid. 
 * 
 * Each element (a struct Grid) contains following information: 
 * 
 * height: a vector to store the height info of all the points within this cell. 
 * 
 * X: x component of the centroid of that cell. 
 * 
 * Y: y component of the centroid of that cell. 
 * 
 * Z: z component of the centroid of that cell. 
*/
void GridAnalysis::divide()
{
	// Determine the size of the info map. 
	int width = 0, height = 0;
	width = ceil((GridAnalysis::maxX - GridAnalysis::minX) / GridAnalysis::cellSize);
	height = ceil((GridAnalysis::maxZ - GridAnalysis::minZ) / GridAnalysis::cellSize);

	// Resize the info map. 
	for (auto& row : GridAnalysis::grid)
	{
		row.resize(width);
	}
	
	GridAnalysis::grid.resize(height, vector<Cell>(width));

	// Iterate through the whole pointcloud to complete the info map. 
	int row = 0, col = 0;

	for (auto& p : GridAnalysis::cloud->points)
	{
		// Determine which row it is in. 
		if (!(p.z < 0.7))  // filter out the points in the invalid range. 
		{
			if ((int)round((p.z - GridAnalysis::minZ) * 1e3) % (int)round(GridAnalysis::cellSize * 1e3) == 0)  // in mm. 
			{
				row = (p.z - GridAnalysis::minZ) / GridAnalysis::cellSize;
			}
			else
			{
				row = floor((p.z - GridAnalysis::minZ) / GridAnalysis::cellSize);
			}
		}
		else
		{
			continue;
		}

		// Determine which column it is in.
		if ((int)round((p.x - GridAnalysis::minX) * 1e3) % (int)round(GridAnalysis::cellSize * 1e3) == 0)  // in mm. 
		{
			col = (p.x - GridAnalysis::minX) / GridAnalysis::cellSize;
		}
		else
		{
			col = floor((p.x - GridAnalysis::minX) / GridAnalysis::cellSize);
		}

		// Fill in that cell. 
		GridAnalysis::grid[row][col].height.push_back(p.y);
		GridAnalysis::grid[row][col].X += p.x;
		GridAnalysis::grid[row][col].Z += p.z;
		GridAnalysis::grid[row][col].counter++;
	}

	// Calculate the statistics of each cell in the grid. 
	for (int i = 0; i < GridAnalysis::grid.size(); i++)
	{
		for (int j = 0; j < GridAnalysis::grid[0].size(); j++)
		{
			if (GridAnalysis::grid[i][j].counter != 0)
			{
				GridAnalysis::grid[i][j].Y = get_median(GridAnalysis::grid[i][j].height);
				GridAnalysis::grid[i][j].X /= GridAnalysis::grid[i][j].counter;
				GridAnalysis::grid[i][j].Z /= GridAnalysis::grid[i][j].counter;
			}
			else
			{
				continue;
			}
		}
	}

	// // Debug for division. 
	// fstream f;
	// string file = "division_1.csv";
	// file = DEBUG_FOLDER + file;
	// f.open(file, ios::out | ios::app);
	// for (int i = 0; i < GridAnalysis::grid.size(); i++)
	// {
	// 	for (int j = 0; j < GridAnalysis::grid[0].size(); j++)
	// 	{
	// 		f << to_string(i) << ", " << to_string(j) << ", " \
	// 		<< to_string(GridAnalysis::grid[i][j].counter) << ", " \
	// 		<< to_string(GridAnalysis::grid[i][j].X) << ", " \
	// 		<< to_string(GridAnalysis::grid[i][j].Y) << ", " \
	// 		<< to_string(GridAnalysis::grid[i][j].Z) << "\n";
	// 	}
	// }
	// f.close();
}


/**
 * @brief Scale to score. 
 * @param scores final score of input data. Scale from lower bound to upper bound.
 * @param data data needs to be scored.
 * @param mode which criterion is going to be used to score.
 */
void GridAnalysis::scale(vector<double> &scores, vector<double> data, string mode)
{
	int len = static_cast<int>(data.size());
	double min = 0, max = 0, score = 0;
	double upperBound = 5, lowerBound = 1;

	if (mode == "height")
	{
		for (int i = 0; i < len; i++)
		{
			data[i] = pow(data[i], 2);
		}

		min = *min_element(data.begin(), data.end());
		max = *max_element(data.begin(), data.end());

		for (int i = 0; i < len; i++)
		{
			score = ((data[i] - min) / (max - min)) * (lowerBound - upperBound) + upperBound;
			scores.push_back(score);
		}
	}
	else if (mode == "number")
	{
		min = *min_element(data.begin(), data.end());
		max = *max_element(data.begin(), data.end());

		for (int i = 0; i < len; i++)
		{
			score = ((data[i] - min) / (max - min)) * (upperBound - lowerBound) + lowerBound;
			scores.push_back(score);
		}
	}
	else if (mode == "continuity")
	{
		min = *min_element(data.begin(), data.end());
		max = *max_element(data.begin(), data.end());

		for (int i = 0; i < len; i++)
		{
			score = ((data[i] - min) / (max - min)) * (lowerBound - upperBound) + upperBound;
			scores.push_back(score);
		}
	}
	else
	{
		exit(-1);
	}
}


/**
 * @brief Score the cells based on the first criterion. 
 */
vector<double> GridAnalysis::firstCriterion()
{
	// Initialize general variables. 
	int len = static_cast<int>(GridAnalysis::cells.size());
	vector<double> numScore, heightScore, heights, numbers, score;

	// Log all the necessary data.
	for (int i = 0; i < len; i++)
	{
		heights.push_back(GridAnalysis::grid[GridAnalysis::cells[i].first][GridAnalysis::cells[i].second].Y);
		// numbers.push_back(GridAnalysis::grid[GridAnalysis::cells[i].first][GridAnalysis::cells[i].second].counter);
	}

	GridAnalysis::scale(heightScore, heights, "height");
	// GridAnalysis::scale(numScore, numbers, "number");

	for (int k = 0; k < len; k++)
	{
		score.push_back(heightScore[k]);
		// score.push_back(heightScore[k] + numScore[k]);
	}
	
	return score;
}


/**
 * @brief Score the cells based on the second criterion. 
 */
vector<double> GridAnalysis::secondCriterion(int iter)
{
	// Initialize general variables. 
	int len = static_cast<int>(GridAnalysis::cells.size());
	vector<double> diss, score;
	double dis = 0.0;
	int center = 0;
	bool isEven = false;

	if (iter == 0)
	{
		if (len % 2 == 0)
		{
			center = len / 2 - 1;
			isEven = true;
		}
		else
		{
			center = len / 2;
			isEven = false;
		}

		for (int i = 0; i < len; i++)
		{
			if (isEven)
			{
				dis = sqrt(
					pow((grid[cells[i].first][cells[i].second].X - \
					((grid[cells[center].first][cells[center].second].X + grid[cells[center+1].first][cells[center+1].second].X) / 2)), 2) + \
					pow((grid[cells[i].first][cells[i].second].Z - \
					((grid[cells[center].first][cells[center].second].Z + grid[cells[center+1].first][cells[center+1].second].Z) / 2)), 2));
			}
			else
			{
				dis = sqrt(
					pow((grid[cells[i].first][cells[i].second].X - grid[cells[center].first][cells[center].second].X), 2) + \
					pow((grid[cells[i].first][cells[i].second].Z - grid[cells[center].first][cells[center].second].Z), 2));
			}

			diss.push_back(dis);
		}
	}
	else
	{
		for (int j = 0; j < len; j++)
		{
			dis = sqrt(
				pow((grid[cells[j].first][cells[j].second].X - grid[bestCells[iter-1].first][bestCells[iter-1].second].X), 2) + \
				pow((grid[cells[j].first][cells[j].second].Z - grid[bestCells[iter-1].first][bestCells[iter-1].second].Z), 2));
			diss.push_back(dis);
		}
	}

	GridAnalysis::scale(score, diss, "continuity");
	return score;
}


/**
 * @brief Score the cells based on the third criterion. 
 */
vector<double> GridAnalysis::thirdCriterion()
{
	// Initialize general variables. 
	int len = static_cast<int>(GridAnalysis::cells.size());
	vector<double> diss, score;
	double dis = 0.0;

	return score;
}


/**
 * @brief Find the local path on the grid.  
 */
void GridAnalysis::findPath(bool isGlobalInvolved)
{
	vector<int> rows;
	double maxScore = -1e6;
	int maxIndex = 0;
	pair<int, int> index;
	vector<double> score1, score2, score3, gScore;
	int len = 0;
	// double weight1 = .58, weight2 = .42;

	// Find the valid row indices. 
	for (int i = GridAnalysis::grid.size()-1; i >= 0; i--)
	{
		for (int j = 0; j < GridAnalysis::grid[0].size(); j++)
		{
			if (GridAnalysis::grid[i][j].counter != 0)
			{
				rows.push_back(i);
				break;
			}
		}
	}

	// Collect all the non-empty cells.
	for(int k = 0; k < static_cast<int>(rows.size()); k++)
	{
		// Initialize the containers for each iteration. 
		GridAnalysis::cells.clear();
		score1.clear();
		score2.clear();
		gScore.clear();
		maxScore = -1e6;
		maxIndex = 0;

		for (int j = 0; j < static_cast<int>(GridAnalysis::grid[0].size()); j++)
		{
			if (grid[rows[k]][j].counter != 0)  // check if empty first. 
			{
				index.first = rows[k];
				index.second = j;
				GridAnalysis::cells.push_back(index);
			}
		}

		len = static_cast<int>(GridAnalysis::cells.size());

		// Get the scores based on two or three criteria. 
        // printf("\n\nCalculating the scores. \n\n");
		score1 = GridAnalysis::firstCriterion();
		score2 = GridAnalysis::secondCriterion(k);

		if (isGlobalInvolved)
		{
			// score3 = GridAnalysis::thirdCriterion();
			// GridAnalysis::weight3 = .20;
			// GridAnalysis::weight1 -= GridAnalysis::weight3 / 2;
			// GridAnalysis::weight2 -= GridAnalysis::weight3 / 2;

			GridAnalysis::weight3 = .00;
			score3.resize(len, 0.0);
		}
		else
		{
			GridAnalysis::weight3 = .00;
			score3.resize(len, 0.0);
		}

		// Get the general score.
        // printf("\n\nCalculating the general scores. \n\n");
		for (int n = 0; n < len; n++)
		{
			if (isGlobalInvolved)
			{
				gScore.push_back(weight1 * score1[n] + weight2 * score2[n] + weight3 * score3[n]);
			}
			else
			{
				gScore.push_back(weight1 * score1[n] + weight2 * score2[n]);
			}
			
		}

		// Find the index of the highest score.
        // printf("\n\nFinding the best score. \n\n");
		for (int m = 0; m < len; m++)
		{
			if (gScore[m] >= maxScore)
			{
				maxScore = gScore[m];
				maxIndex = m;
			}
		}

		// Add it into the bestcells. 
		// printf("\n\nlen = %d and maxIndex = %d\n\n", len, maxIndex);
		GridAnalysis::bestCells.push_back(GridAnalysis::cells[maxIndex]);
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
	// fstream f;

	for (auto& p : cloud->points)
	{
		// f.open(DEBUG_FILE, ios::app | ios::out);
		p.x = vert->x * (-1);
		p.y = vert->y * (-1);
		p.z = vert->z;

		// Perform PC correction. 
		temp_x = p.x;
		temp_y = p.y;
		temp_z = p.z;
		p.y = cos(theta) * temp_y - sin(theta) * temp_z;
		p.z = sin(theta) * temp_y + cos(theta) * temp_z;
		p.y += 65 * CENTI;  // in meter. 
		p.z += 15 * CENTI;  // in meter. 

		// Debug for PC correction. 
		// f << to_string(temp_x) << ", " \
		// << to_string(temp_y) << ", " \
		// << to_string(temp_z) << ", " \
		// << to_string(p.x) << ", " \
		// << to_string(p.y) << ", " \
		// << to_string(p.z) << "\n";

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
    std::fstream f;
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
 * @brief Find the mean of a set of data saved in a vector. 
 * @param data a vector save the statistics. 
 * @return the mean of the given set of data. 
*/
double get_mean(vector<double> data)
{
	double mean = 0.0;
	double len = static_cast<double>(data.size());
	double sum = reduce(data.begin(), data.end(), 0.0);
	mean = sum / len;
	return mean;
}


/**
 * @brief Find the median of a set of data saved in a vector. 
 * @param data a vector save the statistics. 
 * @return the median of the given set of data. 
*/
double get_median(vector<double> data)
{
	// printf("\n\nGet into the get_median. \n\n");
	double median = 0.0;
	int len = static_cast<int>(data.size());
	sort(data.begin(), data.end());

	if (len == 0)
	{
		median = -9999;
	}
	else
	{
		if (len % 2 == 0)
		{
			median = (data[len / 2] + data[len / 2 + 1]) / 2.0;
		}
		else
		{
			median = data[len / 2 + 1];
		}
	}

	return median;
}


/**
 * @brief Find the mode of a set of data saved in a vector. 
 * @param data a vector save the statistics. 
 * @return the mode of the given set of data. 
*/
double get_mode(vector<double> data)
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


/**
 * @brief Calculate the spent time of a specific function. Also log the spent time to debug. 
 * @param start the start time of the function. 
 * @param end the end time of the function. 
 * @param path the path to logging file. 
 * @param isLast the flag to see if this is the last data in this row. 
*/
MyTime getDuration(clock_t start, clock_t end, string path, bool isLast)
{
	fstream f;
	// string file = "spent_time_for_each_component.csv";
	// file = DEBUG_FOLDER + file;
	f.open(path, ios::app | ios::out);
	MyTime time;

	double duration = double(end - start) / double(CLOCKS_PER_SEC);  // in second. 
	duration *= 1000;  // in millisecond. 
	time.is_ms = true;
	time.time = duration;

	if (isLast)
	{
		f << to_string(time.time) << "\n";
	}
	else
	{
		f << to_string(time.time) << ", ";
	}

	f.close();
	return time;
}


/**
 * @brief Calculate the actual execution time. 
 * 
 * Especially, six tests were run to get the data to fit a quadratic function, 
 * 
 * ax^2 + bx + c = y, to predict the actual execution time. 
 * 
 * @param duration the desired execution time. 
 * @return the actual execution time. 
*/
double getActualDuration(double duration)
{
	double time = .0;
	double a = -4.2501e-4;
	double b = .6857;
	double c = -.835;
	// double a = .002;
	// double b = 1.4247;
	// double c = 1.5241;
	time = a * pow(duration, 2) + b * duration + c;
	return time;
}


TEST::TEST()
{
	a = 0;
	b - 10;
}

void changeTest(TEST &t)
{
	t.a = 56;
	t.b = 100;
}
