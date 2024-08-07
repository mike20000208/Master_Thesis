#include "utils.h"

bool TERMINATE = false;

bool isRecording = false;

bool isUseKF = true;

bool isReset = false;


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
	f << to_string(Mike::gps_data.timestamp) << ", "
	  << to_string(Mike::gps_data.latitude) << ", "
	  << to_string(Mike::gps_data.longitude) << ", "
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
	f << to_string(Mike::odo_data.timestamp) << ", "
	  << to_string(Mike::odo_data.serial_number) << ", "
	  << to_string(Mike::odo_data.px) << ", " << to_string(Mike::odo_data.py) << ", "
	  << to_string(Mike::odo_data.pz) << ", " << to_string(Mike::odo_data.ox) << ", "
	  << to_string(Mike::odo_data.oy) << ", " << to_string(Mike::odo_data.oz) << ", "
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
	f << to_string(Mike::vel_data.timestamp) << ", "
	  << to_string(Mike::vel_data.lvx) << ", " << to_string(Mike::vel_data.lvy) << ", "
	  << to_string(Mike::vel_data.lvz) << ", " << to_string(Mike::vel_data.avx) << ", "
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
	heatMap_folder = main_folder + "/heatMaps";

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
			create_directories(map_folder) &&
			create_directories(heatMap_folder))
		{
			printf("\n\nDirectories are created. \n\n");
		}
		else
		{
			printf("\n\nDirectory creation is failed. \n\n");
		}
		break;
	}

	case 14:
	{
		// Image export. Used for thesis writing. 
		if (create_directories(img_folder))
		{
			printf("\n\nDirectories are created. \n\n");
		}
		else
		{
			printf("\n\nDirectory creation is failed. \n\n");
		}
		break;

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
	double q_lower_bound = pow(.001, 2); // square meter.
	double q_upper_bound = pow(.03, 2);	 // square meter
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
		for (auto &row : My_Map::infoMap)
		{
			row.resize(My_Map::width_pixel);
		}

		My_Map::infoMap.resize(My_Map::height_pixel, vector<CellKF>(My_Map::width_pixel));

		// // Resize the score records for A*. 
		// for (auto &row : My_Map::gScore)
		// {
		// 	row.resize(My_Map::width_pixel);
		// }

		// My_Map::My_Map::gScore.resize(My_Map::height_pixel, vector<double>(My_Map::width_pixel));

		// for (auto &row : My_Map::fScore)
		// {
		// 	row.resize(My_Map::width_pixel);
		// }

		// My_Map::My_Map::fScore.resize(My_Map::height_pixel, vector<double>(My_Map::width_pixel));
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
	h.x = cos(e.yaw); // the x component of direction vector;
	h.y = sin(e.yaw); // the y component of direction vector;
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
void My_Map::map2img(Pose &p)
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
	My_Map::center_map[0] = cos(-My_Map::currentPoint.yaw) * center_robot[0] +
							sin(-My_Map::currentPoint.yaw) * center_robot[1] +
							(My_Map::currentPoint.x_meter - My_Map::startPoint.x_meter);

	My_Map::center_map[1] = -sin(-My_Map::currentPoint.yaw) * center_robot[0] +
							cos(-My_Map::currentPoint.yaw) * center_robot[1] +
							(My_Map::currentPoint.y_meter - My_Map::startPoint.y_meter);

	My_Map::center_map[2] = My_Map::center_robot[2];

	// Make the unit consistant. from meter to pixel.
	My_Map::center_map *= My_Map::res;

	// confirm the transformation is done.
	My_Map::isTransformed = true;
}


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
			timeSpan = timestamp - infoMap[center_img.y][center_img.x].timestamp; // in second.
			infoMap[center_img.y][center_img.x].timestamp = timestamp;

			// Measurement.
			My_Map::infoMap[center_img.y][center_img.x].measurement = height;
			sigma = KF::selectSigma(depth);
			// sigma = KF::selectSigma(depth, timeSpan);
			variance = pow(sigma, 2);
			noise = KF::selectProcessNoise(timeSpan);

			if (My_Map::infoMap[center_img.y][center_img.x].iteration == 0) // initialization of KF
			{
				// Initialize the estmated state and its variance with the measurement directly.
				My_Map::infoMap[center_img.y][center_img.x].est_state = height;
				My_Map::infoMap[center_img.y][center_img.x].est_cov = variance;

				// Predict.
				My_Map::infoMap[center_img.y][center_img.x].pre_state = height;
				My_Map::infoMap[center_img.y][center_img.x].pre_cov = variance + noise;
			}
			else // normal iteration.
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

		if (!My_Map::isMap)
		{
			My_Map::map_ = cv::Mat(
				My_Map::height_pixel,
				My_Map::width_pixel,
				CV_8UC3,
				cv::Scalar(150, 150, 150)
			);
		}
		else
		{
			CellKF cell;

			for (int i = 0; i < static_cast<int>(My_Map::infoMap.size()); i++)
			{
				for (int j = 0; j < static_cast<int>(My_Map::infoMap[0].size()); j++)
				{
					My_Map::infoMap[i][j] = cell;
				}
			}
		}
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
				newHeight = My_Map::cellProject(G.grid[i][j].Y, G.grid[i][j].Z, timestamp); // the newer method, but still in test.
				G.grid[i][j].Y = newHeight;
			}
		}
	}

	// // Also update the map for A*.
	// for (int i = 0; i < static_cast<int>(My_Map::infoMap.size()); i++)
	// {
	// 	for (int j = 0; j < static_cast<int>(My_Map::infoMap[0].size()); j++)
	// 	{
	// 		if (My_Map::infoMap[i][j].iteration == -1) // unexplored cell.
	// 		{
	// 			My_Map::AStarMap[i][j] = 0.15; // the cost for the unexplored area. (gray on the map)
	// 		}
	// 		else
	// 		{
	// 			My_Map::AStarMap[i][j] = My_Map::infoMap[0][j].est_state;
	// 		}
	// 	}
	// }
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
 * @brief Reconstruct the path by the end of it.
 */
void My_Map::reconstructPath(pair<int, int> end)
{
	// vector<CellAStar> path = {};
	My_Map::path.clear();
	My_Map::path.push_back(end);
	// cout << "\n\nend (x, y, f, g, parent) = " << "( " << end->x << ", " << \
	// end->y <<", " << end->f << ", " << end->g << ", " << \
	// (end->parent == nullptr) << ") \n\n";

	// cout << "\n\nend (x, y, f, g, parent) = " << "( " << end.x << ", " << \
	// end.y <<", " << end.f << ", " << end.g << ", " << \
	// (end.parent == nullptr) << ") \n\n";

	// while(end->parent)
	// {
	// 	path.push_back(*end);
	// 	end = end->parent;
	// 	cout << "\n\nend (x, y, f, g, parent) = " << "( " << end->x << ", " << \
	// 	end->y <<", " << end->f << ", " << end->g << ", " << \
	// 	(end->parent == nullptr) << ") \n\n";
	// }

	while (My_Map::came_from.find(end) != My_Map::came_from.end())
	{
		end = My_Map::came_from[end];
		My_Map::path.push_back(end);
		// cout << "\n\nend (x, y, f, g, parent) = " << "( " << end.x << ", " << \
		// end.y <<", " << end.f << ", " << end.g << ", " << \
		// (end.parent == nullptr) << ") \n\n";
	}

	// Haven't decided whether to reverse or not. It seem not necessary.

	// return path;
}

/**
 * @brief Calculate the A* distance from the found path.
 *
 * Especially, the function calculates the sum of the g score
 *
 * of all the cells in member My_Map::path.
 *
 */
double My_Map::getAStarDistance()
{
	double dis = .0;
	fstream f;
	f.open((string(DEBUG_FOLDER) + string("AStar_execution.txt")), ios::app | ios::out);
	f << "Got into getAStarDistance() function. \n";
	// printf("\n\nGot into getAStarDistance() function. \n\n");

	if (!My_Map::path.empty())
	{
		for (int i = 0; i < static_cast<int>(My_Map::path.size()); i++)
		// for (int i = 0; i < static_cast<int>(path.size()); i++)
		{
			// dis += My_Map::fScore[My_Map::path[i]];
			// dis += My_Map::gScore[My_Map::path[i]];
			// printf("\n\n(row, col) = (%d, %d)\n\n", My_Map::path[i].first, My_Map::path[i].second);

			if (My_Map::gScore.find(My_Map::path[i]) != My_Map::gScore.end())
			{
				dis += My_Map::gScore[My_Map::path[i]];
			}
			else
			{
				dis += 0;
			}

			// dis += My_Map::gScore[My_Map::path[i].first][My_Map::path[i].second];
		}
	}
	
	f << "Got out of getAStarDistance() function. \n";
	f.close();

	return dis;
}

/**
 * @brief Check the bounday violation.
 * @param cell the cell needs to be evaluated.
 * @return is the examinated cell out of bounds.
 */
bool My_Map::boundaryCheck(CellAStar cell)
{
	bool isOut = false;
	int rows = static_cast<int>(My_Map::infoMap.size());
	int cols = static_cast<int>(My_Map::infoMap[0].size());

	if (cell.x < 0 || cell.x > cols)
	{
		isOut = true;
	}

	if (cell.y < 0 || cell.y > rows)
	{
		isOut = true;
	}

	return isOut;
}

/**
 * @brief Calculate the heuristic cost. Diagonal distance is used.
 * @param cell a cell in the grid.
 * @return heuristic cost.
 */
double My_Map::getHeuristic(CellAStar cell, CellAStar goal)
{
	double h = 0;
	double deltaX = 0, deltaY = 0;
	double D2 = sqrt(2) * pow(My_Map::res, -1); // in meter / pixel
	double D = 1 * pow(My_Map::res, -1);		// in meter / pixel

	deltaX = abs(cell.x - goal.x) * D;  // in meter
	deltaY = abs(cell.y - goal.y) * D;  // in meter

	// // Diagonal distance.
	// if (deltaX > deltaY)
	// {
	// 	h = D * deltaX + (D2 - D) * deltaY;
	// }
	// else
	// {
	// 	h = D * deltaY + (D2 - D) * deltaX;
	// }

	// Manhattan distance. 
	h = deltaX + deltaY;

	return h;
}

/**
 * @brief Perform the A* alrorithm either to filter the foound frontier or find the path.
 *
 * Especially, this is method is performed after the frontier is found.
 *
 */
bool My_Map::AStar(CellAStar start, CellAStar goal)
{
	fstream f;
	f.open((string(DEBUG_FOLDER) + string("AStar_execution.txt")), ios::app | ios::out);
	f << "Got into A* algorithm. \n";
	// printf("\n\nGot into A* algorithm. \n\n");

	// Initialize general variables.
	double gTemp = .0;
	double cost = .0;
	int searchCount = 0;
	CellAStar result(-999, -999);

	// Initialize all the necessary containers.
	priority_queue<CellAStar> qO;
	set<pair<int, int>> Open, Close;
	My_Map::gScore.clear();
	My_Map::fScore.clear();
	// My_Map::path.clear();
	My_Map::came_from.clear();

	// Initialize score record containers. 
	// for (int i = 0; i < static_cast<int>(My_Map::gScore.size()); i++)
	// {
	// 	for (int j = 0; j < static_cast<int>(My_Map::gScore[0].size()); j++)
	for (int i = 0; i < static_cast<int>(My_Map::infoMap.size()); i++)
	{
		for (int j = 0; j < static_cast<int>(My_Map::infoMap[0].size()); j++)
		{
			gScore[pair<int, int>(i, j)] = numeric_limits<double>::infinity();
			fScore[pair<int, int>(i, j)] = numeric_limits<double>::infinity();
			// gScore[i][j] = numeric_limits<double>::infinity();
			// fScore[i][j] = numeric_limits<double>::infinity();
		}
	}

	// Line 2.
	// Initialize the scores of the start cell.
	start.g = 0;
	start.f = getHeuristic(start, goal);
	gScore[pair<int, int>(start.y, start.x)] = 0;
	fScore[pair<int, int>(start.y, start.x)] = getHeuristic(start, goal);
	// gScore[start.y][start.x] = 0;
	// fScore[start.y][start.x] = getHeuristic(start, goal);
	qO.push(start);
	Open.insert(pair<int, int>(start.y, start.x));
	// printf("\n\nlength of queue: %d\n\n", static_cast<int>(qO.size()));
	// printf("\n\nstart (x, y, f) = (%d, %d, %.2f). \n\n", start.x, start.y, start.f);

	// Line 3.
	f << "Start searching.....  \n";
	// printf("\n\nStart searching..... \n\n");
	while (!qO.empty())
	{
		// Line 4 - 7.
		searchCount++;
		// printf("\n\nStill searching.......\n\n");
		CellAStar p = qO.top();
		qO.pop();
		// printf("\n\nlength of queue: %d\n\n", static_cast<int>(qO.size()));
		// printf("\n\np (x, y, f) = (%d, %d, %.2f). \n\n", p.x, p.y, p.f);
		Open.erase(pair<int, int>(p.y, p.x));
		Close.insert(pair<int, int>(p.y, p.x));

		// Line 8 - 10.
		if (p == goal)
		{
			f << "Search " << to_string(searchCount) << " times. \n";
			f << "A* succeed. Reconstructing the path..... \n";
			f.close();
			// printf("\n\nPath is found. Reconstructing the path..... \n\n");
			// path = My_Map::reconstructPath(&p);
			// path = My_Map::reconstructPath(pair<int, int>(p.x, p.y));
			My_Map::reconstructPath(pair<int, int>(p.y, p.x));
			return true;
		}

		// Line 11.
		// printf("\n\nChecking the neighbors..... \n\n");
		for (int i = 0; i < connectivity4; i++)
		{
			CellAStar neighbor(p.x + dx4[i], p.y + dy4[i]);

			// Check if the neighbor is valid.
			if (My_Map::boundaryCheck(neighbor))
			{
				continue;
			}
			else
			{
				// Line 12.
				if (Close.count(pair<int, int>(neighbor.y, neighbor.x)) == 1)
				{
					continue;
				}

				// Line 14.
				if (My_Map::infoMap[neighbor.y][neighbor.x].iteration == -1)
				{
					cost = .35;
				}
				else
				{
					// Out of the threshold. 
					if (My_Map::infoMap[neighbor.y][neighbor.x].est_state > My_Map::height_threshold ||
					My_Map::infoMap[neighbor.y][neighbor.x].est_state < -My_Map::height_threshold)
					{
						cost = My_Map::infoMap[neighbor.y][neighbor.x].est_state * 1e3;
						// cost = numeric_limits<double>::infinity();
					}
					else
					{
						cost = My_Map::infoMap[neighbor.y][neighbor.x].est_state;
					}
				}

				gTemp = gScore[pair<int, int>(p.y, p.x)] + cost;
				// gTemp = gScore[p.y][p.x] + cost;

				// Line 15 - 18.
				if ((gTemp < gScore[pair<int, int>(neighbor.y, neighbor.x)]) ||
					(Open.count(pair<int, int>(neighbor.y, neighbor.x)) == 0))
				// if ((gTemp < gScore[neighbor.y][neighbor.x]) ||
				// 	(Open.count(pair<int, int>(neighbor.y, neighbor.x)) == 0))
				{
					neighbor.g = gTemp;
					neighbor.f = gTemp + getHeuristic(neighbor, goal);

					// Update the score records.
					gScore[pair<int, int>(neighbor.y, neighbor.x)] = neighbor.g;
					fScore[pair<int, int>(neighbor.y, neighbor.x)] = neighbor.f;
					// gScore[neighbor.y][neighbor.x] = neighbor.g;
					// fScore[neighbor.y][neighbor.x] = neighbor.f;
					My_Map::came_from[pair<int, int>(neighbor.y, neighbor.x)] = pair<int, int>(p.y, p.x);

					// Line 19 - 21.
					if (Open.count(pair<int, int>(neighbor.y, neighbor.x)) == 0)
					{
						Open.insert(pair<int, int>(neighbor.y, neighbor.x));
						qO.push(neighbor);
					}
				}
			}
		}
	}

	f << "Search " << to_string(searchCount) << " times. \n";
	f << "A* fails. \n";
	f.close();

	return false;
}

/**
 * @brief Find the path with the filtered frontier based on A* algorithm.
 *
 * Especially, this is method is performed after the frontier is found.
 *
 */
void My_Map::findPath()
{
	fstream f;
	f.open((string(DEBUG_FOLDER) + string("AStar_execution.txt")), ios::app | ios::out);
	f << "Start finding the path based on the selected frontier. \n";
	// Initialize general variables.
	// fstream f;
	// f.open((string(DEBUG_FOLDER) + string("cellsCollection.csv")), ios::out | ios::app);
	CellAStar start(My_Map::currentPoint.x_pixel_img, My_Map::currentPoint.y_pixel_img);
	CellAStar goal(My_Map::frontier.second, My_Map::frontier.first);
	// f << to_string(start.x) << ", " << to_string(start.y) << ", -999, -999" << "\n";
	// f << to_string(goal.x) << ", " << to_string(goal.y) << ", -999, -999" << "\n";

	// Perform A* algorithm to find the path.
	// vector<CellAStar> path = My_Map::AStar(start, goal);
	bool isFound = My_Map::AStar(start, goal);

	// Mark the corresponding cells on the info map.
	// // for (int i = 0; i < static_cast<int>(My_Map::path.size()); i++)
	// for (int i = 0; i < static_cast<int>(My_Map::cellsCollection.size()); i++)
	// {
	// 	f << to_string(My_Map::cellsCollection[i].x) << ", " << to_string(My_Map::cellsCollection[i].y) << ", " << \
	// 	to_string(My_Map::cellsCollection[i].g) << ", " << to_string(My_Map::cellsCollection[i].f) << "\n";

	// 	// My_Map::infoMap[My_Map::path[i].first][My_Map::path[i].second].isPath = true;
	// }

	// Mark the corresponding cells on the info map.
	if (isFound)
	{
		for (int i = 0; i < static_cast<int>(My_Map::path.size()); i++)
		{
			My_Map::infoMap[My_Map::path[i].first][My_Map::path[i].second].isPath = true;
		}
	}

	f << "The found path is marked on the info map. \n";
	f.close();

	// f.close();
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
		0.2);
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
	// f.open(string(DEBUG_FOLDER) + string("KF.csv")c);

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

			if (My_Map::infoMap[i][j].iteration == -1 && My_Map::infoMap[i][j].isPath == false) // this cell hasn't been explored.
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
		-1);
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

	if (My_Map::infoMap[cell.first][cell.second].iteration == -1) // an unknown cell.
	{
		for (int i = 0; i < connectivity8; i++)
		{
			// Check if the adjacent is open-space. (explored and obstacle-free)
			if (My_Map::infoMap[cell.first + dy8[i]][cell.second + dx8[i]].iteration != -1)
			{
				if (My_Map::infoMap[cell.first + dy8[i]][cell.second + dx8[i]].est_state >= -My_Map::height_threshold &&
					My_Map::infoMap[cell.first + dy8[i]][cell.second + dx8[i]].est_state <= My_Map::height_threshold)
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

	for (int i = 0; i < connectivity8; i++)
	{
		if (My_Map::infoMap[cell.first + dy8[i]][cell.second + dx8[i]].iteration != -1)
		{
			if (My_Map::infoMap[cell.first + dy8[i]][cell.second + dx8[i]].est_state >= -My_Map::height_threshold &&
				My_Map::infoMap[cell.first + dy8[i]][cell.second + dx8[i]].est_state <= My_Map::height_threshold)
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
	My_Map::frontiers.clear();

	// Line 2 - 3. Push the current point into the queue. Also mark it as Map_Open.
	pose.first = My_Map::currentPoint.y_pixel_img;
	pose.second = My_Map::currentPoint.x_pixel_img;
	My_Map::map_queue.push(pose);
	My_Map::Map_Open.insert(pose);

	// Line 4. Perform the first BFS. (the outer one)
	while (!My_Map::map_queue.empty())
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
					for (int i = 0; i < connectivity8; i++)
					{
						neighbor.first = frontier.first + dy8[i];
						neighbor.second = frontier.second + dx8[i];

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
			// Not save the whole frontier. Instead, store the
			// centroid of the frontier.
			median = My_Map::getCentroid();

			if ((median.first != 0) && (median.second != 0))
			{
				My_Map::frontiers.push_back(median);
			}

			// line 25.
			for (int k = 0; k < static_cast<int>(My_Map::new_frontier.size()); k++)
			{
				My_Map::Map_Close.insert(My_Map::new_frontier[k]);
			}
		}

		// Line 26. Check all the neighbors of this cell.
		for (int j = 0; j < connectivity8; j++)
		{
			neighbor.first = f_cell.first + dy8[j];
			neighbor.second = f_cell.second + dx8[j];

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
 * @brief Select the frontier with the minimal A* distance.
 */
void My_Map::selectFrontier()
{
	// Initialize general variables.
	double dis = 0.0;
	vector<double> diss, data, extracted, dissAStar;
	map<double, pair<int, int>> medians, mediansAStar;
	int length = 2;

	// First frontiers filtering, based on pixel Euclidean distance.
	if (!My_Map::frontiers.empty())
	{
		// Collect all the medians and the corresponding distance data. 
		fstream f;
		f.open((string(DEBUG_FOLDER) + string("AStar_execution.txt")), ios::app | ios::out);
		f << "\nCalculating the Euclidean distance. \n";
		// printf("\n\nCalculating the Euclidean distance. \n\n");
		for (int i = 0; i < static_cast<int>(My_Map::frontiers.size()); i++)
		{
			if (My_Map::frontiers[i].first == 0 && My_Map::frontiers[i].second == 0)
			{
				continue;
			}
			else
			{
				// Euclidean distance.
				data.clear();
				data.push_back(My_Map::frontiers[i].second);
				data.push_back(My_Map::frontiers[i].first);
				data.push_back(My_Map::currentPoint.x_pixel_img);
				data.push_back(My_Map::currentPoint.y_pixel_img);
				dis = getDistance(data);

				// Save the median and the corresponding distance data.
				diss.push_back(dis);
				medians.insert(pair<double, pair<int, int>>(dis, My_Map::frontiers[i]));
			}
		}

		f << to_string(static_cast<int>(diss.size())) << " frontiers and their Euclidean distances are collected. \n";

		// Pick three medians with minimum distance.
		if (static_cast<int>(My_Map::frontiers.size()) == 2)
		{
			length = 2;
		}

		if (static_cast<int>(My_Map::frontiers.size()) == 1)
		{
			length = 1;
		}

		partial_sort(diss.begin(), diss.begin() + length, diss.end());
		extracted.assign(diss.begin(), diss.begin() + length);
		f << to_string(static_cast<int>(extracted.size())) << " frontier(s) are extracted. \n";

		// Second frontier fitlering, based on A* distance. 
		f << "Calculating the total A* cost. \n";
		f.close();
		// printf("\n\nCalculating the total A* cost. \n\n");
		for(int i = 0; i < static_cast<int>(extracted.size()); i++)
		{
			// printf("\n\ngoal (x, y) = (%d, %d)\n\n", medians[extracted[i]].second, medians[extracted[i]].first);
			CellAStar start(My_Map::currentPoint.x_pixel_img, My_Map::currentPoint.y_pixel_img);
			CellAStar goal(medians[extracted[i]].second, medians[extracted[i]].first);
			// CellAStar result = My_Map::AStar(start, goal);
			bool isFound = My_Map::AStar(start, goal);

			// if (result.x != -999 && result.y != -999) // A* succeeds.
			if (isFound) // A* succeeds.
			{
				dis = My_Map::getAStarDistance();
			}
			else
			{
				dis = 999.99;
			}
			
			dissAStar.push_back(dis);
			mediansAStar.insert(pair<double, pair<int, int>>(dis, medians[extracted[i]]));
			// mediansAStar[dis] = medians[extracted[i]];
		}
	}

	// Get the one with the minimal A* distance as the selected frontier. 
	dis = *min_element(dissAStar.begin(), dissAStar.end());
	My_Map::frontier = mediansAStar[dis];

	// // Collect all the medians.
	// if (!My_Map::frontiers.empty())
	// {
	// 	for (int i = 0; i < static_cast<int>(My_Map::frontiers.size()); i++)
	// 	{
	// 		if (My_Map::frontiers[i].first == 0 && My_Map::frontiers[i].second == 0)
	// 		{
	// 			continue;
	// 		}
	// 		else
	// 		{
	// 			// A* distance.
	// 			printf("\n\nInitilization for A* algorithm. \n\n");
	// 			CellAStar start(My_Map::currentPoint.x_pixel_img, My_Map::currentPoint.y_pixel_img);
	// 			CellAStar goal(My_Map::frontiers[i].second, My_Map::frontiers[i].first);
	// 			CellAStar result = My_Map::AStar(start, goal);
	// 			// vector<CellAStar> path = My_Map::AStar(start, goal)

	// 			// if (!My_Map::path.empty())
	// 			printf("\n\nCalculating the A* distance. \n\n");
	// 			if (result.x != -999 && result.y != -999) // A* succeeds.
	// 			{
	// 				dis = My_Map::getAStarDistance();
	// 				// dis = result.f;
	// 			}
	// 			else
	// 			{
	// 				dis = -999.99;
	// 			}

	// 			// save the median and the corresponding distance data.
	// 			diss.push_back(dis);
	// 			medians.insert(pair<double, pair<int, int>>(dis, My_Map::frontiers[i]));
	// 		}
	// 	}
	// }

	// // Pick the median with the minimum distance.
	// if (!My_Map::frontiers.empty())
	// {
	// 	dis = *min_element(diss.begin(), diss.end());
	// 	My_Map::frontier = medians[dis];
	// }
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
 * @brief Show the frontiers on the map.
 *
 * Especially, only the median of the frontier will be shown.
 *
 */
void My_Map::frontierShow()
{
	// vector<double> rows, cols, diss;
	// vector<double> data;
	// double dis = 0.0;
	// map<double, pair<int, int>> medians;
	// int length = 3;

	// Collect all the medians.
	// printf("\n\nCollecting all the medians\n\n");
	// if (!My_Map::frontiers.empty())
	// {
	// 	for (int i = 0; i < static_cast<int>(My_Map::frontiers.size()); i++)
	// 	{

	// 		if (My_Map::frontiers[i].first == 0 && My_Map::frontiers[i].second == 0)
	// 		{
	// 			continue;
	// 		}
	// 		else
	// 		{
	// 			// // Euclidean distance.
	// 			// printf("\n\nCalculating the Euclidean distance. \n\n");
	// 			// data.clear();
	// 			// data.push_back(My_Map::frontier[i].second);
	// 			// data.push_back(My_Map::frontier[i].first);
	// 			// data.push_back(My_Map::currentPoint.x_pixel_img);
	// 			// data.push_back(My_Map::currentPoint.y_pixel_img);
	// 			// dis = getDistance(data);

	// 			// A* distance.
	// 			printf("\n\nCalculating the A* distance. \n\n");
	// 			CellAStar start(My_Map::currentPoint.x_pixel_img, My_Map::currentPoint.y_pixel_img);
	// 			CellAStar goal(My_Map::frontiers[i].second, My_Map::frontiers[i].first);
	// 			// printf("\n\nstart (x, y) = (%d, %d). \n\n", start.x, start.y);
	// 			// printf("\n\ngoal (x, y) = (%d, %d). \n\n", goal.x, goal.y);
	// 			vector<CellAStar> path = My_Map::AStar(start, goal);

	// 			// printf("\n\nChecking the elements in the found path. \n\n");
	// 			// printf("\n\nlength of the found path: %d\n\n", static_cast<int>(path.size()));
	// 			// for (int i = 0; i < static_cast<int>(path.size()); i++)
	// 			// {
	// 			// 	printf(
	// 			// 		"\n\ncell (x, y, g, f) = (%d, %d, %.2f, %.2f). \n\n",
	// 			// 		path[i].x,
	// 			// 		path[i].y,
	// 			// 		path[i].g,
	// 			// 		path[i].f);
	// 			// }

	// 			if (!path.empty())
	// 			{
	// 				dis = My_Map::getAStarDistance(path);
	// 			}
	// 			else
	// 			{
	// 				dis = -999.99;
	// 			}

	// 			// save the median and the corresponding distance data.
	// 			diss.push_back(dis);
	// 			medians.insert(pair<double, pair<int, int>>(dis, My_Map::frontiers[i]));
	// 		}
	// 	}
	// }

	// // Pick three medians with minimum distance.
	// // printf("\n\nSorting. \n\n");
	// if (!My_Map::frontiers.empty())
	// // if (!diss.empty())
	// {
	// 	printf("\n\nlength of found frontiers: %d\n\n", static_cast<int>(My_Map::frontiers.size()));
	// 	printf("\n\nlength of collected frontiers: %d\n\n", static_cast<int>(diss.size()));
	// 	// printf("\n\nDetermine the length. \n\n");
	// 	if (static_cast<int>(My_Map::frontiers.size()) == 2)
	// 	{
	// 		length = 2;
	// 	}

	// 	if (static_cast<int>(My_Map::frontiers.size()) == 1)
	// 	{
	// 		length = 1;
	// 	}

	// 	partial_sort(diss.begin(), diss.begin() + length, diss.end());
	// 	vector<double> extracted(diss.begin(), diss.begin() + length);
	// 	printf("\n\nlength of filtered frontiers: %d\n\n", static_cast<int>(extracted.size()));

	// 	// Display the filtered medians.
	// 	// printf("\n\nlenght of \n\n");
	// 	for (int i = 0; i < static_cast<int>(extracted.size()); i++)
	// 	{
	// 		printf("\n\n(x, y, dis) = (%d, %d, %.2f)\n\n", medians[extracted[i]].second, medians[extracted[i]].first, extracted[i]);
	// 		cv::circle(
	// 			My_Map::tempMap,
	// 			cv::Point(
	// 				medians[extracted[i]].second,
	// 				medians[extracted[i]].first),
	// 			2,
	// 			cv::Scalar(168, 50, 168),
	// 			-1);
	// 	}
	// }


	// // Display all the frontiers. 
	// for (int i = 0; i < static_cast<int>(My_Map::frontiers.size()); i++)
	// {
	// 	cv::circle(
	// 		My_Map::tempMap,
	// 		cv::Point(
	// 			My_Map::frontiers[i].second,
	// 			My_Map::frontiers[i].first),
	// 		2,
	// 		cv::Scalar(168, 50, 168),
	// 		-1);
	// }

	// Only display the most optimal frontier. 
	cv::circle(
		My_Map::tempMap,
		cv::Point(
			My_Map::frontier.second,
			My_Map::frontier.first),
		2,
		cv::Scalar(168, 50, 168),
		-1);
}


/**
 * @brief Find the neighbor cells depending on the scale parameter. 
 * @param scale the parameter to magnify the map. 
 * @param center the center in the info map coordinate.
 * @return (scale * scale - 1) neighbor cells. (in the heat map coordinate)
 */
vector<pair<int, int>> My_Map::findNeighbor(int scale, pair<int, int> center)
{
	// Initialize general variables. 
	vector<pair<int, int>> neighbors;
	pair<int, int> neighbor;
	int offset = (scale - 1) / 2;
	int row = scale * center.first + offset;
	int col = scale * center.second + offset;

	// Search and collect all the neighbors. 
	for (int i = (row - offset); i <= (row + offset); i++)
	{
		for (int j = (col - offset); j <= (col + offset); j++)
		{
			neighbor.first = i;
			neighbor.second = j;
			neighbors.push_back(neighbor);
		}
	}

	return neighbors;
}


/**
 * @brief Create the heat map for thesis writing, 
 */
void My_Map::heatMapShow()
{
	// Initialize general variables.
	string text;
	cv::Scalar color;
	int newCol = 0, newRow = 0;
	pair<int, int> center;
	int precision = 1;

	// // Initialize OpenCV variables for visualization.
	// cv::namedWindow("heat map", WINDOW_NORMAL);
	// cv::resizeWindow("heat map", 500, 500);
	// cv::moveWindow("heat map", (1280 / 2 + 580), (720 / 2 + 80));
	
	// Initialize the heat map. 
	int scale = 15;  // expected to be odd. 
	int offset = (scale - 1) / 2;
	int rows = height_pixel * scale;
	int cols = width_pixel * scale;
	My_Map::heatMap = cv::Mat(
		rows,
		cols,
		CV_8UC3,
		cv::Scalar(150, 150, 150));

	// Rendering from the info map.
	for (int i = 0; i < static_cast<int>(My_Map::infoMap.size()); i++)
	{
		for (int j = 0; j < static_cast<int>(My_Map::infoMap[0].size()); j++)
		{
			if (My_Map::infoMap[i][j].iteration == -1) // this cell hasn't been explored.
			{
				continue;
			}
			else
			{
				// Assgin the color first. 
				if (My_Map::infoMap[i][j].est_state >= -My_Map::height_threshold &&
					My_Map::infoMap[i][j].est_state <= My_Map::height_threshold)
				{
					color = cv::Scalar(0, 127, 0);
				}
				else
				{
					color = cv::Scalar(0, 0, 127);
				}

				// Check all the neighbor cells. 
				center.first = i;
				center.second = j;
				vector<pair<int, int>> neighbors = My_Map::findNeighbor(scale, center);
				
				for (int k = 0; k < static_cast<int>(neighbors.size()); k++)
				{
					My_Map::heatMap.at<cv::Vec3b>(neighbors[k].first, neighbors[k].second)[0] = color[0];
					My_Map::heatMap.at<cv::Vec3b>(neighbors[k].first, neighbors[k].second)[1] = color[1];
					My_Map::heatMap.at<cv::Vec3b>(neighbors[k].first, neighbors[k].second)[2] = color[2];
				}

				// // Assgin the text then. 
				// if ((i % 2 == 0 && j % 2 == 0) || (i % 2 != 0 && j % 2 != 0))
				// {
				// 	std::ostringstream out; 
				// 	out << std::fixed <<std::setprecision(precision) << My_Map::infoMap[i][j].est_state;
				// 	text = out.str();
				// 	newCol = scale * j;
				// 	newRow = scale * i + 2 * offset;
				// 	cv::putText(
				// 		My_Map::heatMap,
				// 		text,
				// 		cv::Point(newCol, newRow),
				// 		cv::FONT_HERSHEY_COMPLEX_SMALL , 
				// 		.5,
				// 		cv::Scalar(0.0, 0.0, 0.0),
				// 		1);
				// }
			}
		}
	}

	// 
	for (int i = 0; i < static_cast<int>(My_Map::infoMap.size()); i++)
	{
		for (int j = 0; j < static_cast<int>(My_Map::infoMap[0].size()); j++)
		{
			if (My_Map::infoMap[i][j].iteration == -1) // this cell hasn't been explored.
			{
				continue;
			}
			else
			{
				// Assgin the text then. 
				if ((i % 2 == 0 && j % 2 == 0) || (i % 2 != 0 && j % 2 != 0))
				{
					std::ostringstream out; 
					out << std::fixed <<std::setprecision(precision) << My_Map::infoMap[i][j].est_state;
					text = out.str();
					newCol = scale * j;
					newRow = scale * i + 2 * offset;
					cv::putText(
						My_Map::heatMap,
						text,
						cv::Point(newCol, newRow),
						cv::FONT_HERSHEY_COMPLEX_SMALL , 
						.6,
						cv::Scalar(0.0, 0.0, 0.0),
						1);
				}
			}
		}
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

	for (auto &p : GridAnalysis::cloud->points)
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
	for (auto &p : GridAnalysis::cloud->points)
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
	for (auto &row : GridAnalysis::grid)
	{
		row.resize(width);
	}

	GridAnalysis::grid.resize(height, vector<Cell>(width));

	// Iterate through the whole pointcloud to complete the info map.
	int row = 0, col = 0;

	for (auto &p : GridAnalysis::cloud->points)
	{
		// Determine which row it is in.
		if (!(p.z < 0.7)) // filter out the points in the invalid range.
		{
			if ((int)round((p.z - GridAnalysis::minZ) * 1e3) % (int)round(GridAnalysis::cellSize * 1e3) == 0) // in mm.
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
		if ((int)round((p.x - GridAnalysis::minX) * 1e3) % (int)round(GridAnalysis::cellSize * 1e3) == 0) // in mm.
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
					pow((grid[cells[i].first][cells[i].second].X -
						 ((grid[cells[center].first][cells[center].second].X + grid[cells[center + 1].first][cells[center + 1].second].X) / 2)),
						2) +
					pow((grid[cells[i].first][cells[i].second].Z -
						 ((grid[cells[center].first][cells[center].second].Z + grid[cells[center + 1].first][cells[center + 1].second].Z) / 2)),
						2));
			}
			else
			{
				dis = sqrt(
					pow((grid[cells[i].first][cells[i].second].X - grid[cells[center].first][cells[center].second].X), 2) +
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
				pow((grid[cells[j].first][cells[j].second].X - grid[bestCells[iter - 1].first][bestCells[iter - 1].second].X), 2) +
				pow((grid[cells[j].first][cells[j].second].Z - grid[bestCells[iter - 1].first][bestCells[iter - 1].second].Z), 2));
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
	for (int i = GridAnalysis::grid.size() - 1; i >= 0; i--)
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
	for (int k = 0; k < static_cast<int>(rows.size()); k++)
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
			if (grid[rows[k]][j].counter != 0) // check if empty first.
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points2PCL(const rs2::points &points)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto vert = points.get_vertices();
	double theta = deg2rad(14.42);					 // inclination of camera.
	double temp_x = 0.0, temp_y = 0.0, temp_z = 0.0; // temp values for coordinates transformation.

	// // Debug
	// fstream f;

	for (auto &p : cloud->points)
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
		p.y += 65 * CENTI; // in meter.
		p.z += 15 * CENTI; // in meter.

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

	for (auto &p : cloud->points)
	{
		f << to_string(p.x) << ", " << to_string(p.y) << ", "
		  << to_string(p.z) << ", " << to_string(p.r) << ", "
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
	// viewer->addCoordinateSystem(max(cloud->width, cloud->height), "global");
	// viewer->addCoordinateSystem(10000, "global");
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
	std::filesystem::path P{folder};

	for (auto &p : std::filesystem::directory_iterator(P))
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

	for (auto &pair : frequency)
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

	double duration = double(end - start) / double(CLOCKS_PER_SEC); // in second.
	duration *= 1000;												// in millisecond.
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
