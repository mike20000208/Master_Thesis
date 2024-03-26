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

    current.x_pixel_img = -(current.y_pixel_map * 1) + My_Map::width_pixel / 2;
    current.y_pixel_img = -(current.x_pixel_map * 1) + My_Map::height_pixel / 2;
    // current.x_pixel_img = -(current.y_pixel_map * My_Map::res) + My_Map::width_pixel / 2;
    // current.y_pixel_img = -(current.x_pixel_map * My_Map::res) + My_Map::height_pixel / 2;
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
            1.0, 
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
 * @brief Update the map with info from camera. 
*/
void My_Map::mapUpdate()
{
    ;
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
 * @brief Calculate the distance between point and plane.
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
	normalize(temp, rough, 0, 255, cv::NORM_MINMAX, CV_64F);
}


/**
 * @brief Constructor of class Score. 
*/
Score::Score(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud)
{
	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*incloud, *cloud);
	vector<int> X;

	// calculate the boudary for get_slice_1(). 
	for (auto& p : (*cloud).points)
	{
		X.push_back(p.x);
	}
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
	// prepare some variables. 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		slice_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> X;

	// build the condition.. 
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr
		condition(new pcl::ConditionAnd<pcl::PointXYZRGB>);
	(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
	(new pcl::FieldComparison<pcl::PointXYZRGB>("z", 
												pcl::ComparisonOps::GE, 
												z)));
	(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
	(new pcl::FieldComparison<pcl::PointXYZRGB>("z", 
												pcl::ComparisonOps::LT, 
												(z + Score::search_step))));

	// build the filter. 
	pcl::ConditionalRemoval<pcl::PointXYZRGB> filter(true);  // default: extract_removed_indices = false
	filter.setCondition(condition);
	filter.setInputCloud(cloud);
	filter.setKeepOrganized(false);

	// apply the filter. 
	filter.filter(*slice_pc);

	// calculate the boudary for get_slice_2(). 
	for (auto& p : (*slice_pc).points)
	{
		X.push_back(p.x);
	}
	
	cv::minMaxLoc(X, &(Score::minX), &(Score::maxX));
	Score::x_len = Score::maxX - Score::minX;
	Score::num_slices = floor(Score::x_len / Score::stride) - 1;

	// set the flag
	Score::found_boundary = true;
}


/**
 * @brief Get the roi as pointcloud for get_slice_2(). 
 * @param z lower limit of z value.
*/
void Score::get_roi(double z)
{
	Score::roi = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	// build the condition. 
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr
		condition(new pcl::ConditionAnd<pcl::PointXYZRGB>);
	(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
	(new pcl::FieldComparison<pcl::PointXYZRGB>("z",
												pcl::ComparisonOps::GE,
												z)));
	(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
	(new pcl::FieldComparison<pcl::PointXYZRGB>("z",
												pcl::ComparisonOps::LT,
												(z + Score::search_step))));

	// build the filter. 
	pcl::ConditionalRemoval<pcl::PointXYZRGB> filter;
	filter.setCondition(condition);
	filter.setInputCloud(cloud);
	filter.setKeepOrganized(true);

	// apply the filter. 
	filter.filter((*Score::roi));

	// set the flag
	Score::found_roi = true;
}


/**
 * @brief Get the slices along x direction within specified z range. (by built-in function in PCL)
 * @param z lower limit of z value. 
*/
void Score::get_slices_1(double z)
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

	for (int i = 0; i < num_slices; i++)
	{		
		// prepare some variables. 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
			roi(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::IndicesConstPtr inliers;
		Slice slice;

		// build the condition. 
		pcl::ConditionOr<pcl::PointXYZRGB>::Ptr
			condition(new pcl::ConditionOr<pcl::PointXYZRGB>);
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("z", 
													pcl::ComparisonOps::LT, 
													z)));
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("z", 
													pcl::ComparisonOps::GE, 
													(z + Score::search_step))));
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("x",
													pcl::ComparisonOps::LT,
													(Score::minX + i * Score::stride))));
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("x",
													pcl::ComparisonOps::GE,
													(Score::minX + i * Score::stride + Score::size))));

		// build the filter. 
		pcl::ConditionalRemoval<pcl::PointXYZRGB> filter(true);  // default: extract_removed_indices = false
		filter.setCondition(condition);
		filter.setInputCloud(Score::cloud);
		filter.setKeepOrganized(true);

		// apply the filter. 
		filter.filter(*roi);
		inliers = filter.getRemovedIndices();

		if (inliers->size() == 0)  // there is no points in this slice. 
		{
			continue;
		}

		// save the slice. 
		slice.score = 0.0;
		slice.indices = (*inliers);
		
		for (int j = 0; j < (*inliers).size(); j++)
		{
			//slice.indices.push_back((*inliers)[j]);
			X.push_back((*cloud).points[(*inliers)[j]].x);
			Y.push_back((*cloud).points[(*inliers)[j]].y);
			Z.push_back((*cloud).points[(*inliers)[j]].z);
		}

		cv::Scalar cx = cv::sum(X);
		cv::Scalar cy = cv::sum(Y);
		cv::Scalar cz = cv::sum(Z);
		int len = X.size();
		slice.centroid = cv::Vec3d(double(cx[0] / len), double(cy[0] / len), double(cz[0] / len));
		Score::slices.push_back(slice);

		// reset 
		X.clear();
		Y.clear();
		Z.clear();
	}
}


/**
 * @brief Get the slices along x direction within specified z range. This method relys on Score::get_boundary() and Score::get_roi(). 
*/
void Score::get_slices_2()
{
	// check the flag. 
	if (!Score::found_boundary || !Score::found_roi)
	{
		if (!Score::found_boundary)
		{
			cerr << "\n\nNeeds to find the boundary first! \n\n";
			exit(-1);
		}
		else
		{
			cerr << "\n\nNeeds to find the ROI first! \n\n";
			exit(-1);
		}
	}
	else
	{
		Score::found_boundary = false;
		Score::found_roi = false;
	}
	
	// reset attributes. 
	Score::slices.clear();

	// prepare some variables. 
	vector<int> X, Y, Z;

	for (int i = 0; i < num_slices; i++)
	{
		// prepare some variables. 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
			slice_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::IndicesConstPtr inliers;
		Slice slice;

		// build the condition. 
		pcl::ConditionOr<pcl::PointXYZRGB>::Ptr
			condition(new pcl::ConditionOr<pcl::PointXYZRGB>);
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("x",
			pcl::ComparisonOps::LT,
			(Score::minX + i * Score::stride))));
		(*condition).addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr
		(new pcl::FieldComparison<pcl::PointXYZRGB>("x",
			pcl::ComparisonOps::GE,
			(Score::minX + i * Score::stride + Score::size))));

		// build the filter. 
		pcl::ConditionalRemoval<pcl::PointXYZRGB> filter(true);  // default: extract_removed_indices = false
		filter.setCondition(condition);
		filter.setInputCloud(Score::roi);
		filter.setKeepOrganized(true);

		// apply the filter. 
		filter.filter(*slice_pc);
		inliers = filter.getRemovedIndices();

		if (inliers->size() == 0)  // there is no points in this slice. 
		{
			continue;
		}

		// save the slice. 
		slice.score = 0.0;
		slice.indices = (*inliers);

		for (int j = 0; j < (*inliers).size(); j++)
		{
			X.push_back((*cloud).points[(*inliers)[j]].x);
			Y.push_back((*cloud).points[(*inliers)[j]].y);
			Z.push_back((*cloud).points[(*inliers)[j]].z);
		}

		cv::Scalar cx = cv::sum(X);
		cv::Scalar cy = cv::sum(Y);
		cv::Scalar cz = cv::sum(Z);
		int len = X.size();
		slice.centroid = cv::Vec3d(double(cx[0] / len), double(cy[0] / len), double(cz[0] / len));
		Score::slices.push_back(slice);

		// reset 
		X.clear();
		Y.clear();
		Z.clear();
	}
}


/**
 * @brief Get the slice along x direction within specified z range. (by conventional iteration method)
*/
void Score::get_slices_3(double z)
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

		// check slice. 
		if (slice.indices.size() == 0)
		{
			continue;
		}

		// save the slice. 
		slice.score = 0.0;
		cv::Scalar cx = cv::sum(X);
		cv::Scalar cy = cv::sum(Y);
		cv::Scalar cz = cv::sum(Z);
		int len = X.size();
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

	if (Score::slices.size() != 0)
	{
		for (int i = 0; i < Score::slices.size(); i++)
		{
			for (int j = 0; j < Score::slices[i].indices.size(); j++)
			{
				if ((*Score::cloud).points[Score::slices[i].indices[j]].g == 127)  // inliers
				{
					Score::slices[i].score += Score::inlier_weight;
				}
				else if ((*Score::cloud).points[Score::slices[i].indices[j]].g == 0 &&
					(*Score::cloud).points[Score::slices[i].indices[j]].b == 0)  // outliers
				{
					//Score::slices[i].score -= 0.3;
					Score::slices[i].score +=
						((255 - (*Score::cloud).points[Score::slices[i].indices[j]].r) / 255) * Score::outlier_weight;
				}
				else
				{
					continue;
				}
			}

			// normalize the score by the number of points. 
			Score::slices[i].score = (Score::slices[i].score) / (Score::slices[i].indices.size());

			// check the distance between currrnt slice and the best slice from the last search. 
			if (Score::best_paths.size() != 0)
			{
				double distance = get_distance(Score::best_paths[Score::best_paths.size() - 1].centroid, Score::slices[i].centroid);
				double maxD = sqrt(pow(Score::search_step, 2) + pow(Score::x_len, 2));
				double minD = Score::search_step;
				distance = (distance - minD) / (maxD - minD);
				Score::slices[i].score -= distance * Score::dis_weight;
			}

			// if have destination to go, then take it into consideration when calculating the score. 
			if (have_dst)
			{
				double angle = get_angle(Score::slices[i].centroid);
				Score::slices[i].score -= (abs(angle) / 180) * Score::angle_weight;
			}
		}
		return is_Zero;
	}
	else
	{
		return !is_Zero;
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
			if (Score::slices[i].score > best_score)
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
	viewer->setPosition(50, 70);
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


