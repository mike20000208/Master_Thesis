# include "functions.h"


/**
 * @brief Show the robot view and trajectory at the same time. 
 * 
 * When getting the arguments from the terminal, 
 * 
 *  the first one is width,
 * 
 *  the second one is height,
 * 
 *  the third one is res. 
 * 
 * @param node a node that communicates with the capra robot. 
 * @param width width of the map in meter. 
 * @param height height of the map in meter.
 * @param res resolution of the map. 
*/
int stream_test(std::shared_ptr<Mike> node, int width, int height, int res)
{
    // prepare folders and other paths.  
    int count = 0;  // serial number of color images, trajectories, maps, depth info. 
    string img_folder = node->log_path + "/Images";
    string traj_folder = node->log_path + "/Trajectories";
    string depth_folder = node->log_path + "/Depth";
    string map_folder = node->log_path + "/Map";
    string info_path = node->log_path + "/Info.txt";
    string bag_path = node->log_path + "/record.bag";
    string time_path = node->log_path + "/TimeLog.csv";
    string traj_final_path = node->log_path + "/Trajectory_final.png";
    string map_final_path = node->log_path + "/Map_final.png";
    string record_path = node->log_path + "/record.bag";
    string coordinate_path = node->log_path + "/CoordinateLog.csv";
    string traj_suffix;
    string img_suffix;
    string depth_suffix;
    string map_suffix;
    string img_path;
    string traj_path;
    string depth_path;
    string map_path;
    
    // initialize rs2 objects. 
    rs2::pipeline p;
    rs2::frame color_frame, depth_frame;
    rs2::frameset frames;
    rs2::config cfg;
    int stream_width = 1280;
    int stream_height = 720;
    int frame_rate = 30;
    cfg.enable_stream(RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_RGB8, frame_rate);
    cfg.enable_stream(RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16, frame_rate);
    cfg.enable_record_to_file(bag_path);
    // cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    // cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_XYZ32F);

    // initialize cv objects. 
    const string win1 = "Color Image";
    const string win2 = "Trajectory";
	cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
	cv::moveWindow(win1, 0, 0);
	cv::moveWindow(win2, 300, 150);
    cv::Mat image;
    cv::Mat trajectory;

    // initialize other objects.
    My_Map m(width, height, res);
    std::mutex mut;
    std::ofstream f;

    // initialize some variables. 
    Img ImgLog;

    // start the pipeline. 
    // auto profile = p.start(cfg);
    auto profile = p.start(cfg);
    // rs2::device device = profile.get_device();
    // string device_name = device.get_info(RS2_CAMERA_INFO_NAME);
    // printf("\n\n\nCamera [%s] is connected! \n\n\n", device_name.c_str());

    // if (device_name != DEVICE)
    // {
    //     cerr << "\n\n\nCamera connection failed! \n\n\n";
    //     return 0;
    // }
    // else
    // {
    //     printf("\n\n\nCamera [%s] is connected! \n\n\n", device_name.c_str());
    // }

    if (create_directories(img_folder) && create_directories(traj_folder))
    {
        printf("\n\nDirectories are created. \n\n");
    }
    else
    {
        printf("\n\nDirectory creation is failed. \n\n");
    }

    // start streaming. 
    while(1)
    {
        // get frame. 
        frames = p.wait_for_frames();
        // depth_frame = frames.get_depth_frame();
        color_frame = frames.get_color_frame();

        // get the information of the frame. 
        const int w = color_frame.as<rs2::video_frame>().get_width();
        const int h = color_frame.as<rs2::video_frame>().get_height();

        // create color images. 
        image = cv::Mat(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // save the image. 
        img_suffix = "/img_" + to_string(count) + ".png";
        img_path = img_folder + img_suffix;
        cv::imwrite(img_path, image);

        // image number and timestamp logging.
        ImgLog.number = count;
        ImgLog.timestamp = color_frame.get_timestamp() / 1000;
        count ++;
        f.open(time_path, ios::app | ios::out);
        f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << "\n";
        f.close();

        // draw the trajectory.
        mut.lock();

        m.poseUpdate(
            ImgLog.number, 
            node->odo_data.px, 
            node->odo_data.py);

        // if (ImgLog.number == 0)
        // {
        //     m.startPoint.x_meter = node->odo_data.px;
        //     m.startPoint.y_meter = node->odo_data.py;
        //     m.startPoint.x_pixel_img = m.width_pixel / 2;
        //     m.startPoint.y_pixel_img = m.height_pixel / 2;
        //     m.startPoint.x_pixel_map = 0;
        //     m.startPoint.y_pixel_map = 0;
        //     m.previousPoint = m.startPoint;
        //     m.currentPoint = m.startPoint;
        //     // m.map.at<cv::Vec3b>(m.startPoint.y_pixel_img, m.startPoint.x_pixel_img).val[0] = 255;  // b
        //     // m.map.at<cv::Vec3b>(m.startPoint.y_pixel_img, m.startPoint.x_pixel_img).val[1] = 255;  // g
        //     // m.map.at<cv::Vec3b>(m.startPoint.y_pixel_img, m.startPoint.x_pixel_img).val[2] = 255;  // r
        //     cv::circle(
        //         m.map,
        //         cv::Point(m.startPoint.x_pixel_img, m.startPoint.y_pixel_img),
        //         2,
        //         cv::Scalar(255, 255, 255),
        //         FILLED);
        //     cv::putText(
        //         m.map, 
        //         "Start Point", 
        //         cv::Point(m.startPoint.x_pixel_img, 
        //         m.startPoint.y_pixel_img), 
        //         cv::FONT_HERSHEY_PLAIN, 
        //         0.5, 
        //         cv::Scalar(255, 255, 255), 
        //         1);
        // }
        // else
        // {
        //     m.previousPoint = m.currentPoint;
        //     m.currentPoint = m.map2img(node->odo_data.px, node->odo_data.py);
        //     cv::line(
        //         m.map, 
        //         cv::Point(m.currentPoint.x_pixel_img, m.currentPoint.y_pixel_img), 
        //         cv::Point(m.previousPoint.x_pixel_img, m.previousPoint.y_pixel_img), 
        //         cv::Scalar(0, 255, 0), 
        //         2);
        // }

        mut.unlock();

        // coordinate logging.
        f.open(coordinate_path, ios::app | ios::out);
        f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << ", " \
        << to_string(m.currentPoint.x_meter) << ", " << to_string(m.currentPoint.y_meter) << ", " \
        << to_string(m.currentPoint.x_pixel_img) << ", " << to_string(m.currentPoint.y_pixel_img) << ", " \
        << to_string(m.currentPoint.x_pixel_map) << ", " << to_string(m.currentPoint.y_pixel_map) << "\n";
        f.close();

        // save the trajectory as an image.
        traj_suffix = "/trajectory_" + to_string(ImgLog.number) + ".png";
        traj_path = traj_folder + traj_suffix;
        cv::imwrite(traj_path, m.map_);

        // show the current scene. 
        // mut.lock();
        // string time = to_string(node->gps_data.timestamp);
        // mut.unlock();
        // cv::putText(image, time, cv::Point(80, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        cv::resizeWindow(win1, w, h);
        cv::imshow(win1, image);
        cv::cvtColor(m.map_, trajectory, cv::COLOR_RGB2BGR);
        cv::resizeWindow(win2, m.width_pixel, m.height_pixel);
        cv::imshow(win2, trajectory);
        char c = (char)cv::waitKey(10);

        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            cv::imwrite(traj_final_path, m.map_);
            break;
        }
    }

    // document the general info.
    f.open(info_path, ios::app | ios::out);
    f << "Size of the map (width x height) [meter]: " << to_string(m.width_meter) << " x " << to_string(m.height_meter) << "\n\n";
    f << "Resolution of the map [pixel / meter]: " << to_string(m.res) << "\n\n";
    f << "Size of the map (width x height) [pixel]: " << to_string(m.width_pixel) << " x " << to_string(m.height_pixel) << "\n\n";
    f << "Size of color image (width x height): " << to_string(image.cols) << " x " << to_string(image.rows) << "\n\n";
    f << "Size of depth image (width x height): " << to_string(stream_width) << " x " << to_string(stream_height) << "\n\n";
    f.close();
    
    return 0;
}


/**
 * @brief Test the directory creation. 
*/
int time_files_test()
{
    string time = getTime();
    printf("\n\nThe current time is: %s. \n\n", time.c_str());
    string main_path = "/home/mike/RobotLog/";
    string file_name = "GPSLog-" + time + ".csv";
    string file_path = main_path + time;
    if (create_directories(file_path))
    {
        printf("\n\nDirectory [%s] is created. \n\n", file_path.c_str());
    }
    else
    {
        printf("\n\nDirectory creation is failed. \n\n");
    }
    return 0;
}


/**
 * @brief Replay the recorded images and trajectory. 
 * 
 * Can take the first argument from the terminal as folder_name. 
 * 
 * @param folder_name the name of the recording folder. 
 */
int replay(string folder_name)
{
    // initialize counter. 
    int num_files = 0;
    int count = 0;

    // initialize directories.
    string folder_path = REPLAY_FOLDER + folder_name;
    string img_folder = folder_path + "/Images/";
    string traj_folder = folder_path + "/Trajectories/";
    string img_suffix, traj_suffix, full_img_path, full_traj_path;

    // initialize cv objects.
    cv::Mat scene;
    cv::Mat trajectory;
    string win1 = "Scene";
    string win2 = "Trajectory";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    int img_width, traj_width, img_height, traj_height;

    // count the number of files. 
    std::filesystem::path P {img_folder};

    for (auto& p : std::filesystem::directory_iterator(P))
    {
        num_files++;
    }

    // main loop. 
    for (count = 0; count < num_files; count++)
    {
        img_suffix = "img_" + to_string(count) + ".png";
        traj_suffix = "trajectory_" + to_string(count) + ".png";
        full_img_path = img_folder + img_suffix;
        full_traj_path = traj_folder + traj_suffix;
        scene = cv::imread(full_img_path, cv::IMREAD_COLOR);
        trajectory = cv::imread(full_traj_path, cv::IMREAD_COLOR);

        if (scene.empty() || trajectory.empty())
        {
            printf("\n\nReading Error!  \n\n");
            printf("\n\nCannot read images from directories [%s] or [%s]. \n\n", 
            full_img_path.c_str(), 
            full_traj_path.c_str());
            break;
        }

        img_width = scene.cols;
        img_height = scene.rows;
        traj_width = trajectory.cols;
        traj_height = trajectory.rows;
        cv::resizeWindow(win1, img_width, img_height);
        cv::resizeWindow(win2, traj_width, traj_height);
        cv::moveWindow(win1, 0, 0);
	    cv::moveWindow(win2, img_width + 70, 0);
        cv::imshow(win1, scene);
        cv::imshow(win2, trajectory);
        char c = (char)cv::waitKey(100);

        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }
    }

    return 0;
}


/**
 * @brief Create the map with environment information while streaming and robot moving. 
 * 
 * When getting the arguments from the terminal. 
 * 
 *  the first one is width,
 * 
 *  the second one is height,
 * 
 *  the thrid one is res. 
 * 
 * @param node a node that communicates with the capra robot. 
 * @param width width of the map in meter. 
 * @param height height of the map in meter.
 * @param res resolution of the map. 
*/
int stream_map_test(std::shared_ptr<Mike> node, int width, int height, int res)
{
    // prepare folders and other paths.  
    int count = 0;  // serial number of color images, trajectories, maps, depth info. 
    string img_folder = node->log_path + "/Images";
    string traj_folder = node->log_path + "/Trajectories";
    string depth_folder = node->log_path + "/Depth";
    string map_folder = node->log_path + "/Map";
    string info_path = node->log_path + "/Info.txt";
    string bag_path = node->log_path + "/record.bag";
    string time_path = node->log_path + "/TimeLog.csv";
    string traj_final_path = node->log_path + "/Trajectory_final.png";
    string map_final_path = node->log_path + "/Map_final.png";
    string traj_suffix;
    string img_suffix;
    string depth_suffix;
    string map_suffix;
    string img_path;
    string traj_path;
    string depth_path;
    string map_path;
    
    // initialize rs2 objects. 
    rs2::pipeline p;
    rs2::frameset frames;
    rs2::frame color, depth;
    rs2::config cfg;
    rs2::pointcloud pointcloud;
    rs2::points points;
    int stream_width = 1280;
    int stream_height = 720;
    int frame_rate = 30;
    cfg.enable_stream(RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_RGB8, frame_rate);
    cfg.enable_stream(RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16, frame_rate);
    cfg.enable_record_to_file(bag_path);

    // initialize pcl objects.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    viewer->setBackgroundColor(0, 0, 0);
	viewer->setPosition(50, 70);
	viewer->addCoordinateSystem(5, "global");
	viewer->initCameraParameters();

    // initialize cv objects. 
    const string win1 = "Color Image";
    const string win2 = "Map";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    cv::Mat image;
    cv::Mat map;

    // initialize other objects.
    My_Map m(width, height, res);
    std::mutex mut;
    std::ofstream f;

    // initialize other variables.
    Img ImgLog;
    vector<int> inliers;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pc_layers;

    // create the log folders. 
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

    // start the pipeline. 
    p.start(cfg);

    // start streaming. 
    while (1)
    {
        // get frame. 
        frames = p.wait_for_frames();
        color = frames.get_color_frame();
        depth = frames.get_depth_frame();

        // create color image and save it. 
        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();
        image = cv::Mat(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // image and timestamp logging. 
        ImgLog.number = count;
        ImgLog.timestamp = color.get_timestamp() / 1000;
        count ++;
        img_suffix = "/img_" + to_string(ImgLog.number) + ".png";
        img_path = img_folder + img_suffix;
        cv::imwrite(img_path, image);
        f.open(time_path, ios::app | ios::out);
        f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << "\n";
        f.close();

        // calculate realsense pointcloud and convert it into PCL format.
        points = pointcloud.calculate(depth);
        cloud = Points2PCL(points);

        // filter the depth map with z-value. 
		filter.setInputCloud(cloud);
		filter.setFilterFieldName("z");
		filter.setFilterLimits(0, 5);
		filter.filter(*cloud_filtered);

        // // create RANSAC object and compute
        // Eigen::VectorXf* coef = new Eigen::VectorXf;
		// pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
		// 	model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_filtered));
		// pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
		// ransac.setDistanceThreshold(0.10);
		// ransac.setMaxIterations(3000);
		// ransac.computeModel();
		// ransac.getInliers(inliers);
		// ransac.getModelCoefficients(*coef);

        // // show the plane in dark green. 
		// for (int n = 0; n < inliers.size(); n++)
		// {
		// 	cloud_filtered->points[inliers[n]].r = 0;
		// 	cloud_filtered->points[inliers[n]].g = 127;
		// 	cloud_filtered->points[inliers[n]].b = 0;
		// }

        // depth info logging. 
        depth_suffix = "/depth_" + to_string(ImgLog.number) +".csv";
        depth_path = depth_folder + depth_suffix;
        // depth_log(depth_path, cloud_filtered);
        PCL2PLY(cloud_filtered, depth_path);

        // visualization. 
        pc_layers.push_back(cloud_filtered);
        for (int i = 0; i < pc_layers.size(); i++)
        {
			viewer->addPointCloud(
                pc_layers[i], 
                to_string(i));
			viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                4, 
                to_string(i));
        }

        viewer->spinOnce(10);

        cv::resizeWindow(win1, cv::Size(image.cols, image.rows));
        cv::resizeWindow(win2, cv::Size(m.map_.cols, m.map_.rows));
        cv::moveWindow(win1, 0, 0);
        cv::moveWindow(win2, (image.cols + 70), 0);
        cv::imshow(win1, image);
        cv::imshow(win2, m.map_);
        char c = cv::waitKey(10);

        // check whether to terminate the programme. 
        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }

        // reset. 
        inliers.clear();
        pc_layers.clear();
		viewer->removeAllPointClouds();


    }

    // document the general info.
    f.open(info_path, ios::app | ios::out);
    // f << "Map Information" << "\n\n";
    f << "Size of the map (width x height) [meter]: " << to_string(m.width_meter) << " x " << to_string(m.height_meter) << "\n\n";
    f << "Resolution of the map [pixel / meter]: " << to_string(m.res) << "\n\n";
    f << "Size of the map (width x height) [pixel]: " << to_string(m.width_pixel) << " x " << to_string(m.height_pixel) << "\n\n";
    f << "Size of color image (width x height): " << to_string(image.cols) << " x " << to_string(image.rows) << "\n\n";
    f << "Size of depth image (width x height): " << to_string(stream_width) << " x " << to_string(stream_height) << "\n\n";
    f.close();

    return 0;
}


int single_frame_map_test(std::shared_ptr<Mike> node, int width, int height, int res)
{
    // prepare folders and other paths.  
    int count = 0;  // serial number of color images, trajectories, maps, depth info. 
    string img_folder = node->log_path + "/Images";
    string traj_folder = node->log_path + "/Trajectories";
    string depth_folder = node->log_path + "/Depth";
    string map_folder = node->log_path + "/Map";
    string info_path = node->log_path + "/Info.txt";
    // string bag_path = node->log_path + "/record.bag";
    string time_path = node->log_path + "/TimeLog.csv";
    string traj_final_path = node->log_path + "/Trajectory_final.png";
    string map_final_path = node->log_path + "/Map_final.png";
    string traj_suffix;
    string img_suffix;
    string depth_suffix;
    string map_suffix;
    string img_path;
    string traj_path;
    string depth_path;
    string map_path;

    // initialize rs2 objects. 
    rs2::pipeline p;
    rs2::frameset frames;
    rs2::frame color, depth;
    rs2::config cfg;
    rs2::pointcloud pointcloud;
    rs2::points points;
    int stream_width = 1280;
    int stream_height = 720;
    int frame_rate = 30;
    cfg.enable_stream(RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_RGB8, frame_rate);
    cfg.enable_stream(RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16, frame_rate);
    // cfg.enable_record_to_file(bag_path);

    // initialize pcl objects.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    // viewer->setBackgroundColor(0, 0, 0);
	// viewer->setPosition(50, 70);
	// viewer->addCoordinateSystem(5, "global");
	// viewer->initCameraParameters();

    // initialize cv objects. 
    const string win1 = "Color Image";
    const string win2 = "Map";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    cv::Mat image;
    // cv::Mat map;

    // initialize other objects.
    My_Map m(width, height, res);
    // My_Map t(width, height, res);
    std::mutex mut;
    std::ofstream f;

    // initialize other variables.
    Img ImgLog;
    vector<int> inliers;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pc_layers;

    // create the log folders. 
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

    // start the pipeline, and there will be no infinite loop since we only want a single frame. 
    p.start(cfg);

    // get color and depth frame. 
    frames = p.wait_for_frames();
    color = frames.get_color_frame();
    depth = frames.get_depth_frame();

    // create color image and save it. 
    const int w = color.as<rs2::video_frame>().get_width();
    const int h = color.as<rs2::video_frame>().get_height();
    image = cv::Mat(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    // image and timestamp logging. 
    ImgLog.number = count;
    ImgLog.timestamp = color.get_timestamp() / 1000;
    count ++;
    img_suffix = "/img_" + to_string(ImgLog.number) + ".png";
    img_path = img_folder + img_suffix;
    cv::imwrite(img_path, image);
    f.open(time_path, ios::app | ios::out);
    f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << "\n";
    f.close();

    // calculate realsense pointcloud and convert it into PCL format.
    points = pointcloud.calculate(depth);
    cloud = Points2PCL(points);

    // filter the depth map with z-value. 
    filter.setInputCloud(cloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(0, 3);
    filter.filter(*cloud_filtered);

    // create RANSAC object and compute. 
    Eigen::VectorXf* coef = new Eigen::VectorXf;
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_filtered));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
    ransac.setDistanceThreshold(.10);
	ransac.setMaxIterations(3000);
	ransac.setProbability(.65);  // default value is 0.99. 
	// ransac.setProbability(.80);  // default value is 0.99. 
	ransac.computeModel();
	ransac.getInliers(inliers);
	ransac.getModelCoefficients(*coef);

    // show the plane in dark green. 
	for (int n = 0; n < inliers.size(); n++)
	{
		cloud_filtered->points[inliers[n]].r = 0;
		cloud_filtered->points[inliers[n]].g = 127;
		cloud_filtered->points[inliers[n]].b = 0;
	}

    // calculate the roughness. 
	Roughness R(*coef);
	R.get_Roughness(*cloud_filtered);

    // depth info logging. 
    depth_suffix = "/depth_" + to_string(ImgLog.number) +".ply";
    depth_path = depth_folder + depth_suffix;
    PCL2PLY(cloud_filtered, depth_path);

    // visualization. 
    pc_layers.push_back(cloud_filtered);
    cv::Scalar bg_color(0, 0, 0);
    pcl::visualization::PCLVisualizer::Ptr viewer = Visualization(
        pc_layers, 
        bg_color);

    while (!viewer->wasStopped())
	{
        cv::resizeWindow(win1, cv::Size(image.cols, image.rows));
        cv::resizeWindow(win2, cv::Size(m.map_.cols, m.map_.rows));
        cv::moveWindow(win1, 0, 0);
        cv::moveWindow(win2, (image.cols + 70), 0);
        cv::imshow(win1, image);
        cv::imshow(win2, m.map_);
        int c = cv::waitKey(1000);
		viewer->spinOnce(1000);
		std::this_thread::sleep_for(100ms);

        // check whether to terminate the programme. 
        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }
	}

    // reset. 
    cv::destroyAllWindows();
    inliers.clear();
    pc_layers.clear();
    viewer->removeAllPointClouds();
    p.stop();
    
    return 0;
}


/**
 * @brief Replay the trajectory and scene based on the save images and log file in csv format. (more robust than "replay")
 * @param folder_name the name of the recording folder. 
*/
int log_replay(string folder_name)
{
    // initialize loading path. 
    string folder = REPLAY_FOLDER + folder_name;
    string img_folder = folder + "/Images/";
    string time_path = folder + "/TimeLog.csv";
    string odo_path = folder + "/OdoLog.csv";
    string info_path = folder + "/info.csv";
    string img_suffix;
    string img_path;

    // initialize counter. 
    int imgNum = 0;
    int num_files = getFilesNum(img_folder);
    int i = 0;
    int mark = 0;
    int dataCnt = 0;

    // initialize cv objects. 
    const string win1 = "Color Image";
    const string win2 = "Trajectory";
	cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    cv::Mat scene;
    cv::Mat trajectory;

    // initialize log containers. 
    vector<Odo> odoLog;
    map<string, int> infoLog;
    vector<double> timeLog;

    // initialize object and variables for file reading. 
    fstream f;
    vector<string> row;
    string line, word, temp;
    Odo tempOdo;

    // initialize variables for data matching based on timestamp. 
    double currentTime = 0.0; // sec
    double timeRange = 0.1; // sec

    // read TimeLog.csv. 
    f.open(time_path, ios::in);

    while (getline(f, line))
    {
        row.clear();
        stringstream linestream(line);

        while(getline(linestream, word, ','))
        {
            row.push_back(word);
        }

        timeLog.push_back(stod(row[0]));
    }

    f.close();

    // read OdoLog.csv. 
    f.open(odo_path, ios::in);

    while (getline(f, line))
    {
        row.clear();
        stringstream linestream(line);

        while(getline(linestream, word, ','))
        {
            row.push_back(word);
        }

        tempOdo.timestamp = stod(row[0]);
        tempOdo.px = stod(row[1]);
        tempOdo.py = stod(row[2]);
        tempOdo.pz = stod(row[3]);
        tempOdo.ox = stod(row[4]);
        tempOdo.oy = stod(row[5]);
        tempOdo.oz = stod(row[6]);
        tempOdo.ow = stod(row[7]);

        // odoLog[stod(row[0])] = tempOdo;
        odoLog.push_back(tempOdo);
    }

    f.close();

    // read info.csv. 
    f.open(info_path, ios::in);

    while (getline(f, line))
    {
        row.clear();
        stringstream linestream(line);

        while(getline(linestream, word, ','))
        {
            row.push_back(word);
        }

        infoLog["map_width_meter"] = stoi(row[0]);
        infoLog["map_height_meter"] = stoi(row[1]);
        infoLog["resolution"] = stoi(row[2]);
        infoLog["map_width_pixel"] = stoi(row[3]);
        infoLog["map_width_pixel"] = stoi(row[4]);
        infoLog["image_width"] = stoi(row[5]);
        infoLog["image_height"] = stoi(row[6]);
        infoLog["depth_width"] = stoi(row[7]);
        infoLog["depth_height"] = stoi(row[8]);
    }

    f.close();

    // initialize map drawing.
    // My_Map t(
    //     infoLog["map_width_meter"], 
    //     infoLog["map_height_meter"], 
    //     infoLog["resolution"]);

    printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
    int map_width_meter, map_height_meter, map_res;
    cin >> map_width_meter >> map_height_meter >> map_res; 
    My_Map t(map_width_meter, map_height_meter, map_res);

    // My_Map t(20, 20, 5);

    // My_Map t(200, 200, 5);

    // start replaying. 
    for (; imgNum < num_files; imgNum++)
    {
        // prepare the color image. 
        img_suffix = "img_" + to_string(imgNum) + ".png";
        img_path = img_folder + img_suffix;
        scene = cv::imread(img_path, cv::IMREAD_COLOR);

        // check whether the reading is successful or not. 
        if (scene.empty())
        {
            printf("\n\nReading Error!  \n\n");
            printf("\n\nCannot read images from directories [%s]. \n\n", 
            img_path.c_str());
            break;
        }

        // check the timestamp of the current scene.
        currentTime = timeLog[imgNum];

        // find the corresponding odometry data. 
        i = mark;
        Odo currentOdo;

        for (; i < odoLog.size(); i++)
        {
            if (odoLog[i].timestamp >= (currentTime - timeRange) && 
            odoLog[i].timestamp <= (currentTime + timeRange))
            {
                currentOdo = odoLog[i];
                mark = i;
                break;
            }
        }

        // update the pose of the robot. 
        t.poseUpdate(imgNum, currentOdo.px, currentOdo.py);

        // show the map and scene. 
        cv::resizeWindow(win1, scene.cols, scene.rows);
        cv::resizeWindow(win2, t.map_.cols / 2, t.map_.rows / 2);
        cv::moveWindow(win1, 0, 0);
	    cv::moveWindow(win2, scene.cols + 70, 0);
        cv::imshow(win1, scene);
        cv::imshow(win2, t.map_);
        char c = (char)cv::waitKey(100);

        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }
    }

    return 0;
}
