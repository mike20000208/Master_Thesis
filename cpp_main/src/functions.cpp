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
        // get a frame. 
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

        Quaternion_ q;
        q.w = node->odo_data.ow;
        q.x = node->odo_data.ox;
        q.y = node->odo_data.oy;
        q.z = node->odo_data.oz;
        m.poseUpdate(
            ImgLog.number, 
            node->odo_data.px, 
            node->odo_data.py,
            q);

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
int replay_from_images(string folder_name)
{
    // Initialize flag. 
    bool isPressed = false;

    // Initialize counter. 
    int num_files = 0;
    int count = 0;

    // Initialize log containers. 
    vector<double> timeLog;

    // Initialize general variables and objects.
    fstream f;
    vector<string> row;
    string line, word, temp;

    // initialize directories.
    string folder = REPLAY_FOLDER + folder_name;
    string img_folder = folder + "/Images/";
    string traj_folder = folder + "/Trajectories/";
    string time_path = folder + "/TimeLog.csv";
    string img_suffix, traj_suffix, img_full_path, traj_full_path;

    // initialize cv objects.
    cv::Mat scene;
    cv::Mat trajectory;
    string win1 = "Scene";
    string win2 = "Trajectory";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    int img_width, traj_width, img_height, traj_height;

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

    // count the number of files. 
    std::filesystem::path P {img_folder};

    for (auto& p : std::filesystem::directory_iterator(P))
    {
        num_files++;
    }

    // Main loop. Start replaying
    count = 0;
    // for (count = 0; count < num_files; count++)
    while(count < num_files)
    {
        img_suffix = "img_" + to_string(count) + ".png";
        traj_suffix = "trajectory_" + to_string(count) + ".png";
        img_full_path = img_folder + img_suffix;
        traj_full_path = traj_folder + traj_suffix;
        scene = cv::imread(img_full_path, cv::IMREAD_COLOR);
        trajectory = cv::imread(traj_full_path, cv::IMREAD_COLOR);

        if (scene.empty() || trajectory.empty())
        {
            printf("\n\nReading Error!  \n\n");
            printf("\n\nCannot read images from directories [%s] or [%s]. \n\n", 
            img_full_path.c_str(), 
            traj_full_path.c_str());
            break;
        }

        // Visualization. 
        img_width = scene.cols;
        img_height = scene.rows;
        traj_width = trajectory.cols;
        traj_height = trajectory.rows;
        cv::putText(
            scene, 
            "Replaying.....",
		    cv::Point(50, 50),
		    FONT_HERSHEY_DUPLEX,
		    1.0,
		    cv::Scalar(0, 0, 255),
		    1);
        cv::putText(
            scene, 
            to_string(timeLog[count]),
		    cv::Point(50, 100),
		    FONT_HERSHEY_DUPLEX,
		    1.0,
		    cv::Scalar(0, 0, 255),
		    1);
        cv::resizeWindow(win1, (int)img_width / 2, (int)img_height / 2);
        cv::resizeWindow(win2, traj_width, traj_height);
        cv::moveWindow(win1, 0, 0);
	    cv::moveWindow(win2, img_width + 70, 0);
        cv::imshow(win1, scene);
        cv::imshow(win2, trajectory);
        char c = (char)cv::waitKey(100);

        // if (c == 13 || TERMINATE == true)
        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }

        if (c == 100)
        {
            printf("\n\nKey [D] is pressed, 15 frames forward. \n\n");
            count += 15;
            isPressed = true;
        }

        if (c == 97)
        {
            printf("\n\nKey [A] is pressed, 10 frames backward. \n\n");
            count -= 10;
            isPressed = true;
        }

        if (!isPressed)
        {
            count++;
        }
        
        isPressed = false;
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
    int stream_color_width = 1280;
    int stream_color_height = 720;
    int stream_depth_width = 1280;
    int stream_depth_height = 720;
    // int stream_depth_width = 848;
    // int stream_depth_height = 480;
    int frame_rate = 30;

    if (isEnableFromFile)
    {
        cfg.enable_device_from_file("/home/mike/Documents/20240401_175236.bag");
    }
    else
    {
        cfg.enable_stream(RS2_STREAM_COLOR, stream_color_width, stream_color_height, RS2_FORMAT_RGB8, frame_rate);
        cfg.enable_stream(RS2_STREAM_DEPTH, stream_depth_width, stream_depth_height, RS2_FORMAT_Z16, frame_rate);
    }

    // cfg.enable_record_to_file(bag_path);

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
    const string win3 = "Trajectory";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    cv::namedWindow(win3, WINDOW_NORMAL);
    cv::Mat image;

    // initialize other objects.
    My_Map m(width, height, res, true);
    My_Map t(width, height, res);
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

        // draw the map. 
        mut.lock();

        Quaternion_ q;
        q.w = node->odo_data.ow;
        q.x = node->odo_data.ox;
        q.y = node->odo_data.oy;
        q.z = node->odo_data.oz;
        m.poseUpdate(
            ImgLog.number, 
            node->odo_data.px, 
            node->odo_data.py,
            q);
        t.poseUpdate(
            ImgLog.number, 
            node->odo_data.px, 
            node->odo_data.py,
            q);

        mut.unlock();

        // calculate realsense pointcloud and convert it into PCL format.
        points = pointcloud.calculate(depth);
        cloud = Points2PCL(points);

        // filter the depth map with z-value. 
		filter.setInputCloud(cloud);
		filter.setFilterFieldName("z");
		filter.setFilterLimits(0, 3);
		filter.filter(*cloud_filtered);

        // create RANSAC object and compute. (SampleConsensusModelPlane and RandomSampleConsensus)
        Eigen::VectorXf* coef = new Eigen::VectorXf;
        pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_filtered));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
        ransac.setDistanceThreshold(.01);
        ransac.setMaxIterations(2500);
        ransac.setProbability(.60);  // default value is 0.99. 
        ransac.computeModel();
        ransac.getInliers(inliers);
        ransac.getModelCoefficients(*coef);

        // show the plane in dark green. (SampleConsensusModelPlane)
        for (int n = 0; n < inliers.size(); n++)
        {
            cloud_filtered->points[inliers[n]].r = 0;
            cloud_filtered->points[inliers[n]].g = 127;
            cloud_filtered->points[inliers[n]].b = 0;
        }

        // // use another RANSAC object (SACSegmentation) to segment the plane. 
        // pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
	    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // seg.setModelType(pcl::SACMODEL_PLANE);
        // seg.setMethodType(pcl::SAC_RRANSAC);
        // // seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setDistanceThreshold(0.10);
        // seg.setProbability(.80);
        // seg.setMaxIterations(3000);
        // seg.setInputCloud(cloud_filtered);
        // seg.segment(*inliers, *coef);

        // // Show the plane in dark green. (SACSegmentation)
        // for (int n = 0; n < (*inliers).indices.size(); n++)
        // {
        // 	cloud_filtered->points[(*inliers).indices[n]].r = 0;
        // 	cloud_filtered->points[(*inliers).indices[n]].g = 127;
        // 	cloud_filtered->points[(*inliers).indices[n]].b = 0;
        // }

        // calculate the roughness. 
        Roughness R(*coef);
        R.get_Roughness(*cloud_filtered);

        // show the roughness on the pointcloud in red gradient. 
        for (int i = 0; i < R.outliers.size(); i++)
        {
            (*cloud_filtered).points[R.outliers[i]].r = R.rough[i];
            (*cloud_filtered).points[R.outliers[i]].g = 0;
            (*cloud_filtered).points[R.outliers[i]].b = 0;
        }

        // // calculate the best path. 
        // Score S(cloud_filtered); 
        // S.setSearchRange(3.0);
        // S.setSearchStep(0.70);
        // S.setSize(0.60);
        // S.setStride(0.5 * S.size);
        // S.setInlierWeight(0.70);
        // S.setOutlierWeight(1.80);
        // S.setDisWeight(1.80);
        // S.setAngleWeight(0.1);

        // for (double z = 0.0; z < S.search_range; z += S.search_step)
        // {
        //     S.get_boundary(z);
        //     S.get_slices(z);
        //     S.get_score(z);
        //     m.mapUpdate(S);
        //     S.find_best_path();
        // }

        m.headingShow();
        t.headingShow();

        // // show the best path in the point cloud. 
        // for (int k = 0; k < S.best_paths.size(); k++)
        // {
        //     for (int n = 0; n < S.best_paths[k].indices.size(); n++)
        //     {
        //         (*cloud_filtered).points[S.best_paths[k].indices[n]].r = 0;
        //         (*cloud_filtered).points[S.best_paths[k].indices[n]].g = 255;
        //     }
        // }

        // // depth info logging. 
        // depth_suffix = "/depth_" + to_string(ImgLog.number) +".ply";
        // depth_path = depth_folder + depth_suffix;
        // PCL2PLY(cloud_filtered, depth_path);

        // trajectory logging. 
        traj_suffix = "/trajectory_" + to_string(ImgLog.number) + ".png";
        traj_path = traj_folder + traj_suffix;
        cv::imwrite(traj_path, t.tempMap);

        // map logging. 
        map_suffix = "/map_" + to_string(ImgLog.number) + ".png";
        map_path = map_folder + map_suffix;
        cv::imwrite(map_path, m.tempMap);

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

        cv::resizeWindow(win1, cv::Size(image.cols, image.rows));
        cv::resizeWindow(win2, cv::Size(m.map_.cols, m.map_.rows));
        cv::resizeWindow(win3, cv::Size(t.map_.cols, t.map_.rows));
        cv::moveWindow(win1, 0, 0);
        cv::moveWindow(win2, (image.cols + 70), 0);
        cv::moveWindow(win3, (image.cols + 70), (m.map_.rows + 250));
        cv::imshow(win1, image);
        cv::imshow(win2, m.tempMap);
        cv::imshow(win3, t.tempMap);
        char c = cv::waitKey(10);

        viewer->spinOnce(10);

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
    f << "Size of depth image (width x height): " << to_string(stream_depth_width) << " x " << to_string(stream_depth_height) << "\n\n";
    f.close();

    return 0;
}


int single_frame_map_test(std::shared_ptr<Mike> node, int width, int height, int res)
{
    // Prepare folders and other paths.  
    int count = 0;  // serial number of color images, trajectories, maps, depth info. 
    string img_folder = node->log_path + "/Images";
    string traj_folder = node->log_path + "/Trajectories";
    string depth_folder = node->log_path + "/Depth";
    string map_folder = node->log_path + "/Map";
    string info_path = node->log_path + "/Info.txt";
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

    // Initialize rs2 objects. 
    rs2::pipeline p;
    rs2::frameset frames;
    rs2::frame color, depth;
    rs2::config cfg;
    rs2::pointcloud pointcloud;
    rs2::points points;
    int stream_color_width = 1280;
    int stream_color_height = 720;
    int stream_depth_width = 848;
    int stream_depth_height = 480;
    // int stream_depth_width = 1280;
    // int stream_depth_height = 720;
    int frame_rate = 30;

    if (isEnableFromFile)
    {
        cfg.enable_device_from_file("/home/mike/Documents/20240401_175236.bag");
        // cfg.enable_device_from_file("/home/mike/Recording/Room005.bag");
    }
    else
    {
        cfg.enable_stream(RS2_STREAM_COLOR, stream_color_width, stream_color_height, RS2_FORMAT_RGB8, frame_rate);
        cfg.enable_stream(RS2_STREAM_DEPTH, stream_depth_width, stream_depth_height, RS2_FORMAT_Z16, frame_rate);
    }

    // Initialize pcl objects.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> filter;

    // initialize cv objects. 
    const string win1 = "Color Image";
    const string win2 = "Map";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    cv::Mat image;

    // Initialize other objects.
    My_Map m(width, height, res, true);
    std::mutex mut;
    std::ofstream f;

    // Initialize other variables.
    Img ImgLog;
    vector<int> inliers;
    Eigen::VectorXf* coef = new Eigen::VectorXf;
    // pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
	// pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pc_layers;

    // Create the log folders. 
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

    // Start the pipeline, and there will be no infinite loop since we only want a single frame. 
    p.start(cfg);

    // Get color and depth frame. 
    frames = p.wait_for_frames();
    color = frames.get_color_frame();
    depth = frames.get_depth_frame();

    // Create color image and save it. 
    const int w = color.as<rs2::video_frame>().get_width();
    const int h = color.as<rs2::video_frame>().get_height();
    image = cv::Mat(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    // Image and timestamp logging. 
    ImgLog.number = count;
    ImgLog.timestamp = color.get_timestamp() / 1000;
    count ++;
    img_suffix = "/img_" + to_string(ImgLog.number) + ".png";
    img_path = img_folder + img_suffix;
    cv::imwrite(img_path, image);
    f.open(time_path, ios::app | ios::out);
    f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << "\n";
    f.close();

    // Update the pose of the robot.  
    mut.lock();
    Quaternion_ q;
    q.w = node->odo_data.ow;
    q.x = node->odo_data.ox;
    q.y = node->odo_data.oy;
    q.z = node->odo_data.oz;
    m.poseUpdate(
        ImgLog.number, 
        node->odo_data.px, 
        node->odo_data.py,
        q);
    mut.unlock();

    // Calculate realsense pointcloud and convert it into PCL format.
    points = pointcloud.calculate(depth);
    cloud = Points2PCL(points);

    // // debug
    // f.open(DEBUG_FILE, ios::out | ios::app);
    // f << to_string(cloud->points.size()) << "\n";
    // f.close();

    // Filter the depth map with z-value. 
    filter.setInputCloud(cloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(0, 3);
    filter.filter(*cloud_filtered);

    // // debug
    // f.open(DEBUG_FILE, ios::out | ios::app);
    // f << to_string(cloud_filtered->points.size()) << "\n";
    // f.close();

    // Create RANSAC object and compute. (SampleConsensusModelPlane and RandomSampleConsensus)
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_filtered));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
    ransac.setDistanceThreshold(.01);
	ransac.setMaxIterations(2500);
	ransac.setProbability(.99);  // default value is 0.99. 
    ransac.setNumberOfThreads(2);
	ransac.computeModel();
	ransac.getInliers(inliers);
	ransac.getModelCoefficients(*coef);

    // // debug
    // f.open(DEBUG_FILE, ios::out | ios::app);
    // f << to_string(inliers.size()) << "\n";
    // f.close();

    // // use another RANSAC object (SACSegmentation) to segment the plane. 
    // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // seg.setModelType(pcl::SACMODEL_PLANE);
	// seg.setMethodType(pcl::SAC_RRANSAC);
	// // seg.setMethodType(pcl::SAC_RANSAC);
	// seg.setDistanceThreshold(0.10);
    // seg.setProbability(.80);
	// seg.setMaxIterations(3000);
	// seg.setInputCloud(cloud_filtered);
	// seg.segment(*inliers, *coef);

    // Show the plane in dark green. (SampleConsensusModelPlane)
	for (int n = 0; n < inliers.size(); n++)
	{
		cloud_filtered->points[inliers[n]].r = 0;
		cloud_filtered->points[inliers[n]].g = 127;
		cloud_filtered->points[inliers[n]].b = 0;
	}

    // // Show the plane in dark green. (SACSegmentation)
	// for (int n = 0; n < (*inliers).indices.size(); n++)
	// {
	// 	cloud_filtered->points[(*inliers).indices[n]].r = 0;
	// 	cloud_filtered->points[(*inliers).indices[n]].g = 127;
	// 	cloud_filtered->points[(*inliers).indices[n]].b = 0;
	// }


    // // Calculate the roughness. 
	// Roughness R(*coef);
	// R.get_Roughness(*cloud_filtered);

    // // Show the roughness on the pointcloud in red gradient. 
	// for (int i = 0; i < R.outliers.size(); i++)
	// {
	// 	(*cloud_filtered).points[R.outliers[i]].r = R.rough[i];
	// 	(*cloud_filtered).points[R.outliers[i]].g = 0;
	// 	(*cloud_filtered).points[R.outliers[i]].b = 0;
	// }

    // // Calculate the best path. 
	// Score S(cloud_filtered); 
    // S.setStartZ(0.7);
	// S.setSearchRange(3.0);
	// S.setSearchStep(0.70);
	// S.setSize(0.60);
	// S.setStride(0.5 * S.size);
	// S.setInlierWeight(0.70);
	// S.setOutlierWeight(1.80);
	// S.setDisWeight(1.80);
	// S.setAngleWeight(0.1);

    // for (double z = S.start_z; z < S.search_range; z += S.search_step)
	// {
	// 	S.get_boundary(z);
	// 	S.get_slices(z);
	// 	S.get_score(z, false);

    //     if (m.isMap)
    //     {
    //         m.mapUpdate(S);
    //     }

	// 	S.find_best_path();
	// }

    // Show the heading of the robot, also as an indicator of the robot. 
    m.headingShow();
    // m.originShow();
    m.mapShow();

	// // Show the best path in the point cloud. 
	// for (int k = 0; k < S.best_paths.size(); k++)
	// {
	// 	for (int n = 0; n < S.best_paths[k].indices.size(); n++)
	// 	{
	// 		(*cloud_filtered).points[S.best_paths[k].indices[n]].r = 0;
	// 		(*cloud_filtered).points[S.best_paths[k].indices[n]].g = 255;
	// 	}
	// }

    // Depth info logging. 
    depth_suffix = "/depth_" + to_string(ImgLog.number) +".ply";
    depth_path = depth_folder + depth_suffix;
    PCL2PLY(cloud_filtered, depth_path);

    // Map logging.
    map_suffix = "/map_" + to_string(ImgLog.number) +".png";
    map_path = map_folder + map_suffix;
    cv::imwrite(map_path, m.tempMap);

    // Visualization. 
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
        cv::imshow(win2, m.tempMap);
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
 * @param width
 * @param height
 * @param mode
 * 
*/
int replay_from_odometry(string folder_name, int width, int height, int res)
{
    // Initialize flag. 
    bool isPressed = false;
    
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
    // printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
    // int map_width_meter, map_height_meter, map_res;
    // cin >> map_width_meter >> map_height_meter >> map_res; 
    // My_Map t(map_width_meter, map_height_meter, map_res);
    My_Map t(width, height, res);

    // start replaying. 
    while (imgNum < num_files)
    // for (; imgNum < num_files; imgNum++)
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

        // Gather the odometry data to update the pose of the robot. 
        Quaternion_ q;
        q.w = currentOdo.ow;
        q.x = currentOdo.ox;
        q.y = currentOdo.oy;
        q.z = currentOdo.oz;
        t.poseUpdate(imgNum, currentOdo.px, currentOdo.py, q);
        t.headingShow();
        t.mapShow();

        // Show the map and scene. 
        cv::putText(
            scene, 
            "Replaying.....",
		    cv::Point(50, 50),
		    FONT_HERSHEY_DUPLEX,
		    1.0,
		    cv::Scalar(0, 0, 255),
		    1);
        cv::putText(
            scene, 
            to_string(timeLog[imgNum]),
		    cv::Point(50, 100),
		    FONT_HERSHEY_DUPLEX,
		    1.0,
		    cv::Scalar(0, 0, 255),
		    1);
        cv::resizeWindow(win1, (int)scene.cols / 2, (int)scene.rows / 2);
        cv::resizeWindow(win2, t.map_.cols, t.map_.rows);
        cv::moveWindow(win1, 0, 0);
	    cv::moveWindow(win2, scene.cols + 70, 0);
        cv::imshow(win1, scene);
        cv::imshow(win2, t.tempMap);
        char c = (char)cv::waitKey(100);

        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }

        // reset the temporary map which is only used to display. 
        t.tempMap = Mat();

        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }

        if (c == 100)
        {
            printf("\n\nKey [D] is pressed, 15 frames forward. \n\n");
            imgNum += 15;
            isPressed = true;
        }

        if (c == 97)
        {
            printf("\n\nKey [A] is pressed, 10 frames backward. \n\n");
            imgNum -= 10;
            isPressed = true;
        }

        if (!isPressed)
        {
            imgNum++;
        }

        isPressed = false;
    }

    return 0;
}


/**
 * @brief Debug the pointcloud details and the map projection in the early stage.  
*/
int pointcloud_debug(int width, int height, int res)
{
    // Initialize rs2 objects. 
    rs2::pipeline p;
    rs2::frameset frames;
    rs2::frame color, depth;
    rs2::config cfg;
    rs2::pointcloud pointcloud;
    rs2::points points;
    int stream_width = 1280;
    int stream_height = 720;
    int frame_rate = 30;
    cfg.enable_device_from_file("/home/mike/Recording/Room005.bag");
    // cfg.enable_stream(RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_RGB8, frame_rate);
    // cfg.enable_stream(RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16, frame_rate);

    // Initialize pcl objects.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> filter;

    // Initialize other variables and objects.
    ofstream f;
    f.open(DEBUG_FILE, ios::out);
    vector<int> inliers;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pc_layers;
    My_Map m(width, height, res, true);
    cv::namedWindow("Map", WINDOW_NORMAL);

    // Start the pipeline, and there will be no infinite loop since we only want a single frame. 
    p.start(cfg);

    // Get color and depth frame. 
    frames = p.wait_for_frames();
    color = frames.get_color_frame();
    depth = frames.get_depth_frame();

    // Calculate realsense pointcloud and convert it into PCL format.
    points = pointcloud.calculate(depth);
    cloud = Points2PCL(points);

    // Filter the depth map with z-value. 
    filter.setInputCloud(cloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(0, 5);
    filter.filter(*cloud_filtered);
    f << to_string(cloud_filtered->points.size()) << "\n";

    // Update the pose of the robot.  
    Quaternion_ q;
    q.w = -0.000000;
    q.x = 0.015092;
    q.y = -0.000000;
    q.z = -0.999886;
    m.poseUpdate(
        0, 
        0.000000, 
        0.000000,
        q);

    // Show the ranges in different colors.
    vector<cv::Vec3i> colors;
    colors.push_back(cv::Vec3i(143, 9, 9));  // red
    colors.push_back(cv::Vec3i(214, 91, 19));  // orange
    colors.push_back(cv::Vec3i(211, 214, 19));  // yellow
    colors.push_back(cv::Vec3i(19, 214, 55));  // green
    colors.push_back(cv::Vec3i(19, 104, 214));  // blue
    colors.push_back(cv::Vec3i(152, 19, 214));  // purple
    colors.push_back(cv::Vec3i(235, 59, 123));  // pink
    double step = 0.7;
    double range = 3.8;
    int c = 0;
    int count = 0;

    // Calculate the best path. 
	Score S(cloud_filtered); 
	S.setSearchRange(3.0);
	S.setSearchStep(0.50);
	S.setSize(0.60);
	S.setStride(0.5 * S.size);
	S.setInlierWeight(0.70);
	S.setOutlierWeight(1.80);
	S.setDisWeight(1.80);
	S.setAngleWeight(0.1);

    for (double z = 0.0; z < range; z += step)
    {
        for (int i = 0; i < cloud_filtered->points.size(); i++)
        {
            if ((cloud_filtered->points[i].z >= z) && 
            (cloud_filtered->points[i].z < (z + step)))
            {
                cloud_filtered->points[i].r = colors[c][0];
                cloud_filtered->points[i].g = colors[c][1];
                cloud_filtered->points[i].b = colors[c][2];
                count++;
            }
        }

        // debug
        f << to_string(z) << ", " << to_string(c) << ", " << to_string(count) << "\n";

		S.get_boundary(z);
		S.get_slices(z);
		S.get_score(z);

        if (m.isMap)
        {
            m.mapUpdate(S, colors, c);
        }

        c++;
        count = 0;
    }

    m.tempMap = m.map_.clone();

    f.close();

    PCL2PLY(cloud_filtered, "/home/mike/Debug/pointcloud.ply");

    // Visualization. 
    pc_layers.push_back(cloud_filtered);
    cv::Scalar bg_color(0, 0, 0);
    pcl::visualization::PCLVisualizer::Ptr viewer = Visualization(
        pc_layers, 
        bg_color);

    while (!viewer->wasStopped())
	{
        cv::resizeWindow("Map", cv::Size(m.map_.cols, m.map_.rows));
        cv::moveWindow("Map", 1200, 0);
        // cv::cvtColor(m.tempMap, m.tempMap, cv::COLOR_RGB2BGR);
        cv::imshow("Map", m.tempMap);
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
    
    return 0;
}


/**
 * @brief Debug the map projection function. 
*/
int map_projection_debug(std::shared_ptr<Mike> node, int width, int height, int res, string mode)
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
    int stream_color_width = 1280;
    int stream_color_height = 720;
    int stream_depth_width = 1280;
    int stream_depth_height = 720;
    // int stream_depth_width = 848;
    // int stream_depth_height = 480;
    int frame_rate = 30;

    if (isEnableFromFile)
    {
        cfg.enable_device_from_file("/home/mike/Documents/20240401_175236.bag");
    }
    else
    {
        cfg.enable_stream(RS2_STREAM_COLOR, stream_color_width, stream_color_height, RS2_FORMAT_RGB8, frame_rate);
        cfg.enable_stream(RS2_STREAM_DEPTH, stream_depth_width, stream_depth_height, RS2_FORMAT_Z16, frame_rate);
    }

    // cfg.enable_record_to_file(bag_path);

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
    // const string win3 = "Trajectory";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    // cv::namedWindow(win3, WINDOW_NORMAL);
    cv::Mat image;

    // initialize other objects.
    My_Map m(width, height, res, true);
    // My_Map t(width, height, res);
    std::mutex mut;
    std::ofstream f;

    // initialize other variables.
    Img ImgLog;
    vector<int> inliers;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pc_layers;

    // Show the ranges in different colors.
    vector<cv::Vec3i> colors;
    colors.push_back(cv::Vec3i(143, 9, 9));  // red
    colors.push_back(cv::Vec3i(214, 91, 19));  // orange
    colors.push_back(cv::Vec3i(211, 214, 19));  // yellow
    colors.push_back(cv::Vec3i(19, 214, 55));  // green
    colors.push_back(cv::Vec3i(19, 104, 214));  // blue
    colors.push_back(cv::Vec3i(152, 19, 214));  // purple
    colors.push_back(cv::Vec3i(235, 59, 123));  // pink

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

        // draw the map. 
        Quaternion_ q;
        q.w = -0.000000;
        q.x = 0.015092;
        q.y = -0.000000;
        q.z = -0.999886;
        m.poseUpdate(
            0, 
            0.000000, 
            0.000000,
            q);

        // calculate realsense pointcloud and convert it into PCL format.
        points = pointcloud.calculate(depth);
        cloud = Points2PCL(points);

        // filter the depth map with z-value. 
		filter.setInputCloud(cloud);
		filter.setFilterFieldName("z");
		filter.setFilterLimits(0, 4);
		filter.filter(*cloud_filtered);

        // create RANSAC object and compute. 
        Eigen::VectorXf* coef = new Eigen::VectorXf;
        pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_filtered));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
        ransac.setDistanceThreshold(.01);
        ransac.setMaxIterations(2500);
        ransac.setProbability(.60);  // default value is 0.99. 
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

        // show the roughness on the pointcloud in red gradient. 
        for (int i = 0; i < R.outliers.size(); i++)
        {
            (*cloud_filtered).points[R.outliers[i]].r = R.rough[i];
            (*cloud_filtered).points[R.outliers[i]].g = 0;
            (*cloud_filtered).points[R.outliers[i]].b = 0;
        }

        // render the pointcloud and debug the map projection. 
        int color = 0;
        Score S(cloud_filtered); 
        S.setStartZ(0.3);
        S.setSearchRange(3.0);
        S.setSearchStep(0.70);
        S.setSize(0.60);
        S.setStride(0.5 * S.size);
        S.setInlierWeight(0.70);
        S.setOutlierWeight(1.80);
        S.setDisWeight(1.80);
        S.setAngleWeight(0.1);

        for (double z = S.start_z; z < S.search_range; z += S.search_step)
        {
            if (mode == "color")
            {
                for (int i = 0; i < cloud_filtered->points.size(); i++)
                {
                    if ((cloud_filtered->points[i].z >= z) && 
                    (cloud_filtered->points[i].z < (z + S.search_step)))
                    {
                        cloud_filtered->points[i].r = colors[color][0];
                        cloud_filtered->points[i].g = colors[color][1];
                        cloud_filtered->points[i].b = colors[color][2];
                    }
                }
                S.get_boundary(z);
                S.get_slices(z);
                if (m.isMap)
                {
                    // m.mapUpdate(S, colors, color);
                    m.mapUpdate(S, colors, color);
                }
                color++;
            }
            else if (mode == "info")
            {
                S.get_boundary(z);
                S.get_slices(z);
                // S.get_score(z);
                S.get_roughness(z);
                if (m.isMap)
                {
                    // m.mapUpdate(S, colors, color);
                    m.mapUpdate(S);
                }
            }
            else 
            {
                cerr << "Wrong mode was used! (" << mode << "). \n\n";
                return -1;
            }

        }

        // m.headingShow();
        m.mapShow();

        // // show the best path in the point cloud. 
        // for (int k = 0; k < S.best_paths.size(); k++)
        // {
        //     for (int n = 0; n < S.best_paths[k].indices.size(); n++)
        //     {
        //         (*cloud_filtered).points[S.best_paths[k].indices[n]].r = 0;
        //         (*cloud_filtered).points[S.best_paths[k].indices[n]].g = 255;
        //     }
        // }

        // // depth info logging. 
        // depth_suffix = "/depth_" + to_string(ImgLog.number) +".ply";
        // depth_path = depth_folder + depth_suffix;
        // PCL2PLY(cloud_filtered, depth_path);

        // // trajectory logging. 
        // traj_suffix = "/trajectory_" + to_string(ImgLog.number) + ".png";
        // traj_path = traj_folder + traj_suffix;
        // cv::imwrite(traj_path, t.tempMap);

        // // map logging. 
        // map_suffix = "/map_" + to_string(ImgLog.number) + ".png";
        // map_path = map_folder + map_suffix;
        // cv::imwrite(map_path, m.tempMap);

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

        cv::resizeWindow(win1, cv::Size(image.cols, image.rows));
        cv::resizeWindow(win2, cv::Size(m.map_.cols, m.map_.rows));
        // cv::resizeWindow(win3, cv::Size(t.map_.cols, t.map_.rows));
        cv::moveWindow(win1, 0, 0);
        cv::moveWindow(win2, (image.cols + 70), 0);
        // cv::moveWindow(win3, (image.cols + 70), (m.map_.rows + 250));
        cv::imshow(win1, image);
        cv::imshow(win2, m.tempMap);
        // cv::imshow(win3, t.tempMap);
        char c = cv::waitKey(10);

        viewer->spinOnce(10);

        // check whether to terminate the programme. 
        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }

        // reset. 
        // inliers.clear();
        pc_layers.clear();
		viewer->removeAllPointClouds();
    }

    // // document the general info.
    // f.open(info_path, ios::app | ios::out);
    // // f << "Map Information" << "\n\n";
    // f << "Size of the map (width x height) [meter]: " << to_string(m.width_meter) << " x " << to_string(m.height_meter) << "\n\n";
    // f << "Resolution of the map [pixel / meter]: " << to_string(m.res) << "\n\n";
    // f << "Size of the map (width x height) [pixel]: " << to_string(m.width_pixel) << " x " << to_string(m.height_pixel) << "\n\n";
    // f << "Size of color image (width x height): " << to_string(image.cols) << " x " << to_string(image.rows) << "\n\n";
    // f << "Size of depth image (width x height): " << to_string(stream_depth_width) << " x " << to_string(stream_depth_height) << "\n\n";
    // f.close();
    
    return 0;
}


/**
 * @brief Test the delay between the receiving node and the main program. 
 * 
 * @param node a ROS node that communicates with the Capra robot. 
*/
int delay_test(std::shared_ptr<Mike> node)
{
    // Initialize general objects and variables.
    My_Map t(10, 10, 5);
    // My_Map t(width, height, res);
    std::mutex mut;
    std::ofstream f;
    Img ImgLog;
    double pc_time = 0.0, node_time = 0.0, camera_time = 0.0;
    
    // Prepare folders and other paths.  
    int count = 0;  // serial number of color images, trajectories, maps, depth info. 
    mut.lock();
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
    // string debug_path = node->log_path + "/delay_test_only_scene.csv";
    string debug_path = node->log_path + "/delay_test_scene_and_trajectory.csv";
    mut.unlock();

    string traj_suffix;
    string img_suffix;
    string depth_suffix;
    string map_suffix;
    string img_path;
    string traj_path;
    string depth_path;
    string map_path;

    // Initialize rs2 objects. 
    rs2::pipeline p;
    rs2::frame color_frame, depth_frame;
    rs2::frameset frames;
    rs2::config cfg;
    int stream_color_width = 1280;
    int stream_color_height = 720;
    int stream_depth_width = 1280;
    int stream_depth_height = 720;
    int frame_rate = 30;
    cfg.enable_stream(RS2_STREAM_COLOR, stream_color_width, stream_color_height, RS2_FORMAT_RGB8, frame_rate);
    cfg.enable_stream(RS2_STREAM_DEPTH, stream_depth_width, stream_depth_height, RS2_FORMAT_Z16, frame_rate);
    // cfg.enable_record_to_file(bag_path);

    // Initialize cv objects. 
    const string win1 = "Color Image";
    const string win2 = "Trajectory";
	cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
	cv::moveWindow(win1, 0, 0);
	cv::moveWindow(win2, 300, 150);
    cv::Mat image;

    // Start the pipeline. 
    auto profile = p.start(cfg);

    // Create the folders for logging. 
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

    // Start streaming. 
    while (1)
    {
        // Get a frame. 
        frames = p.wait_for_frames();
        // depth_frame = frames.get_depth_frame();
        color_frame = frames.get_color_frame();

        // Get the information of the frame. 
        const int w = color_frame.as<rs2::video_frame>().get_width();
        const int h = color_frame.as<rs2::video_frame>().get_height();

        // Create color images. 
        image = cv::Mat(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // Save the image. 
        img_suffix = "/img_" + to_string(count) + ".png";
        img_path = img_folder + img_suffix;
        cv::imwrite(img_path, image);

        // Image number and timestamp logging.
        ImgLog.number = count;
        ImgLog.timestamp = color_frame.get_timestamp() / 1000;
        count ++;
        f.open(time_path, ios::app | ios::out);
        f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << "\n";
        f.close();

        // Test the delay between the main program and the ROS node. 
        const auto currenTime = std::chrono::system_clock::now();
        pc_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(currenTime.time_since_epoch()).count() * MICRO;
        camera_time = ImgLog.timestamp;

        mut.lock();
        node_time = node->odo_data.timestamp;
        mut.unlock();

        // Logging the delay. 
        f.open(debug_path, ios::app | ios::out);
        f << to_string(camera_time) << ", " \
        << to_string(pc_time) << ", " \
        << to_string(node_time) << "\n";
        f.close();

        // Draw the trajectory. 
        mut.lock();
        Quaternion_ q;
        q.w = node->odo_data.ow;
        q.x = node->odo_data.ox;
        q.y = node->odo_data.oy;
        q.z = node->odo_data.oz;
        t.poseUpdate(
            ImgLog.number, 
            node->odo_data.px, 
            node->odo_data.py,
            q);
        mut.unlock();
        t.headingShow();
        t.mapShow();

        // Show the current scene. 
        cv::resizeWindow(win1, image.cols, image.rows);
        cv::resizeWindow(win2, t.width_pixel, t.height_pixel);
        cv::imshow(win1, image);
        cv::imshow(win2, t.tempMap);
        char c = (char)cv::waitKey(10);

        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            cv::imwrite(traj_final_path, t.tempMap);
            break;
        }
    }

    return 0;
}


/**
 * @brief Logging function for the field trip. 
 * @param node
 * @param width
 * @param height
 * @param res
*/
int field_trip(std::shared_ptr<Mike> node, int width, int height, int res)
{
    // Prepare folders and other paths.  
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
    
    // Initialize rs2 objects. 
    rs2::pipeline p;
    rs2::frameset frames;
    rs2::frame color, depth;
    rs2::config cfg;
    rs2::pointcloud pointcloud;
    rs2::points points;
    int stream_color_width = 1280;
    int stream_color_height = 720;
    int stream_depth_width = 1280;
    int stream_depth_height = 720;
    // int stream_depth_width = 848;
    // int stream_depth_height = 480;
    int frame_rate = 30;

    if (isEnableFromFile)
    {
        cfg.enable_device_from_file("/home/mike/Documents/20240401_175236.bag");
    }
    else
    {
        cfg.enable_stream(RS2_STREAM_COLOR, stream_color_width, stream_color_height, RS2_FORMAT_RGB8, frame_rate);
        cfg.enable_stream(RS2_STREAM_DEPTH, stream_depth_width, stream_depth_height, RS2_FORMAT_Z16, frame_rate);
    }

    if (isRecording)
    {
        cfg.enable_record_to_file(bag_path);
    }
    
    // Initialize cv objects. 
    const string win1 = "Color Image";
    // const string win2 = "Map";
    const string win3 = "Trajectory";
    cv::namedWindow(win1, WINDOW_NORMAL);
    // cv::namedWindow(win2, WINDOW_NORMAL);
    cv::namedWindow(win3, WINDOW_NORMAL);
    cv::Mat image;
    
    // Initialize general objects and variables.
    // My_Map m(width, height, res, true);
    My_Map t(width, height, res);
    std::mutex mut;
    std::ofstream f;
    Img ImgLog;
    
    // Create the log folders. 
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

    // Start the pipeline. 
    p.start(cfg);

    // Start streaming. 
    while (1)
    {
        // Get frame. 
        frames = p.wait_for_frames();
        color = frames.get_color_frame();
        // depth = frames.get_depth_frame();

        // Create color image and save it. 
        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();
        image = cv::Mat(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // Image and timestamp logging. 
        ImgLog.number = count;
        ImgLog.timestamp = color.get_timestamp() / 1000;
        count ++;
        img_suffix = "/img_" + to_string(ImgLog.number) + ".png";
        img_path = img_folder + img_suffix;
        cv::imwrite(img_path, image);
        f.open(time_path, ios::app | ios::out);
        f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << "\n";
        f.close();

        // Draw the trajectory only. 
        mut.lock();
        Quaternion_ q;
        q.w = node->odo_data.ow;
        q.x = node->odo_data.ox;
        q.y = node->odo_data.oy;
        q.z = node->odo_data.oz;
        t.poseUpdate(
            ImgLog.number, 
            node->odo_data.px, 
            node->odo_data.py,
            q);
        mut.unlock();
        t.headingShow();
        t.mapShow();

        // Trajectory logging. 
        traj_suffix = "/trajectory_" + to_string(ImgLog.number) + ".png";
        traj_path = traj_folder + traj_suffix;
        cv::imwrite(traj_path, t.tempMap);

        // Visualization. 
        cv::resizeWindow(win1, cv::Size((int)image.cols / 2, (int)image.rows));
        // cv::resizeWindow(win2, cv::Size(m.map_.cols, m.map_.rows));
        cv::resizeWindow(win3, cv::Size(t.map_.cols, t.map_.rows));
        cv::moveWindow(win1, 0, 0);
        // cv::moveWindow(win2, (image.cols + 70), 0);
        cv::moveWindow(win3, (image.cols + 70), (t.map_.rows + 250));
        cv::imshow(win1, image);
        // cv::imshow(win2, m.tempMap);
        cv::imshow(win3, t.tempMap);
        char c = cv::waitKey(10);

        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            cv::imwrite(traj_final_path, t.tempMap);
            TERMINATE = true;
            break;
        }
    }

    return 0;
}
