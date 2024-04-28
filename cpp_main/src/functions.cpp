#include "functions.h"

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
    int count = 0; // serial number of color images, trajectories, maps, depth info.
    Logging l(node);
    l.createDir("trajectory");
    // string img_folder = node->log_path + "/Images";
    // string traj_folder = node->log_path + "/Trajectories";
    // string depth_folder = node->log_path + "/Depth";
    // string map_folder = node->log_path + "/Map";
    // string info_path = node->log_path + "/Info.txt";
    // string bag_path = node->log_path + "/record.bag";
    // string time_path = node->log_path + "/TimeLog.csv";
    // string traj_final_path = node->log_path + "/Trajectory_final.png";
    // string map_final_path = node->log_path + "/Map_final.png";
    // string record_path = node->log_path + "/record.bag";
    // string coordinate_path = node->log_path + "/CoordinateLog.csv";
    // string traj_suffix;
    // string img_suffix;
    // string depth_suffix;
    // string map_suffix;
    // string img_path;
    // string traj_path;
    // string depth_path;
    // string map_path;

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
    cfg.enable_record_to_file(l.bag_path);
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
    std::fstream f;

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

    // if (create_directories(l.img_folder) && create_directories(l.traj_folder))
    // {
    //     printf("\n\nDirectories are created. \n\n");
    // }
    // else
    // {
    //     printf("\n\nDirectory creation is failed. \n\n");
    // }

    // start streaming.
    while (1)
    {
        // get a frame.
        frames = p.wait_for_frames();
        // depth_frame = frames.get_depth_frame();
        color_frame = frames.get_color_frame();

        // get the information of the frame.
        const int w = color_frame.as<rs2::video_frame>().get_width();
        const int h = color_frame.as<rs2::video_frame>().get_height();

        // create color images.
        image = cv::Mat(Size(w, h), CV_8UC3, (void *)color_frame.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // save the image.
        l.img_suffix = "/img_" + to_string(count) + ".png";
        l.img_path = l.img_folder + l.img_suffix;
        cv::imwrite(l.img_path, image);

        // image number and timestamp logging.
        ImgLog.number = count;
        ImgLog.timestamp = color_frame.get_timestamp() / 1000;
        count++;
        f.open(l.time_path, ios::app | ios::out);
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

        // // coordinate logging.
        // f.open(coordinate_path, ios::app | ios::out);
        // f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << ", " \
        // << to_string(m.currentPoint.x_meter) << ", " << to_string(m.currentPoint.y_meter) << ", " \
        // << to_string(m.currentPoint.x_pixel_img) << ", " << to_string(m.currentPoint.y_pixel_img) << ", " \
        // << to_string(m.currentPoint.x_pixel_map) << ", " << to_string(m.currentPoint.y_pixel_map) << "\n";
        // f.close();

        // save the trajectory as an image.
        l.traj_suffix = "/trajectory_" + to_string(ImgLog.number) + ".png";
        l.traj_path = l.traj_folder + l.traj_suffix;
        cv::imwrite(l.traj_path, m.map_);

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
            cv::imwrite(l.traj_final_path, m.map_);
            break;
        }
    }

    // document the general info.
    f.open(l.info_path, ios::app | ios::out);
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
int replay_from_images(string folder_name, string mode)
{
    // Initialize variables to control replay.
    bool isPressed = false;
    float speed = 100.0; // milliseconds
    int pauseCnt = 0;
    float currentSpeed = 100.0, previousSpeed = 100.0;

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
    string map_folder = folder + "/Maps/";
    string time_path = folder + "/TimeLog.csv";
    string img_suffix, traj_suffix, map_suffix, img_full_path, traj_full_path, map_full_path;

    // initialize cv objects.
    cv::Mat scene;
    cv::Mat trajectory;
    cv::Mat map;
    string win1 = "Scene";
    string win2 = "Trajectory";
    string win3 = "Map";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    cv::resizeWindow(win1, 1280 / 2, 720 / 2);
    cv::resizeWindow(win2, 600, 600);
    int img_width, traj_width, img_height, traj_height, map_width, map_height;

    if (mode == "map")
    {
        cv::namedWindow(win3, WINDOW_NORMAL);
        cv::resizeWindow(win3, 600, 600);
        int map_width, map_height;
    }

    // read TimeLog.csv.
    f.open(time_path, ios::in);

    while (getline(f, line))
    {
        row.clear();
        stringstream linestream(line);

        while (getline(linestream, word, ','))
        {
            row.push_back(word);
        }

        timeLog.push_back(stod(row[0]));
    }

    f.close();

    // count the number of files.
    std::filesystem::path P{img_folder};

    for (auto &p : std::filesystem::directory_iterator(P))
    {
        num_files++;
    }

    // Main loop. Start replaying
    count = 0;
    // for (count = 0; count < num_files; count++)
    while (count < num_files)
    {
        img_suffix = "img_" + to_string(count) + ".png";
        traj_suffix = "trajectory_" + to_string(count) + ".png";
        img_full_path = img_folder + img_suffix;
        traj_full_path = traj_folder + traj_suffix;
        scene = cv::imread(img_full_path, cv::IMREAD_COLOR);
        trajectory = cv::imread(traj_full_path, cv::IMREAD_COLOR);

        if (mode == "map")
        {
            map_suffix = "map_" + to_string(count) + ".png";
            map_full_path = map_folder + map_suffix;
            map = cv::imread(map_full_path, cv::IMREAD_COLOR);
            map_height = map.rows;
            map_width = map.cols;
        }

        // if (scene.empty() || trajectory.empty() || map.empty())
        // {
        //     printf("\n\nReading Error!  \n\n");
        //     // printf("\n\nCannot read images from directories [%s] or [%s]. \n\n",
        //     // img_full_path.c_str(),
        //     // traj_full_path.c_str());
        //     break;
        // }

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
        // cv::resizeWindow(win1, (int)img_width / 2, (int)img_height / 2);
        // cv::resizeWindow(win2, traj_width, traj_height);
        // cv::moveWindow(win1, 0, 0);
        // cv::moveWindow(win2, img_width + 70, 0);
        // cv::imshow(win1, scene);
        // cv::imshow(win2, trajectory);

        if (mode == "map")
        {
            // cv::resizeWindow(win1, (int)img_width / 2, (int)img_height / 2);
            // cv::resizeWindow(win3, map_width, map_height);
            // cv::resizeWindow(win2, traj_width, traj_height);
            cv::moveWindow(win1, 0, 0);
            cv::moveWindow(win3, 0, 720 / 2 + 75);
            cv::moveWindow(win2, 680, 720 / 2 + 75);
            cv::imshow(win1, scene);
            cv::imshow(win2, trajectory);
            cv::imshow(win3, map);
        }
        else
        {
            cv::resizeWindow(win1, (int)1280 / 2, (int)720 / 2);
            cv::resizeWindow(win2, traj_width, traj_height);
            // cv::resizeWindow(win3, map_width, map_height);
            cv::moveWindow(win1, 0, 0);
            cv::moveWindow(win2, 1280 + 70, 0);
            // cv::moveWindow(win3, (img_width + 70), (traj_height + 250));
            cv::imshow(win1, scene);
            cv::imshow(win2, trajectory);
            // cv::imshow(win3, map);
        }

        char c = (char)cv::waitKey(speed);

        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }

        if (c == 112)
        {
            printf("\n\nKey [P] is pressed. \n\n");
            if (pauseCnt % 2 == 0)
            {
                speed = 0;
            }
            else
            {
                speed = previousSpeed;
            }
            pauseCnt++;
            isPressed = true;
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

        if (c == 119)
        {
            printf("\n\nKey [W] is pressed, speed up 5 times. \n\n");
            speed /= 2.5;
            isPressed = true;
        }

        if (c == 115)
        {
            printf("\n\nKey [S] is pressed, slow down 5 times. \n\n");
            speed *= 2.5;
            isPressed = true;
        }

        if (!isPressed)
        {
            count++;
        }
        previousSpeed = currentSpeed;
        currentSpeed = speed;
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
    // Prepare folders and other paths.
    int count = 0; // serial number of color images, trajectories, maps, depth info.
    Logging l(node);
    l.createDir("stream_map");

    // Initialize rs2 objects.
    rs2::pipeline p;
    rs2::frameset frames;
    rs2::frame color, depth;
    rs2::config cfg;
    rs2::pointcloud pointcloud;
    rs2::points points;
    int stream_color_width = 848;
    int stream_color_height = 480;
    // int stream_color_width = 1280;
    // int stream_color_height = 720;
    // int stream_depth_width = 1280;
    // int stream_depth_height = 720;
    int stream_depth_width = 848;
    int stream_depth_height = 480;
    int frame_rate = 30;

    if (isEnableFromFile)
    {
        cfg.enable_device_from_file("/home/mike/Documents/20240401_175236.bag");
    }
    else
    {
        cfg.enable_stream(RS2_STREAM_COLOR, stream_color_width, stream_color_height, RS2_FORMAT_RGB8, frame_rate);
        cfg.enable_stream(RS2_STREAM_DEPTH, stream_depth_width, stream_depth_height, RS2_FORMAT_Z16, frame_rate);
        // cfg.enable_stream(RS2_STREAM_INFRARED, 1, stream_depth_width, stream_depth_height, RS2_FORMAT_Y8, frame_rate);
        // cfg.enable_stream(RS2_STREAM_INFRARED, 2, stream_depth_width, stream_depth_height, RS2_FORMAT_Y8, frame_rate);
    }

    if (isRecording)
    {
        cfg.enable_record_to_file(l.bag_path);
    }

    // Initialize pcl objects.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setPosition(0, 450);
    // viewer->setPosition(50, 70);
    viewer->addCoordinateSystem(5, "global");
    viewer->initCameraParameters();

    // Initialize cv objects.
    const string win1 = "Color Image";
    const string win2 = "Map";
    const string win3 = "Trajectory";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    cv::namedWindow(win3, WINDOW_NORMAL);
    cv::resizeWindow(win1, 1280 / 2, 720 / 2);
    cv::resizeWindow(win2, 600, 600);
    cv::resizeWindow(win3, 600, 600);
    cv::Mat image;

    // Initialize other objects.
    My_Map m(width, height, res, true);
    My_Map t(width, height, res);
    std::mutex mut;
    std::fstream f;

    // Initialize other variables.
    Img ImgLog;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pc_layers;
    clock_t start, end;
    clock_t start_whole, end_whole;

    // Clear the debug folder.
    std::filesystem::path P {DEBUG_FOLDER};
    for (auto& p : std::filesystem::directory_iterator(P))
    {
        std::filesystem::remove(p);
    }

    // Start the pipeline.
    p.start(cfg);

    // Start streaming.
    while (1)
    {
        // Get frame.
        start_whole = clock();
        frames = p.wait_for_frames();
        color = frames.get_color_frame();
        depth = frames.get_depth_frame();

        // Create color image and save it.
        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();
        image = cv::Mat(Size(w, h), CV_8UC3, (void *)color.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // Image and timestamp logging.
        ImgLog.number = count;
        ImgLog.timestamp = color.get_timestamp() / 1000;
        count++;
        l.img_suffix = "/img_" + to_string(ImgLog.number) + ".png";
        l.img_path = l.img_folder + l.img_suffix;
        cv::imwrite(l.img_path, image);
        f.open(l.time_path, ios::app | ios::out);
        f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << "\n";
        f.close();

        // Draw the map.
        start = clock();

        // Initialize the info map first. 
        if (ImgLog.number == 0)
        {
            m.initialize(ImgLog.timestamp);
        }

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
        end = clock();
        getDuration(start, end, l.detailed_time_path); // Get the spent time. (1)

        // Calculate realsense pointcloud and convert it into PCL format.
        start = clock();
        points = pointcloud.calculate(depth);
        cloud = Points2PCL(points);
        end = clock();
        getDuration(start, end, l.detailed_time_path); // Get the spent time. (2)

        // Filter the depth map with z-value.
        start = clock();
        filter.setInputCloud(cloud);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(0, 4);
        filter.filter(*cloud_filtered);
        end = clock();
        getDuration(start, end, l.detailed_time_path); // Get the spent time. (3)

        // Divide the pointcloud into grid.
        start = clock();
        GridAnalysis G(cloud_filtered);
        G.setCellSize(pow(res, -1));
        G.setHeightThreshold(.10);
        G.rendering();
        G.divide();

        // Project the grid divided from the pointcloud on the map.
        if (m.isMap)
        {
            m.mapUpdate(G, ImgLog.timestamp);  // at this point, the info map is being updated. 
        }

        end = clock();
        getDuration(start, end, l.detailed_time_path); // Get the spent time. (4)

        // Find the frontier to explore as much as it can. 
        m.findFrontier();

        // Display the map and trajectory.
        m.renderingFromInfoMap();
        m.originShow();
        m.frontierShow();
        m.locShow();
        m.mapShow();
        m.flagReset();
        t.headingShow();
        t.mapShow();
        t.flagReset();

        // // Depth info logging.
        // depth_suffix = "/depth_" + to_string(ImgLog.number) +".ply";
        // depth_path = depth_folder + depth_suffix;
        // PCL2PLY(cloud_filtered, depth_path);

        // Trajectory logging.
        l.traj_suffix = "/trajectory_" + to_string(ImgLog.number) + ".png";
        l.traj_path = l.traj_folder + l.traj_suffix;
        cv::imwrite(l.traj_path, t.tempMap);

        // Map logging.
        l.map_suffix = "/map_" + to_string(ImgLog.number) + ".png";
        l.map_path = l.map_folder + l.map_suffix;
        cv::imwrite(l.map_path, m.tempMap);

        // Visualization.
        pc_layers.push_back(G.cloud);

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

        cv::moveWindow(win1, 0, 0);
        cv::moveWindow(win2, 1280 / 2 + 75, 0);
        cv::moveWindow(win3, 1280 / 2 + 680, 0);
        cv::putText(
            image,
            to_string(ImgLog.timestamp),
            cv::Point(50, 50),
            FONT_HERSHEY_DUPLEX,
            1.0,
            cv::Scalar(0, 0, 255),
            1);
        cv::imshow(win1, image);
        cv::imshow(win2, m.tempMap);
        cv::imshow(win3, t.tempMap);
        char c = cv::waitKey(1);

        viewer->spinOnce(1);

        // Check whether to terminate the programme.
        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }

        // Reset.
        pc_layers.clear();
        viewer->removeAllPointClouds();

        end_whole = clock();
        getDuration(start_whole, end_whole, l.detailed_time_path, true); // Get the spent time. (5)
    }

    // Document the general info.
    f.open(l.info_path, ios::app | ios::out);
    // f << "Map Information" << "\n\n";
    f << "Size of the map (width x height) [meter]: " << to_string(m.width_meter) << " x " << to_string(m.height_meter) << "\n\n";
    f << "Resolution of the map [pixel / meter]: " << to_string(m.res) << "\n\n";
    f << "Size of the map (width x height) [pixel]: " << to_string(m.width_pixel) << " x " << to_string(m.height_pixel) << "\n\n";
    f << "Size of color image (width x height): " << to_string(image.cols) << " x " << to_string(image.rows) << "\n\n";
    f << "Size of depth image (width x height): " << to_string(stream_depth_width) << " x " << to_string(stream_depth_height) << "\n\n";
    f.close();

    return 0;
}

// /**
//  * @brief Create the map with environment information while only take a single frame from camera.
//  *
//  * When getting the arguments from the terminal.
//  *
//  *  the first one is width,
//  *
//  *  the second one is height,
//  *
//  *  the thrid one is res.
//  *
//  * @param node a node that communicates with the capra robot.
//  * @param width width of the map in meter.
//  * @param height height of the map in meter.
//  * @param res resolution of the map.
// */
// int single_frame_map_test(std::shared_ptr<Mike> node, int width, int height, int res)
// {
//     // Prepare folders and other paths.
//     int count = 0;  // serial number of color images, trajectories, maps, depth info.
//     Logging l(node);
//     l.createDir("single_frame_map");

//     // Initialize rs2 objects.
//     rs2::pipeline p;
//     rs2::frameset frames;
//     rs2::frame color, depth;
//     rs2::config cfg;
//     rs2::pointcloud pointcloud;
//     rs2::points points;
//     int stream_color_width = 1280;
//     int stream_color_height = 720;
//     // int stream_depth_width = 848;
//     // int stream_depth_height = 480;
//     int stream_depth_width = 1280;
//     int stream_depth_height = 720;
//     int frame_rate = 30;

//     if (isEnableFromFile)
//     {
//         cfg.enable_device_from_file("/home/mike/Documents/20240401_175236.bag");
//         // cfg.enable_device_from_file("/home/mike/Recording/Room005.bag");
//     }
//     else
//     {
//         cfg.enable_stream(RS2_STREAM_COLOR, stream_color_width, stream_color_height, RS2_FORMAT_RGB8, frame_rate);
//         cfg.enable_stream(RS2_STREAM_DEPTH, stream_depth_width, stream_depth_height, RS2_FORMAT_Z16, frame_rate);
//     }

//     // Initialize pcl objects.
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PassThrough<pcl::PointXYZRGB> filter;

//     // initialize cv objects.
//     const string win1 = "Color Image";
//     const string win2 = "Map";
//     cv::namedWindow(win1, WINDOW_NORMAL);
//     cv::namedWindow(win2, WINDOW_NORMAL);
//     cv::Mat image;

//     // Initialize other objects.
//     My_Map m(width, height, res, true);
//     std::mutex mut;
//     std::fstream f;

//     // Initialize other variables.
//     Img ImgLog;
//     vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pc_layers;

//     // Start the pipeline, and there will be no infinite loop since we only want a single frame.
//     p.start(cfg);

//     // Get color and depth frame.
//     frames = p.wait_for_frames();
//     color = frames.get_color_frame();
//     depth = frames.get_depth_frame();

//     // Create color image and save it.
//     const int w = color.as<rs2::video_frame>().get_width();
//     const int h = color.as<rs2::video_frame>().get_height();
//     image = cv::Mat(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
//     cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

//     // Image and timestamp logging.
//     ImgLog.number = count;
//     ImgLog.timestamp = color.get_timestamp() / 1000;
//     count ++;
//     l.img_suffix = "/img_" + to_string(ImgLog.number) + ".png";
//     l.img_path = l.img_folder + l.img_suffix;
//     cv::imwrite(l.img_path, image);
//     f.open(l.time_path, ios::app | ios::out);
//     f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << "\n";
//     f.close();

//     // Update the pose of the robot.
//     mut.lock();
//     Quaternion_ q;
//     q.w = node->odo_data.ow;
//     q.x = node->odo_data.ox;
//     q.y = node->odo_data.oy;
//     q.z = node->odo_data.oz;
//     m.poseUpdate(
//         ImgLog.number,
//         node->odo_data.px,
//         node->odo_data.py,
//         q);
//     mut.unlock();

//     // Calculate realsense pointcloud and convert it into PCL format.
//     points = pointcloud.calculate(depth);
//     cloud = Points2PCL(points);

//     // // debug
//     // f.open(DEBUG_FILE, ios::out | ios::app);
//     // f << to_string(cloud->points.size()) << "\n";
//     // f.close();

//     // Filter the depth map with z-value.
//     filter.setInputCloud(cloud);
//     filter.setFilterFieldName("z");
//     filter.setFilterLimits(0, 4);
//     filter.filter(*cloud_filtered);

//     // calculate the best path.
//     cv::Scalar rendering;
//     Score S(cloud_filtered);
//     S.setStartZ(0.0);
//     S.setSearchRange(3.5);
//     S.setSearchStep(0.40);
//     S.setSize(0.40);
//     S.setStride(0.5 * S.size);
//     // S.setInlierWeight(0.70);
//     // S.setOutlierWeight(1.80);
//     // S.setDisWeight(1.80);
//     // S.setAngleWeight(0.1);

//     S.rendering();

//     // for (double z = S.search_range; z >= S.start_z; z -= S.search_step)
//     // // for (double z = S.start_z; z < S.search_range; z += S.search_step)
// 	// {
// 	// 	S.get_boundary(z);
// 	// 	S.get_slices(z);
// 	// 	// S.get_score(z, false);
//     //     S.get_height(z);

//     //     if (m.isMap)
//     //     {
//     //         m.mapUpdate(S);
//     //     }

// 	// 	// S.find_best_path();
// 	// }

//     for (double z = S.search_range; z >= S.start_z; z -= S.search_step)
//     {
//         S.get_boundary(z);
//         S.get_slices(z);
//         // S.get_score(z);
//         S.get_height(z);
//         if (m.isMap)
//         {
//             m.mapUpdate(S);
//         }

//     }

//     // Show the heading of the robot, also as an indicator of the robot.
//     // m.headingShow();
//     m.originShow();
//     m.renderingFromInfoMap();
//     m.locShow();
//     m.mapShow();
//     m.flagReset();

//     // Depth info logging.
//     l.depth_suffix = "/depth_" + to_string(ImgLog.number) +".ply";
//     l.depth_path = l.depth_folder + l.depth_suffix;
//     PCL2PLY(cloud_filtered, l.depth_path);

//     // Map logging.
//     l.map_suffix = "/map_" + to_string(ImgLog.number) +".png";
//     l.map_path = l.map_folder + l.map_suffix;
//     cv::imwrite(l.map_path, m.tempMap);

//     // Visualization.
//     pc_layers.push_back(cloud_filtered);
//     pc_layers.push_back(S.cloud);
//     cv::Scalar bg_color(0, 0, 0);
//     pcl::visualization::PCLVisualizer::Ptr viewer = Visualization(
//         pc_layers,
//         bg_color);

//     while (!viewer->wasStopped())
// 	{
//         cv::resizeWindow(win1, cv::Size(image.cols, image.rows));
//         cv::resizeWindow(win2, cv::Size(m.map_.cols, m.map_.rows));
//         cv::moveWindow(win1, 0, 0);
//         cv::moveWindow(win2, (image.cols + 70), 0);
//         cv::imshow(win1, image);
//         cv::imshow(win2, m.tempMap);
//         int c = cv::waitKey(1000);
// 		viewer->spinOnce(1000);
// 		std::this_thread::sleep_for(100ms);

//         // check whether to terminate the programme.
//         if (c == 32 || c == 13 || TERMINATE == true)
//         {
//             printf("\n\nThe programme is terminated by keyboard. \n\n");
//             TERMINATE = true;
//             break;
//         }
// 	}

//     // reset.
//     cv::destroyAllWindows();
//     pc_layers.clear();
//     viewer->removeAllPointClouds();
//     p.stop();

//     return 0;
// }

/**
 * @brief Replay the trajectory and scene based on the save images and log file in csv format. (more robust than "replay")
 * @param folder_name the name of the recording folder.
 * @param width
 * @param height
 * @param res
 */
int replay_from_odometry(string folder_name, int width, int height, int res)
{
    // Initialize flag.
    bool isPressed = false;
    float speed = 100; // milliseconds

    // Initialize loading path.
    string folder = REPLAY_FOLDER + folder_name;
    string img_folder = folder + "/Images/";
    string time_path = folder + "/TimeLog.csv";
    string odo_path = folder + "/OdoLog.csv";
    string info_path = folder + "/info.csv";
    string img_suffix;
    string img_path;

    // Initialize counter.
    int imgNum = 0;
    int num_files = getFilesNum(img_folder);
    int i = 0;
    int mark = 0;

    // Initialize cv objects.
    const string win1 = "Color Image";
    const string win2 = "Trajectory";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::namedWindow(win2, WINDOW_NORMAL);
    cv::Mat scene;
    cv::Mat trajectory;

    // Initialize log containers.
    vector<Odo> odoLog;
    map<string, int> infoLog;
    vector<double> timeLog;

    // Initialize object and variables for file reading.
    fstream f;
    vector<string> row;
    string line, word, temp;
    Odo tempOdo;

    // Initialize variables for data matching based on timestamp.
    double currentTime = 0.0; // sec
    double timeRange = 0.3;   // sec

    // Read TimeLog.csv.
    f.open(time_path, ios::in);

    while (getline(f, line))
    {
        row.clear();
        stringstream linestream(line);

        while (getline(linestream, word, ','))
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

        while (getline(linestream, word, ','))
        {
            row.push_back(word);
        }

        tempOdo.timestamp = stod(row[0]);
        tempOdo.px = stod(row[2]);
        tempOdo.py = stod(row[3]);
        tempOdo.pz = stod(row[4]);
        tempOdo.ox = stod(row[5]);
        tempOdo.oy = stod(row[6]);
        tempOdo.oz = stod(row[7]);
        tempOdo.ow = stod(row[8]);

        // tempOdo.px = stod(row[1]);
        // tempOdo.py = stod(row[2]);
        // tempOdo.pz = stod(row[3]);
        // tempOdo.ox = stod(row[4]);
        // tempOdo.oy = stod(row[5]);
        // tempOdo.oz = stod(row[6]);
        // tempOdo.ow = stod(row[7]);

        // odoLog[stod(row[0])] = tempOdo;
        odoLog.push_back(tempOdo);
    }

    f.close();

    // // read info.csv.
    // f.open(info_path, ios::in);

    // while (getline(f, line))
    // {
    //     row.clear();
    //     stringstream linestream(line);

    //     while(getline(linestream, word, ','))
    //     {
    //         row.push_back(word);
    //     }

    //     infoLog["map_width_meter"] = stoi(row[0]);
    //     infoLog["map_height_meter"] = stoi(row[1]);
    //     infoLog["resolution"] = stoi(row[2]);
    //     infoLog["map_width_pixel"] = stoi(row[3]);
    //     infoLog["map_width_pixel"] = stoi(row[4]);
    //     infoLog["image_width"] = stoi(row[5]);
    //     infoLog["image_height"] = stoi(row[6]);
    //     infoLog["depth_width"] = stoi(row[7]);
    //     infoLog["depth_height"] = stoi(row[8]);
    // }

    // f.close();

    // initialize map drawing.
    My_Map t(width, height, res);

    // Main loop. Start replaying.
    while (imgNum < num_files)
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

        // // Logging to debug.
        // fstream f;
        // f.open(DEBUG_FILE, ios::app | ios::out);
        // f << to_string(imgNum) << ", " \
        // << to_string(timeLog[imgNum]) << ", " \
        // << to_string(mark) << ", " \
        // << to_string(i) << ", " \
        // << to_string(currentOdo.px) << ", " \
        // << to_string(currentOdo.py) << ", " \
        // << to_string(currentOdo.pz) << "\n";
        // f.close();

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
        char c = (char)cv::waitKey(speed);

        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }

        // imgNum++;

        if (c == 100)
        {
            printf("\n\nKey [D] is pressed, 15 frames forward. \n\n");
            imgNum += 15;
            isPressed = true;
        }

        if (c == 119)
        {
            printf("\n\nKey [W] is pressed, speed up 5 times. \n\n");
            speed /= 5;
            isPressed = true;
        }

        if (c == 115)
        {
            printf("\n\nKey [S] is pressed, slow down 5 times. \n\n");
            speed *= 5;
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
    std::fstream f;
    Img ImgLog;
    double pc_time = 0.0, node_time = 0.0, camera_time = 0.0;
    int serial_number = 0;

    // Prepare folders and other paths.
    int count = 0; // serial number of color images, trajectories, maps, depth info.
    Logging l(node);
    l.createDir("delay_test");
    // mut.lock();
    // string img_folder = node->log_path + "/Images";
    // string traj_folder = node->log_path + "/Trajectories";
    // string depth_folder = node->log_path + "/Depth";
    // string map_folder = node->log_path + "/Map";

    // string info_path = node->log_path + "/Info.txt";
    // string bag_path = node->log_path + "/record.bag";
    // string time_path = node->log_path + "/TimeLog.csv";
    // string traj_final_path = node->log_path + "/Trajectory_final.png";
    // string map_final_path = node->log_path + "/Map_final.png";
    // string record_path = node->log_path + "/record.bag";
    // string coordinate_path = node->log_path + "/CoordinateLog.csv";
    // // string debug_path = node->log_path + "/delay_test_only_scene.csv";
    // string debug_path = node->log_path + "/delay_test_scene_and_trajectory.csv";
    // mut.unlock();

    // string traj_suffix;
    // string img_suffix;
    // string depth_suffix;
    // string map_suffix;
    // string img_path;
    // string traj_path;
    // string depth_path;
    // string map_path;

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

    // // Create the folders for logging.
    // if (
    //     create_directories(img_folder) &&
    //     create_directories(traj_folder))
    // {
    //     printf("\n\nDirectories are created. \n\n");
    // }
    // else
    // {
    //     printf("\n\nDirectory creation is failed. \n\n");
    // }

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
        image = cv::Mat(Size(w, h), CV_8UC3, (void *)color_frame.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // Save the image.
        l.img_suffix = "/img_" + to_string(count) + ".png";
        l.img_path = l.img_folder + l.img_suffix;
        cv::imwrite(l.img_path, image);

        // Image number and timestamp logging.
        ImgLog.number = count;
        ImgLog.timestamp = color_frame.get_timestamp() / 1000;
        count++;
        f.open(l.time_path, ios::app | ios::out);
        f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << "\n";
        f.close();

        // Test the delay between the main program and the ROS node.
        const auto currenTime = std::chrono::system_clock::now();
        pc_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(currenTime.time_since_epoch()).count() * MICRO;
        camera_time = ImgLog.timestamp;

        mut.lock();
        node_time = node->odo_data.timestamp;
        serial_number = node->odo_data.serial_number;
        mut.unlock();

        // Logging the delay.
        f.open(l.debug_path, ios::app | ios::out);
        f << to_string(camera_time) << ", "
          << to_string(pc_time) << ", "
          << to_string(node_time) << ", "
          << to_string(serial_number) << "\n";
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
            cv::imwrite(l.traj_final_path, t.tempMap);
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
    int count = 0; // serial number of color images, trajectories, maps, depth info.
    Logging l(node);
    l.createDir("field_trip");

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
        cfg.enable_record_to_file(l.bag_path);
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
    std::fstream f;
    Img ImgLog;

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
        image = cv::Mat(Size(w, h), CV_8UC3, (void *)color.get_data(), Mat::AUTO_STEP);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // Image and timestamp logging.
        ImgLog.number = count;
        ImgLog.timestamp = color.get_timestamp() / 1000;
        count++;
        l.img_suffix = "/img_" + to_string(ImgLog.number) + ".png";
        l.img_path = l.img_folder + l.img_suffix;
        cv::imwrite(l.img_path, image);
        f.open(l.time_path, ios::app | ios::out);
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
        l.traj_suffix = "/trajectory_" + to_string(ImgLog.number) + ".png";
        l.traj_path = l.traj_folder + l.traj_suffix;
        cv::imwrite(l.traj_path, t.tempMap);

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
            cv::imwrite(l.traj_final_path, t.tempMap);
            TERMINATE = true;
            break;
        }
    }

    return 0;
}

int simple_test()
{
    // cv::Vec3d n = cv::Vec3d(1, 2, 3);
    // n += cv::Vec3d(2, 4, 6);

    // cout << "\n\n" << n << endl;
    // printf("(x, y, z) = (%f, %f, %f). \n\n", n[0], n[1], n[2]);

    // n *= 3;

    // cout << "\n\n" << n << endl;
    // printf("(x, y, z) = (%f, %f, %f). \n\n", n[0], n[1], n[2]);

    // n = cv::Scalar(10, 20, 30);
    // cout << "\n\n" << n << endl;
    // printf("(x, y, z) = (%f, %f, %f). \n\n", n[0], n[1], n[2]);

    // vector<vector<int>> n{
    //     {0, 1, 2},
    //     {3, 4 ,5},
    //     {6, 7, 8}};

    // printf("\n\n");

    // for (int i= 0; i < n.size(); i++)
    // {
    //     for (int j = 0; j < n[0].size(); j++)
    //     {
    //         printf("%d ", n[i][j]);
    //     }
    //     printf("\n");
    // }

    // int newCol = 4;
    // int newRow = 4;

    // for (auto& row : n)
    // {
    //     row.resize(newCol);
    // }

    // n.resize(newRow, vector<int>(newCol));

    // printf("\n\n");

    // for (int i= 0; i < n.size(); i++)
    // {
    //     for (int j = 0; j < n[0].size(); j++)
    //     {
    //         printf("%d ", n[i][j]);
    //     }
    //     printf("\n");
    // }

    // double minZ = -3.87;
    // double z = -2.87;
    // cout << "\n\n"
    //      << (z - minZ) / 0.2 << "\n\n";
    // cout << "\n\n"
    //      << fmod((z - minZ), 0.2) << "\n\n";

    // map<CellType, bool> typeList;
    // typeList[Map_Close] = true;
    printf("\n\n");
    // cout << typeList[Map_Close] << endl;
    // cout << typeList[Map_Open] << endl;

    vector<CellType> test;
    test.push_back(Map_Close);
    test = {Map_Open, Frontier_Close};
    for (int i = 0; i < test.size(); i++)
    {
        cout << test[i] << endl;
    }

    return 0;
}

/**
 * @brief Record everything necessary for working offline.
 *
 * Specifically, this function only creates:
 *
 * OdoLog.csv,
 *
 * GPSLog.csv,
 *
 * VelLog.csv,
 *
 * record.bag.
 */
int recording(std::shared_ptr<Mike> node)
{
    // Prepare folders and other paths.
    int count = 0; // serial number of color images, trajectories, maps, depth info.
    Logging l(node);

    // Initialize rs2 objects.
    rs2::pipeline p;
    rs2::frameset frames;
    rs2::frame color, depth;
    rs2::config cfg;
    int stream_color_width = 1280;
    int stream_color_height = 720;
    int stream_depth_width = 848;
    int stream_depth_height = 480;
    // int stream_depth_width = 1280;
    // int stream_depth_height = 720;
    int frame_rate = 30;

    // Configure the Intel camera.
    cfg.enable_stream(RS2_STREAM_COLOR, stream_color_width, stream_color_height, RS2_FORMAT_RGB8, frame_rate);
    cfg.enable_stream(RS2_STREAM_DEPTH, stream_depth_width, stream_depth_height, RS2_FORMAT_Z16, frame_rate);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, stream_depth_width, stream_depth_height, RS2_FORMAT_Y8, frame_rate);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, stream_depth_width, stream_depth_height, RS2_FORMAT_Y8, frame_rate);
    cfg.enable_record_to_file(l.bag_path);

    // Initialize cv objects.
    const string win1 = "Color Image";
    cv::namedWindow(win1, WINDOW_NORMAL);
    cv::Mat image(720, 1280, CV_8UC3, cv::Scalar(0, 0, 0));

    // Start the pipeline.
    p.start(cfg);

    // Start streaming.
    while (1)
    {
        // Get frame.
        // frames = p.wait_for_frames();
        // color = frames.get_color_frame();

        // // Create color image and save it.
        // const int w = color.as<rs2::video_frame>().get_width();
        // const int h = color.as<rs2::video_frame>().get_height();
        // image = cv::Mat(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        // cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // Visualization.
        cv::resizeWindow(win1, cv::Size((int)image.cols / 2, (int)image.rows));
        cv::moveWindow(win1, 0, 0);
        cv::putText(
            image,
            "Recording",
            cv::Point(50, 50),
            FONT_HERSHEY_DUPLEX,
            1.0,
            cv::Scalar(0, 0, 255),
            1);
        cv::imshow(win1, image);
        char c = cv::waitKey(10);

        if (c == 32 || c == 13 || TERMINATE == true)
        {
            printf("\n\nThe programme is terminated by keyboard. \n\n");
            TERMINATE = true;
            break;
        }
    }
    return 0;
}

// /**
//  * @brief Create the map with environment infomation while streaming and robot moving. (from recording)
//  *
//  * This is for working offline.
//  *
//  * @param file folder name that contains all necessary recording and logs.
//  * @param width width of the map in meter.
//  * @param height height of the map in meter.
//  * @param res resolution of the map.
// */
// int stream_map_test_from_recording(string folder, int width, int height, int res)
// {
//     // Prepare folders and other paths.
//     int count = 0;  // serial number of color images, trajectories, maps, depth info.
//     Logging l;
//     l.createDir("stream_map");

//     // Initialize rs2 objects.
//     rs2::pipeline p;
//     rs2::frameset frames;
//     rs2::frame color, depth;
//     rs2::config cfg;
//     rs2::pointcloud pointcloud;
//     rs2::points points;
//     // string temp = folder + "record.bag";
//     cfg.enable_device_from_file(folder + "record.bag", false);
//     int stream_color_width = 1280;
//     int stream_color_height = 720;
//     int stream_depth_width = 1280;
//     int stream_depth_height = 720;

//     // Initialize pcl objects.
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PassThrough<pcl::PointXYZRGB> filter;

//     // Initialize cv objects.
//     const string win1 = "Color Image";
//     const string win2 = "Map";
//     const string win3 = "Trajectory";
//     cv::namedWindow(win1, WINDOW_NORMAL);
//     cv::namedWindow(win2, WINDOW_NORMAL);
//     cv::namedWindow(win3, WINDOW_NORMAL);
//     cv::resizeWindow(win1, 1280 / 2, 720 / 2);
//     cv::resizeWindow(win2, 500, 500);
//     cv::resizeWindow(win3, 500, 500);
//     cv::Mat image;

//     // Initialize general objects.
//     My_Map m(width, height, res, true);
//     My_Map t(width, height, res);

//     // Initialize object and variables for file reading.
//     fstream f;
//     vector<string> row;
//     string line, word, temp;
//     string odo_path = folder + "OdoLog.csv";
//     vector<Odo> odoLog;
//     Odo tempOdo;

//     // Initialize general variables.
//     Img ImgLog;
//     int i = 0;
//     int mark = 0;
//     double currentTime = 0.0; // sec
//     double timeRange = 200 * MILLI; // sec
//     // double timeRange = 0.3; // sec

//     // Read OdoLog.csv.
//     f.open(odo_path, ios::in);

//     while (getline(f, line))
//     {
//         row.clear();
//         stringstream linestream(line);

//         while(getline(linestream, word, ','))
//         {
//             row.push_back(word);
//         }

//         tempOdo.timestamp = stod(row[0]);
//         tempOdo.serial_number = stoi(row[1]);
//         tempOdo.px = stod(row[2]);
//         tempOdo.py = stod(row[3]);
//         tempOdo.pz = stod(row[4]);
//         tempOdo.ox = stod(row[5]);
//         tempOdo.oy = stod(row[6]);
//         tempOdo.oz = stod(row[7]);
//         tempOdo.ow = stod(row[8]);
//         odoLog.push_back(tempOdo);
//     }

//     f.close();

//     // Start the pipeline.
//     p.start(cfg);

//     // Start streaming.
//     while (1)
//     {
//         // Get frame.
//         frames = p.wait_for_frames();
//         color = frames.get_color_frame();
//         depth = frames.get_depth_frame();

//         // Create color image and save it.
//         const int w = color.as<rs2::video_frame>().get_width();
//         const int h = color.as<rs2::video_frame>().get_height();
//         stream_color_height = h;
//         stream_color_width = w;
//         stream_depth_height = depth.as<rs2::video_frame>().get_height();
//         stream_depth_width = depth.as<rs2::video_frame>().get_width();
//         image = cv::Mat(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
//         cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

//         // Image and timestamp logging.
//         ImgLog.number = count;
//         ImgLog.timestamp = color.get_timestamp() / 1000;
//         count ++;
//         l.img_suffix = "/img_" + to_string(ImgLog.number) + ".png";
//         l.img_path = l.img_folder + l.img_suffix;
//         cv::imwrite(l.img_path, image);
//         f.open(l.time_path, ios::app | ios::out);
//         f << to_string(ImgLog.timestamp) << ", " << to_string(ImgLog.number) << "\n";
//         f.close();

//         // Match the current camera timestamp to the OdoLog.csv.
//         currentTime = ImgLog.timestamp;
//         i = mark;
//         Odo currentOdo;

//         for (; i < odoLog.size(); i++)
//         {
//             if (odoLog[i].timestamp >= (currentTime - timeRange) &&
//             odoLog[i].timestamp <= (currentTime + timeRange))
//             {
//                 currentOdo = odoLog[i];
//                 mark = i;
//                 break;
//             }
//         }

//         // Debug
//         f.open(DEBUG_FILE, ios::app | ios::out);
//         f << to_string(currentTime) << ", " \
//         << to_string(currentOdo.timestamp) << "\n";
//         f.close();

//         // Gather the odometry data to update the pose of the robot on the map.
//         Quaternion_ q;
//         q.w = currentOdo.ow;
//         q.x = currentOdo.ox;
//         q.y = currentOdo.oy;
//         q.z = currentOdo.oz;
//         m.poseUpdate(
//             ImgLog.number,
//             currentOdo.px,
//             currentOdo.py,
//             q);
//         t.poseUpdate(
//             ImgLog.number,
//             currentOdo.px,
//             currentOdo.py,
//             q);

//         // Calculate realsense pointcloud and convert it into PCL format.
//         points = pointcloud.calculate(depth);
//         cloud = Points2PCL(points);

//         // Filter the depth map with z-value.
// 		filter.setInputCloud(cloud);
// 		filter.setFilterFieldName("z");
// 		filter.setFilterLimits(0, 4);
// 		filter.filter(*cloud_filtered);

//         // Project the pointcloud to the map.
//         Score S(cloud_filtered);
//         S.setStartZ(0.0);
//         S.setSearchRange(3.5);
//         S.setSearchStep(0.40);
//         S.setSize(0.40);
//         S.setStride(1.0 * S.size);
//         S.rendering();

//         for (double z = S.search_range; z >= S.start_z; z -= S.search_step)
//         {
//             S.get_boundary(z);
//             S.get_slices(z);
//             S.get_height(z);
//             // S.get_score(z);
//             if (m.isMap)
//             {
//                 m.mapUpdate(S);
//             }

//         }

//         m.renderingFromInfoMap();
//         m.originShow();
//         m.locShow();
//         m.mapShow();
//         m.flagReset();
//         t.headingShow();
//         t.mapShow();
//         t.flagReset();

//         // Trajectory logging.
//         l.traj_suffix = "/trajectory_" + to_string(ImgLog.number) + ".png";
//         l.traj_path = l.traj_folder + l.traj_suffix;
//         cv::imwrite(l.traj_path, t.tempMap);

//         // Map logging.
//         l.map_suffix = "/map_" + to_string(ImgLog.number) + ".png";
//         l.map_path = l.map_folder + l.map_suffix;
//         cv::imwrite(l.map_path, m.tempMap);

//         // Visualization.
//         cv::moveWindow(win1, 0, 0);
//         cv::moveWindow(win2, (image.cols / 2 + 75), 0);
//         cv::moveWindow(win3, (image.cols / 2 + 580), 0);
//         cv::putText(
//             image,
//             to_string(ImgLog.timestamp),
// 		    cv::Point(50, 50),
// 		    FONT_HERSHEY_DUPLEX,
// 		    1.0,
// 		    cv::Scalar(0, 0, 255),
// 		    1);
//         cv::imshow(win1, image);
//         cv::imshow(win2, m.tempMap);
//         cv::imshow(win3, t.tempMap);
//         char c = cv::waitKey(10);

//         // Check whether to terminate the programme.
//         if (c == 32 || c == 13 || TERMINATE == true)
//         {
//             printf("\n\nThe programme is terminated by keyboard. \n\n");
//             TERMINATE = true;
//             break;
//         }
//     }

//     // Document the general info.
//     f.open(l.info_path, ios::app | ios::out);
//     // f << "Map Information" << "\n\n";
//     f << "Size of the map (width x height) [meter]: " << to_string(m.width_meter) << " x " << to_string(m.height_meter) << "\n\n";
//     f << "Resolution of the map [pixel / meter]: " << to_string(m.res) << "\n\n";
//     f << "Size of the map (width x height) [pixel]: " << to_string(m.width_pixel) << " x " << to_string(m.height_pixel) << "\n\n";
//     f << "Size of color image (width x height): " << to_string(image.cols) << " x " << to_string(image.rows) << "\n\n";
//     f << "Size of depth image (width x height): " << to_string(stream_depth_width) << " x " << to_string(stream_depth_height) << "\n\n";
//     f.close();

//     return 0;
// }
