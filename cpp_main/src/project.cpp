#include "utils.h"
#include "functions.h"


int main(int argc, char * argv[])
{
    // Initialize the variables for commands. 
    string command;
    map<string, int> commands = {
        {"replay_from_images", 1}, 
        // {"trajectory", 2}, 
        {"stream_map", 3},
        // {"single_frame_map", 4}, 
        // {"None", 5},
        {"debug", 6},
        {"replay_from_odometry", 7},
        // {"communication", 8}, 
        // {"map_demo", 9},
        {"delay_test", 10}, 
        // {"field_trip", 11},
        {"recording", 12},
        {"stream_map_from_recording", 13},
        {"image_extraction", 14}
        };
    vector<string> temp_commands{
        "replay_from_images", 
        // "trajectory", 
        "stream_map", 
        // "single_frame_map", 
        // "None", 
        "debug",
        "replay_from_odometry", 
        // "communication", 
        // "map_demo", 
        "delay_test", 
        // "field_trip",
        "recording",
        "stream_map_from_recording",
        "image_extraction"
        };
    
    // Get into the command selection menu. 
    if (argc > 1)
    {
        bool isWrong = true;

        // check if the command is correct. 
        for (int i = 0; i < temp_commands.size(); i++)
        {
            if (argv[1] == temp_commands[i])
            {
                isWrong = false;
                break;
            }
        }

        if (!isWrong)
        {
            command = argv[1];
        }
        else
        {
            printf("\n\nPlease select one of below functions: \n\n");

            for (int i = 0; i < temp_commands.size(); i++)
            {
                printf("%d  ->  %s \n\n", i, temp_commands[i].c_str());
            }

            int temp_command;
            cin >> temp_command;
            command = temp_commands[temp_command];
        }
    }
    else
    {
        // command = "None";
        printf("\nPlease select one of below commands after planner: \n\n");

        for (int i = 0; i < temp_commands.size(); i++)
        {
            printf("%d  ->  %s \n\n", i, temp_commands[i].c_str());
        }

        int temp_command;
        cin >> temp_command;
        command = temp_commands[temp_command];
    }
    
    switch (commands[command])
    {
        case 1:  // Test replay purely from images. 
        {
            bool isWrong = true;
            int i = 0;

            // See how many folders are there in the main folder. 
            vector<string> folders;
            vector<string> subFolders;
            vector<string> modes{"trajectory", "map"};
            std::filesystem::path P {REPLAY_FOLDER};
            
            for (auto& p : std::filesystem::directory_iterator(P))
            {
                folders.push_back(p.path().filename());
            }

            sort(folders.begin(), folders.end());

            // Choose subfolder to get in. 
            printf("Please select which folder you want to get in. The availbale options are shown below: \n\n");
            
            for (i = 0; i < folders.size(); i++)
            {
                printf("%d  ->  %s \n\n", i, folders[i].c_str());
            }

            int folder;
            cin >> folder;
            string temp = REPLAY_FOLDER + folders[folder];

            // See how many folders are there in the sub folder. 
            std::filesystem::path P1 {temp};

            for (auto& p : std::filesystem::directory_iterator(P1))
            {
                subFolders.push_back(p.path().filename());
            }

            sort(subFolders.begin(), subFolders.end());
            printf("\n\nPlease select which folder you want to replay. The availbale options are shown below: \n\n");
            
            for (i = 0; i < subFolders.size(); i++)
            {
                printf("%d  ->  %s \n\n", i, subFolders[i].c_str());
            }

            int subFolder;
            cin >> subFolder;

            // select replay mode.
            printf("\n\nPlease select which mode you want to use, the available options are shown below: \n\n");
            
            for (int i = 0; i < modes.size(); i++)
            {
                printf("%d  ->  %s \n\n", i, modes[i].c_str());
            }

            int mode;
            cin >> mode;
            temp = folders[folder] + "/" + subFolders[subFolder];
            replay_from_images(temp, modes[mode]);
            break;
        }

        case 2:
        {
            // Test trajectory building. 
            rclcpp::init(argc, argv);
            std::shared_ptr<Mike> node = std::make_shared<Mike>();

            if (argc > 2)
            {
                thread thread1 (Communication, node);
                thread thread2 (stream_test, node, stoi(argv[2]), stoi(argv[3]), stoi(argv[4]));
                thread1.join();
                thread2.join();
            }
            else
            {
                thread thread1 (Communication, node);
                thread thread2 (stream_test, node, 30, 30, 5);
                thread1.join();
                thread2.join();
            }

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            fstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [trajectory] is used. \n";
            f.close(); 
            break;
        }

        case 3:
        {
            // test map building in streaming. 
            rclcpp::init(argc, argv);
            std::shared_ptr<Mike> node = std::make_shared<Mike>();
            if (argc > 2)
            {
                thread thread1 (Communication, node);
                thread thread2 (stream_map_test, node, stoi(argv[2]), stoi(argv[3]), stoi(argv[4]));
                thread1.join();
                thread2.join();
            }
            else
            {
                printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
                int map_width_meter, map_height_meter, map_res;
                cin >> map_width_meter >> map_height_meter >> map_res; 
 
                thread thread1 (Communication, node);
                thread thread2 (stream_map_test, node, map_width_meter, map_height_meter, map_res);
                thread1.join();
                thread2.join();
            }

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            fstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [stream_map] is used. \n";
            f.close(); 
            break;
        }

        case 4:
        {
            // // Test map building in single frame. 
            // rclcpp::init(argc, argv);
            // std::shared_ptr<Mike> node = std::make_shared<Mike>();

            // if (argc > 2)
            // {
            //     thread thread1 (Communication, node);
            //     thread thread2 (single_frame_map_test, node, stoi(argv[2]), stoi(argv[3]), stoi(argv[4]));
            //     thread1.join();
            //     thread2.join();
            // }
            // else
            // {
            //     printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
            //     int map_width_meter, map_height_meter, map_res;
            //     cin >> map_width_meter >> map_height_meter >> map_res; 
            //     thread thread1 (Communication, node);
            //     thread thread2 (single_frame_map_test, node, map_width_meter, map_height_meter, map_res);
            //     thread1.join();
            //     thread2.join();
            // }

            // // Write a note to specify which command is executed in this folder. 
            // string node_path = node->log_path + "/Mode.txt";
            // fstream f;
            // f.open(node_path, ios::out | ios::app);
            // f << "Command [" << temp_commands[3] << "] is used. \n";
            // f.close(); 
            break;
        }

        case 5:
        {
            // No command is typed.
            printf("\n\nPlease enter one of below commands after planner: \n\n");

            for (auto c : commands)
            {
                cout << "  =>  " << c.first << "\n";
            }

            break;
        }

        case 6:
        {
            // // test how the auguments from the terminal are received. 
            // for (int i = 0; i < argc; i++)
            // {
            //     printf("No. %d argument = %s. \n\n", i, argv[i]);
            // }

            // // test time format as folder name. 
            // string time = getTime();
            // printf("\n\nThe current time is: %s. \n\n", time.c_str());
            // string main_path = "/home/mike/RobotLog/";
            // string file_path = main_path + time;
            // if (create_directories(file_path))
            // {
            //     printf("\n\nDirectory [%s] is created. \n\n", file_path.c_str());
            // }
            // else
            // {
            //     printf("\n\nDirectory creation is failed. \n\n");
            // }

            // // pointcloud debug
            // pointcloud_debug(stoi(argv[2]), stoi(argv[3]), stoi(argv[4]));

            // // map projection debug
            // rclcpp::init(argc, argv);
            // std::shared_ptr<Mike> node = std::make_shared<Mike>();
            // printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
            // int map_width_meter, map_height_meter, map_res;
            // cin >> map_width_meter >> map_height_meter >> map_res; 
            // printf("\n\nPlease enter which projection mode you want to use: \n\n");
            // string mode;
            // cin >> mode;
            // thread thread1 (Communication, node);
            // thread thread2 (map_projection_debug, node, map_width_meter, map_height_meter, map_res, mode);
            // thread1.join();
            // thread2.join();

            // simple test 
            simple_test();

            // // Write a note to specify which command is executed in this folder. 
            // string node_path = node->log_path + "/Mode.txt";
            // fstream f;
            // f.open(node_path, ios::out | ios::app);
            // f << "Command [" << argv[1] << "] is used. \n";
            // f.close(); 
            break;
        }

        case 7:
        {
            // Test replay from odometry logs. 
            bool isWrong = true;
            int i = 0;

            // See how many folders are there in the destinaiton. 
            vector<string> folders;
            std::filesystem::path P {REPLAY_FOLDER};
            for (auto& p : std::filesystem::directory_iterator(P))
            {
                folders.push_back(p.path().filename());
            }
            sort(folders.begin(), folders.end());

            if (argc > 2)
            {
                // Check if the input folder name is correct. 
                for (i = 0; i < folders.size(); i++)
                {
                    if (argv[2] == folders[i])
                    {
                        isWrong = false;
                        break;
                    }
                }
                
                if (isWrong)
                {
                    printf("\n\nPlease type the correct folder name you want to replay! \n\n");
                    printf("The available folder names are shown below: \n\n");
                    for (i = 0; i < folders.size(); i++)
                    {
                        printf("%d  ->  %s \n\n", i, folders[i].c_str());
                    }
                    int folder;
                    cin >> folder;
                    printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
                    int map_width_meter, map_height_meter, map_res;
                    cin >> map_width_meter >> map_height_meter >> map_res; 
                    replay_from_odometry(folders[folder], map_width_meter, map_height_meter, map_res);
                }
                else
                {
                    printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
                    int map_width_meter, map_height_meter, map_res;
                    cin >> map_width_meter >> map_height_meter >> map_res; 
                    replay_from_odometry(argv[2], map_width_meter, map_height_meter, map_res);
                }

            }
            else
            {
                printf("\n\nPlease specify which folder you want to replay! \n\n");
                printf("The available folder names are shown below: \n\n");
                for (i = 0; i < folders.size(); i++)
                {
                    printf("%d  ->  %s \n\n", i, folders[i].c_str());
                }
                int folder;
                cin >> folder;
                printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
                int map_width_meter, map_height_meter, map_res;
                cin >> map_width_meter >> map_height_meter >> map_res; 
                replay_from_odometry(folders[folder], map_width_meter, map_height_meter, map_res);
            }

            break;
        }

        case 8:
        {
            rclcpp::init(argc, argv);
            std::shared_ptr<Mike> node = std::make_shared<Mike>();
            Communication(node);

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            fstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [communication] is used. \n";
            f.close(); 
            break;
        }

        case 10:
        {
            // Delay test
            rclcpp::init(argc, argv);
            std::shared_ptr<Mike> node = std::make_shared<Mike>();
            thread thread1 (Communication, node);
            thread thread2 (delay_test, node);
            thread1.join();
            thread2.join();

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            ofstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [delay_test] is used. \n";
            f.close(); 
            break;
        }


        case 11:
        {
            // Field trip
            rclcpp::init(argc, argv);
            std::shared_ptr<Mike> node = std::make_shared<Mike>();
            printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
            int map_width_meter, map_height_meter, map_res;
            cin >> map_width_meter >> map_height_meter >> map_res; 
            thread thread1 (Communication, node);
            thread thread2 (field_trip, node, map_width_meter, map_height_meter, map_res);
            thread1.join();
            thread2.join();

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            ofstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [field_trip] is used. \n";
            f.close(); 
            break;
        }

        case 12:
        {
            // Only recording. 
            rclcpp::init(argc, argv);
            std::shared_ptr<Mike> node = std::make_shared<Mike>();

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            ofstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [recording] is used. \n";
            f.close(); 

            printf("\n\nHow long you want to record? \n\nChoose the unit first: (second [s] or minute [m]) \n\nOr record until termination keys are pressed. ([u]) \n\n" );
            string unit;
            double duration = .0;
            cin >> unit;
            
            if (unit == "m" || unit == "s")
            {
                printf("\n\nHow long you want to record? \n\n" );
                cin >> duration;

                if (unit == "m")
                {
                    duration *= 60;
                }

                duration = getActualDuration(duration);
            }
            else if (unit == "u")
            {
                duration = 10 * 60;
            }
            else
            {
                cerr << "\n\nWrong unit is selected! \n\n";
                exit(-1);
            }

            printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
            int map_width_meter, map_height_meter, map_res;
            cin >> map_width_meter >> map_height_meter >> map_res; 

            // Start threading. 
            thread thread1 (Communication, node);
            thread thread2 (recording, node, duration, map_width_meter, map_height_meter, map_res);
            thread1.join();
            thread2.join();

            break;
        }

        case 13:
        {
            // Test map building in streaming which is from a recording. 
            vector<string> folders;
            
            // See how many folders are there in the main folder. 
            std::filesystem::path P {RECORDING_FOLDER};
            for (auto& p : std::filesystem::directory_iterator(P))
            {
                folders.push_back(p.path().filename());
            }
            sort(folders.begin(), folders.end());

            // Choose folder to use. 
            printf("\n\nPlease select which folder you want to use. The availbale options are shown below: \n\n");
            for (int i = 0; i < folders.size(); i++)
            {
                printf("%d  ->  %s \n\n", i, folders[i].c_str());
            }
            int folder;
            cin >> folder;
            string temp = RECORDING_FOLDER + folders[folder] + "/";
            printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
            int map_width_meter, map_height_meter, map_res;
            cin >> map_width_meter >> map_height_meter >> map_res; 
            stream_map_test_from_recording(temp, map_width_meter, map_height_meter, map_res);
            break;
        }

        case 14:
        {
            printf("\n\nPlease enter the serial number of map you want to extract: \n\n");
            int number, ROISize, offsetX, offsetY;
            cin >> number;
            printf("\n\nPlease enter the size of ROI and the center offset : \n\n");
            cin >> ROISize >> offsetX >> offsetY;
            image_extraction(number, ROISize, offsetX, offsetY);
            break;
        }

        default:
        {
            // Incorrect command is detected. . 
            printf("\n\nPlease enter the correct command! \n\n");
            printf("Avalibale commands are shown below, please choose one of them: \n\n");

            for (auto c : commands)
            {
                cout << "  =>  " << c.first << "\n";
            }

            break;
        }
    }

    ProcessDone();

    return 0;
}
