#include "utils.h"
#include "functions.h"


int main(int argc, char * argv[])
{
    string command;
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
        {"field_trip", 11}};
    
    if (argc > 1)
    {
        command = argv[1];
    }
    else
    {
        command = "None";
    }
    
    switch (commands[command])
    {
        case 1:
        {
            // Test replay purely from images. 
            bool isWrong = true;
            int i = 0;

            // See how many folders are there in the destinaiton. 
            vector<string> folders;
            std::filesystem::path P {REPLAY_FOLDER};
            for (auto& p : std::filesystem::directory_iterator(P))
            {
                folders.push_back(p.path().filename());
            }

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

        case 2:
        {
            // test trajectory building. 
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
            f << "Command [" << argv[1] << "] is used. \n";
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
                thread thread1 (Communication, node);
                thread thread2 (stream_map_test, node, 100, 100, 10);
                thread1.join();
                thread2.join();
            }

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            fstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [" << argv[1] << "] is used. \n";
            f.close(); 
            break;
        }

        case 4:
        {
            // test map building in single frame. 
            rclcpp::init(argc, argv);
            std::shared_ptr<Mike> node = std::make_shared<Mike>();

            if (argc > 2)
            {
                thread thread1 (Communication, node);
                thread thread2 (single_frame_map_test, node, stoi(argv[2]), stoi(argv[3]), stoi(argv[4]));
                thread1.join();
                thread2.join();
            }
            else
            {
                thread thread1 (Communication, node);
                thread thread2 (single_frame_map_test, node, 100, 100, 10);
                thread1.join();
                thread2.join();
            }

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            fstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [" << argv[1] << "] is used. \n";
            f.close(); 
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

            // map projection debug
            rclcpp::init(argc, argv);
            std::shared_ptr<Mike> node = std::make_shared<Mike>();
            printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
            int map_width_meter, map_height_meter, map_res;
            cin >> map_width_meter >> map_height_meter >> map_res; 
            printf("\n\nPlease enter which projection mode you want to use: \n\n");
            string mode;
            cin >> mode;
            thread thread1 (Communication, node);
            thread thread2 (map_projection_debug, node, map_width_meter, map_height_meter, map_res, mode);
            thread1.join();
            thread2.join();

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            fstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [" << argv[1] << "] is used. \n";
            f.close(); 
            break;
        }

        case 7:
        {
            // Test replay from odometry logs. 
            bool isWrong = true;
            if (argc > 2)
            {
                std::filesystem::path P {REPLAY_FOLDER};

                for (auto& p : std::filesystem::directory_iterator(P))
                {
                    if (argv[2] == p.path().filename())
                    {
                        isWrong = false;
                        break;
                    }
                }
                
                if (isWrong)
                {
                    printf("\n\nPlease type the correct folder name you want to replay! \n\n");
                    printf("The available folder names are shown below: \n\n");
                    int cnt = 1;
                    std::filesystem::path P {REPLAY_FOLDER};

                    for (auto& p : std::filesystem::directory_iterator(P))
                    {
                        printf("%d  ->  %s \n\n", cnt, p.path().filename().c_str());
                        cnt ++;
                    }
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
                int cnt = 1;
                std::filesystem::path P {REPLAY_FOLDER};

                for (auto& p : std::filesystem::directory_iterator(P))
                {
                    printf("%d  ->  %s \n\n", cnt, p.path().filename().c_str());
                    cnt ++;
                }
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
            f << "Command [" << argv[1] << "] is used. \n";
            f.close(); 
            break;
        }

        case 9:
        {
            rclcpp::init(argc, argv);
            std::shared_ptr<Mike> node = std::make_shared<Mike>();
            printf("\n\nPlease enter the size of map (width & height [meter]) and the resolution of the map [pixel / meter]: \n\n");
            int map_width_meter, map_height_meter, map_res;
            cin >> map_width_meter >> map_height_meter >> map_res; 
            printf("\n\nPlease enter which projection mode you want to use: \n\nOptions: \n\n  -> color\n\n  -> info\n\n");
            string mode;
            cin >> mode;
            thread thread1 (Communication, node);
            thread thread2 (map_projection_debug, node, map_width_meter, map_height_meter, map_res, mode);
            // thread thread2 (map_projection_debug, node, stoi(argv[2]), stoi(argv[3]), stoi(argv[4]));
            thread1.join();
            thread2.join();

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            fstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [" << argv[1] << "] is used. \n";
            f.close(); 
            break;
        }

        case 10:
        {
            rclcpp::init(argc, argv);
            std::shared_ptr<Mike> node = std::make_shared<Mike>();
            thread thread1 (Communication, node);
            thread thread2 (delay_test, node);
            thread1.join();
            thread2.join();

            // Write a note to specify which command is executed in this folder. 
            string node_path = node->log_path + "/Mode.txt";
            fstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [" << argv[1] << "] is used. \n";
            f.close(); 
            break;
        }


        case 11:
        {
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
            fstream f;
            f.open(node_path, ios::out | ios::app);
            f << "Command [" << argv[1] << "] is used. \n";
            f.close(); 
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
