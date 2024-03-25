#include "utils.h"
#include "functions.h"


int main(int argc, char * argv[])
{
    string command;
    // vector<string> commands{"replay", "trajectory", "map", "stream_test"};
    map<string, int> commands = {
        {"replay", 1}, 
        {"trajectory", 2}, 
        {"stream_map", 3},
        {"single_frame_map", 4}, 
        {"None", 5},
        {"test", 6},
        {"log_replay", 7}};
    
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
            //// test replay. 
            if (argc > 2)
            {
                replay(argv[2]);
            }
            else
            {
                // replay(REPLAY_DATE);
                printf("\n\nPlease specify which folder you want to replay! \n\n");
                printf("The available folder names are shown below: \n\n");
                int cnt = 1;
                std::filesystem::path P {REPLAY_FOLDER };

                for (auto& p : std::filesystem::directory_iterator(P))
                {
                    printf("%d  ->  %s \n\n", cnt, p.path().filename().c_str());
                    cnt ++;
                }
            }

            break;
        }

        case 2:
        {
            // test trajectory building. 
            if (argc > 2)
            {
                rclcpp::init(argc, argv);
                std::shared_ptr<Mike> node = std::make_shared<Mike>();
                thread thread1 (Communication, node);
                thread thread2 (stream_test, node, stoi(argv[2]), stoi(argv[3]), stoi(argv[4]));
                thread1.join();
                thread2.join();
            }
            else
            {
                rclcpp::init(argc, argv);
                std::shared_ptr<Mike> node = std::make_shared<Mike>();
                thread thread1 (Communication, node);
                thread thread2 (stream_test, node, 30, 30, 5);
                thread1.join();
                thread2.join();
            }

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
            break;
        }

        case 5:
        {
            // No command is typed.
            printf("\n\nPlease enter one of below commands: \n\n");

            for (auto c : commands)
            {
                cout << "  =>  " << c.first << "\n";
            }

            break;
        }

        case 6:
        {
            // test how the auguments from the terminal are received. 
            for (int i = 0; i < argc; i++)
            {
                printf("No. %d argument = %s. \n\n", i, argv[i]);
            }
        }

        case 7:
        {
            //// test replay from csv log. 
            if (argc > 2)
            {
                log_replay(argv[2]);
            }
            else
            {
                // replay(REPLAY_DATE);
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

        default:
        {
            // incorrect command is detected. . 
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
