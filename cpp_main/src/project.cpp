#include "utils.h"
#include "functions.h"


int main(int argc, char * argv[])
{
    string command;
    // vector<string> commands{"replay", "trajectory", "map", "stream_test"};
    map<string, int> commands = {
        {"replay", 0}, 
        {"trajectory", 1}, 
        {"map", 2}, 
        {"stream_test", 3},
        {"None", 4},
        {"test", 5}};
    
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
        case 0:
        {
            //// test replay. 
            if (argc > 2)
            {
                replay(argv[2]);
            }
            else
            {
                replay(REPLAY_PATH);
            }

            break;
        }

        case 1:
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

        case 2:
        {
            // test map building. 
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
                thread thread2 (stream_map_test, node, 100, 100, 5);
                thread1.join();
                thread2.join();
            }
            
            break;
        }

        // case 3:
        // {
        //     // test Intel camera streaming. 
        //     stream_test();
        //     break;
        // }

        case 4:
        {
            printf("\n\nPlease enter one of below commands: \n\n");

            for (auto c : commands)
            {
                cout << "  =>  " << c.first << "\n";
            }

            break;
        }

        case 5:
        {
            for (int i = 0; i < argc; i++)
            {
                printf("No. %d argument = %s. \n\n", i, argv[i]);
            }
        }

        default:
        {
            printf("\n\nPlease enter the correct command! \n\n");
            break;
        }

    }

    ProcessDone();

    return 0;
}
