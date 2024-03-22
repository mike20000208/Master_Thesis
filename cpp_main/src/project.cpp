#include "utils.h"
#include "functions.h"


int main(int argc, char * argv[])
{
    //// test realsense2 library. 
    // stream_test();

    //// test ros node. 
    // rclcpp::init(argc, argv);
    // shared_ptr<Mike> node = make_shared<Mike>();
    // Communication(node);

    //// test time acrquirong and directoried creation. 
    // time_files_test();

    //// test multithread. 
    // if (argc > 1)
    // {
    //     rclcpp::init(argc, argv);
    //     std::shared_ptr<Mike> node = std::make_shared<Mike>();
    //     thread thread1 (Communication, node);
    //     thread thread2 (stream_test, node, stoi(argv[1]), stoi(argv[2]), stoi(argv[3]));
    //     thread1.join();
    //     thread2.join();
    // }
    // else
    // {
    //     rclcpp::init(argc, argv);
    //     std::shared_ptr<Mike> node = std::make_shared<Mike>();
    //     thread thread1 (Communication, node);
    //     thread thread2 (stream_test, node, 30, 30, 5);
    //     thread1.join();
    //     thread2.join();
    // }

    //// test replay. 
    // if (argc > 1)
    // {
    //     replay(argv[1]);
    // }
    // else
    // {
    //     replay(REPLAY_PATH);
    // }


    //// test map
    rclcpp::init(argc, argv);
    std::shared_ptr<Mike> node = std::make_shared<Mike>();

    if (argc > 1)
    {
        thread thread1 (Communication, node);
        thread thread2 (stream_map_test, node, stoi(argv[1]), stoi(argv[2]), stoi(argv[3]));
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

    ProcessDone();

    return 0;
}
