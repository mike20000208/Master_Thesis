#include <iostream>
#include <string>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "librealsense2/rs.hpp"
// #include "pcl/io/pcd_io.h"
// #include "pcl-1.10/pcl/io/pcd_io.h"
// #include "pcl-1.10/pcl/visualization/cloud_viewer.h"
// #include "pcl-1.10/pcl/filters/passthrough.h"

using namespace std;
using namespace std::chrono_literals;
using namespace rs2;
// using namespace pcl;

#define TOPIC "/mike/gps"


class Publisher : public rclcpp::Node
{
public:
    Publisher() : Node("gps_pub"), count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(TOPIC, 10);
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&Publisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        auto msg = sensor_msgs::msg::NavSatFix();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "test";
        msg.altitude = 90.0;
        msg.latitude = 55.0;
        msg.longitude = 12.0;
        msg.status.status = 1;
        msg.status.service = 1;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing GPS messages......");
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}