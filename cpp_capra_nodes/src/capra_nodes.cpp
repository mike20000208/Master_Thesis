#include <iostream>
#include <string>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std;
using namespace std::chrono_literals;

#define ODO_TOPIC "/mike/odo"
#define GPS_TOPIC "/mike/gps"


class Capra_Nodes : public rclcpp::Node
{
public:
    Capra_Nodes() : Node("capra_nodes"), count_(0)
    {
        gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            GPS_TOPIC, 
            10);

        gps_timer = this->create_wall_timer(
            100ms,
            std::bind(&Capra_Nodes::gps_timer_callback, this)
        );

        odo_pub = this->create_publisher<nav_msgs::msg::Odometry>(
            ODO_TOPIC, 
            10);

        odo_timer = this->create_wall_timer(
            100ms,
            std::bind(&Capra_Nodes::odo_timer_callback, this)
        );
    }

private:
    void gps_timer_callback()
    {
        auto msg = sensor_msgs::msg::NavSatFix();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "test";
        msg.altitude = 90.0;
        msg.latitude = 55.0;
        msg.longitude = 12.0;
        msg.status.status = 1;
        msg.status.service = 1;
        gps_pub->publish(msg);
        RCLCPP_INFO(
            this->get_logger(), 
            "Publishing GPS messages to topic [%s]......", 
            GPS_TOPIC);
    }

    void odo_timer_callback()
    {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "test";
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation.x = -0.000000;
        msg.pose.pose.orientation.y = 0.015092;
        msg.pose.pose.orientation.z = -0.000000;
        msg.pose.pose.orientation.w = -0.999886;
        odo_pub->publish(msg);
        RCLCPP_INFO(
            this->get_logger(), 
            "Publishing GPS messages to topic [%s]......", 
            ODO_TOPIC);
    }
    
    // timer
    rclcpp::TimerBase::SharedPtr gps_timer;
    rclcpp::TimerBase::SharedPtr odo_timer;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub;

    size_t count_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Capra_Nodes>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
