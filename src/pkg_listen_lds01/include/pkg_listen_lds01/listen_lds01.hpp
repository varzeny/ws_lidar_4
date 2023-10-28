#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class NodeListenLds01 : public rclcpp::Node
{
    public:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_laser;


        NodeListenLds01();
        ~NodeListenLds01();

        void callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg_laser);
};