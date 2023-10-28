#include "pkg_listen_lds01/listen_lds01.hpp"


NodeListenLds01::NodeListenLds01() : Node("node_listenlds01")
{
    subscriber_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::QoS(rclcpp::SensorDataQoS()),
        std::bind(&NodeListenLds01::callback_laser,this,std::placeholders::_1)
    );
}

NodeListenLds01::~NodeListenLds01()
{

}

void NodeListenLds01::callback_laser(sensor_msgs::msg::LaserScan::SharedPtr msg_laser){
    // RCLCPP_INFO(this->get_logger(),"!!!");
    // RCLCPP_INFO(this->get_logger(), "angle_min: %f, angle_max: %f, range_min: %f, range_max: %f", msg_laser->angle_min, msg_laser->angle_max, msg_laser->range_min, msg_laser->range_max);
    
    RCLCPP_INFO(this->get_logger(),"%s",msg_laser->header.frame_id.c_str());
    
    // for (auto &range : msg_laser->ranges) {
    //     RCLCPP_INFO(this->get_logger(), "Range: %f", range);
    // }

}










int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NodeListenLds01>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
