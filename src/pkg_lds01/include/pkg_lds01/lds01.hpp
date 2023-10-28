#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "boost/asio.hpp"
#include "boost/array.hpp"


class NodeLds01 : public rclcpp::Node
{
    public:
        NodeLds01();
        ~NodeLds01();

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_laser;

        
        sensor_msgs::msg::LaserScan::SharedPtr msg_laser;

        boost::asio::io_service loop;
        boost::asio::serial_port serial_laser;


        // boost::array<uint8_t, 2520> raw_bytes;
        uint8_t start_count;
        bool shutting_down_;
        bool got_scan;
        uint8_t good_sets;
        uint32_t motor_speed;
        uint16_t rpms;
        int index;



        void poll();

        

};