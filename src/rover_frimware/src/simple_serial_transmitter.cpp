#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include<libserial/SerialPort.h>

#include <chrono>
using std::placeholders::_1;
using namespace std::chrono_literals;


class SimpleSerialTransmitter : public rclcpp::Node
{
    public:
        SimpleSerialTransmitter() : Node("simple_sub"){
            declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
            port_name_ = get_parameter("port_name").as_string();
            serial_port_.Open(port_name_);
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

            sub_ = create_subscription<std_msgs::msg::String>("serial_transmitter", 10,bind(&SimpleSerialTransmitter::msgCallback,this,_1));
        }
        ~SimpleSerialTransmitter()
        {
            serial_port_.Close();
        }
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
        LibSerial::SerialPort serial_port_;
        std::string port_name_;
        void msgCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            serial_port_.Write(msg->data);
        } 
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSerialTransmitter>());
    rclcpp::shutdown();
    return 0;
}