#ifndef ROVER_INTERFACE_HPP
#define ROVER_INTERFACE_HPP

#include<rclcpp_lifecycle/state.hpp>
#include<rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include<rclcpp/rclcpp.hpp>
#include<hardware_interface/system_interface.hpp>
#include<libserial/SerialPort.h>
#include<vector>
#include<string> 
#include <sstream>


namespace rover_firmware{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class RoverInterface : public hardware_interface::SystemInterface
    {
    public:
        RoverInterface(); // constructor  
        virtual ~RoverInterface(); // virtual is needed for proper cleanup of derived classes
        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state ) override; //start the hardware interface, e.g., open serial port, initialize variables
        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state ) override; //stop the hardware interface, e.g., close serial port, reset variables
        virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info ) override; //initialize the hardware interface with the provided hardware info, e.g., parse the hardware info to get serial port parameters
        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override; //export the state interfaces, e.g., velocity and position states
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override; //export the command interfaces, e.g., velocity command
        virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & ) override; //read the state from the hardware, e.g., read velocity and position from the motor controller via serial communication
        virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & ) override; //write the command to the hardware, e.g., send velocity command to the motor controller via serial communication
    private:
        LibSerial::SerialPort serial_port_; // serial port object for communication with the motor controller
        std::string port_name_; // serial port name, e.g., "/dev/ttyUSB0"
        int baud_rate_; // baud rate for serial communication, e.g., 115200
        std::vector<double> velocity_command_;// velocity command to be sent to the motor controller
        std::vector<double> velocity_state_; // velocity state read from the motor controller
        std::vector<double> position_state_; // position state read from the motor controller
        rclcpp::Time last_run_; // time of the last read/write operation مفيد لتتبع الوقت بين عمليات القراءة والكتابة
    };

}
#endif