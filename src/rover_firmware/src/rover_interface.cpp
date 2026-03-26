#include "rover_firmware/rover_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>


namespace rover_firmware{

RoverInterface::RoverInterface() 
{

}
RoverInterface::~RoverInterface() {
    if (serial_port_.IsOpen()) {
        try{
            serial_port_.Close();
        }
        catch (...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoverInterface"), "Failed to close serial port: " << port_name_);
        }
    }
}
CallbackReturn RoverInterface::on_init(const hardware_interface::HardwareInfo &hardware_info ){
    CallbackReturn resilt = hardware_interface::SystemInterface::on_init(hardware_info);
    if (resilt != CallbackReturn::SUCCESS) {
        return resilt;
    }
    try{
        port_name_ = info_.hardware_parameters.at("port_name");
       // baud_rate_ = std::stoi(hardware_info_.hardware_parameters.at("baud_rate"));
    }
    catch (const std::out_of_range &e) {
        RCLCPP_FATAL(rclcpp::get_logger("RoverInterface"), "no serial port provided in hardware info");
        return CallbackReturn::FAILURE;
    }

    
    velocity_command_.reserve(info_.joints.size());
    velocity_state_.reserve(info_.joints.size());
    position_state_.reserve(info_.joints.size());
    last_run_=  rclcpp::Clock().now();

    return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> RoverInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY, &velocity_state_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, hardware_interface::HW_IF_POSITION, &position_state_[i]));
    }
    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> RoverInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY, &velocity_command_[i]));
    }
    return command_interfaces;
}


CallbackReturn RoverInterface::on_activate(const rclcpp_lifecycle::State &previous_state ) {
    RCLCPP_INFO(rclcpp::get_logger("RoverInterface"), " starting robot hardware interface...");
    velocity_command_ = { 0.0 ,0.0 , 0.0 ,0.0 };
    position_state_ = { 0.0 ,0.0 , 0.0 , 0.0 };
    velocity_state_ = { 0.0 ,0.0 , 0.0 , 0.0 };

    try{
        serial_port_.Open(port_name_);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch (...) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoverInterface"), "Failed to open serial port: " << port_name_);
        return CallbackReturn::FAILURE;
    }
     RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoverInterface"), "hardware started , ready to take commands" );
    return CallbackReturn::SUCCESS;
}
CallbackReturn RoverInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state ) {
    RCLCPP_INFO(rclcpp::get_logger("RoverInterface"), "stopping robot hardware interface...");
    if (serial_port_.IsOpen()) {
        try{
            serial_port_.Close();
        }
        catch (...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoverInterface"), "Failed to close serial port: " << port_name_);
            return CallbackReturn::FAILURE;
        }
    }
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverInterface::read(const rclcpp::Time & time, const rclcpp::Duration & ) {
    // read the state from the hardware, e.g., read velocity and position from the motor controller via serial communication
    // this is a placeholder implementation, you need to replace it with the actual implementation to read from your hardware
    if (serial_port_.IsDataAvailable()) {
        auto dt = (rclcpp::Clock().now() - last_run_).seconds();
        std::string message;
        serial_port_.ReadLine(message);
        std::stringstream ss(message);
        std::string res;
        int multiplier = 1;
        while (std::getline(ss, res, ','))
        {
            multiplier = res[1] == 'P' ? 1 : -1;
            if(res[0] == 'r')
            {
                velocity_state_[0] = multiplier * std::stod(res.substr(2, res.size()));
                position_state_[0] += velocity_state_[0] * dt;
            }
            else if(res[0] == 'l')
            {
                velocity_state_[1] = multiplier * std::stod(res.substr(2, res.size()));
                position_state_[1] += velocity_state_[1] * dt;
            }
        }
        last_run_ = rclcpp::Clock().now();
        
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverInterface::write(const rclcpp::Time & time, const rclcpp::Duration & ) {
    // write the command to the hardware, e.g., send velocity command to the motor controller via serial communication
    // this is a placeholder implementation, you need to replace it with the actual implementation to write to your hardware
    std::stringstream message_stream ;
    char right_wheel_sign = velocity_command_[0] >= 0 ? 'P' : 'n';
    char left_wheel_sign = velocity_command_[1] >= 0 ? 'P' : 'n';
    std::string compensate_zeros_right ="";
    std::string compensate_zeros_left ="";
    if(std::abs(velocity_command_[0]) < 10)
    {
        compensate_zeros_right = "0";
    }
    else{
        compensate_zeros_right = "";
    }
    if(std::abs(velocity_command_[1]) < 10)
    {
        compensate_zeros_left = "0";
    }
    else{
        compensate_zeros_left = "";
    }
    message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_command_[0]) << 
    ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_command_[1]) << ",";
    try
    {
        serial_port_.Write(message_stream.str());
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RoverInterface"), "Failed to write"<< message_stream.str() << "to serial port: " << port_name_);
        return hardware_interface::return_type::ERROR;
    }
    
    return hardware_interface::return_type::OK;
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rover_firmware::RoverInterface, hardware_interface::SystemInterface)