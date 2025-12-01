#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}

return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info)
{
    if (configure_default(info) != return_type::OK) {
        return return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "Configuring DiffDriveArduino...");

    time_ = std::chrono::system_clock::now();

    // Use the keys from XACRO
    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    // Set up the wheels
    l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

    // Set up the Arduino
    arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

    // Debug: check connection
    if (!arduino_.connected()) {
        RCLCPP_ERROR(logger_, "Arduino NOT connected on %s", cfg_.device.c_str());
        return return_type::ERROR;
    } else {
        RCLCPP_INFO(logger_, "Arduino connected on %s", cfg_.device.c_str());
    }

    RCLCPP_INFO(logger_, "Finished Configuration");
    status_ = hardware_interface::status::CONFIGURED;
    return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));
    return command_interfaces;
}

return_type DiffDriveArduino::start()
{
    RCLCPP_INFO(logger_, "Starting Controller...");
    arduino_.sendEmptyMsg();
    arduino_.setPidValues(30, 20, 0, 100);
    status_ = hardware_interface::status::STARTED;
    return return_type::OK;
}

return_type DiffDriveArduino::stop()
{
    RCLCPP_INFO(logger_, "Stopping Controller...");
    status_ = hardware_interface::status::STOPPED;
    return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::read()
{
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    double deltaSeconds = diff.count();
    time_ = new_time;

    if (!arduino_.connected()) {
        return return_type::ERROR;
    }

    // Read raw encoder values from Arduino
    arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

    // Previous positions
    double pos_prev = l_wheel_.pos;
    l_wheel_.pos = l_wheel_.calcEncAngle();   // raw angle in radians
    r_wheel_.pos = r_wheel_.calcEncAngle();   // raw angle in radians

    // ✅ Apply scaling factor (real robot rotates twice → RViz sees one)
    const double scale_factor = 1.838;  // adjust this if needed
    l_wheel_.pos *= scale_factor;
    r_wheel_.pos *= scale_factor;

    // Compute wheel velocities
    l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

    pos_prev = r_wheel_.pos;
    r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

    return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::write()
{
    if (!arduino_.connected()) {
        return return_type::ERROR;
    }

    arduino_.setMotorValues(
        l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate,
        r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate
    );

    return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(DiffDriveArduino, hardware_interface::SystemInterface)
