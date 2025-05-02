#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"

class MyPIDController : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        return {controller_interface::interface_configuration_type::INDIVIDUAL, {"joint1/position", "joint2/position"}};
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        return {controller_interface::interface_configuration_type::INDIVIDUAL, {"joint1/position", "joint2/velocity"}};
    }

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override {
        // Implement PID logic here
        double error = target_position_ - state_position_;
        integral_ += error * period.seconds();
        double derivative = (error - last_error_) / period.seconds();
        double command = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
        last_error_ = error;

        // Write to command interfaces
        command_interfaces_[0].set_value(command);
        return controller_interface::return_type::OK;
    }

    // Add parameters (kp, ki, kd) and reference subscriber
private:
    double kp_ = 1.0, ki_ = 0.0, kd_ = 0.0;
    double target_position_ = 0.0;
    double state_position_ = 0.0;
    double integral_ = 0.0;
    double last_error_ = 0.0;
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MyPIDController, controller_interface::ControllerInterface)