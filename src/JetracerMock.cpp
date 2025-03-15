//
// Created by arno on 3/14/25.
//

#include "jetracer_controller/JetracerMock.h"

namespace JetracerController {
    JetracerMock::JetracerMock(JetracerCreateInfo create_info, rclcpp::Logger logger)
        : IJetracer(logger), serial_port(create_info.serial_port), baud_rate(create_info.baud_rate),
          kp(create_info.kp), ki(create_info.ki), kd(create_info.kd),
          linear_correction(create_info.linear_correction), servo_bias(create_info.servo_bias),
          a(create_info.a), b(create_info.b), c(create_info.c), d(create_info.d){
    }

    JetracerMock::~JetracerMock() {
        RCLCPP_INFO(this->get_logger(), "destroy");
    }


    bool JetracerMock::init() {
        RCLCPP_INFO(this->get_logger(), "init");
        SetParams(kp, ki, kd, linear_correction, servo_bias);
        SetCoefficient(a, b, c, d);
        return true;
    }

    bool JetracerMock::activate() {
        RCLCPP_INFO(this->get_logger(), "activate");
        return true;
    }

    bool JetracerMock::deactivate() {
        RCLCPP_INFO(this->get_logger(), "deactivate");
        return true;
    }


    bool JetracerMock::SetParams(int kp, int ki, int kd, double linear_correction, int servo_bias) {
        RCLCPP_INFO(this->get_logger(), "Set params");
        RCLCPP_INFO(this->get_logger(), "serial_port: %s", serial_port.c_str());
        RCLCPP_INFO(this->get_logger(), "baud_rate: %d", baud_rate);
        RCLCPP_INFO(this->get_logger(), "kp: %d", kp);
        RCLCPP_INFO(this->get_logger(), "ki: %d", ki);
        RCLCPP_INFO(this->get_logger(), "kd: %d", kd);
        RCLCPP_INFO(this->get_logger(), "linear_correction: %f", linear_correction);
        RCLCPP_INFO(this->get_logger(), "servo_bias: %d", servo_bias);
        return true;
    }

    bool JetracerMock::SetCoefficient(float a, float b, float c, float d) {
        RCLCPP_INFO(this->get_logger(), "set coefficient");
        RCLCPP_INFO(this->get_logger(), "a: %f", a);
        RCLCPP_INFO(this->get_logger(), "b: %f", b);
        RCLCPP_INFO(this->get_logger(), "c: %f", c);
        RCLCPP_INFO(this->get_logger(), "d: %f", d);
        return true;
    }

    bool JetracerMock::setVelocity(const double x, const double y, const double yaw) {
        RCLCPP_INFO(this->get_logger(), "set_velocity: x=%f, y=%f, yaw=%f", x, y, yaw);
        motorStates.motor_lset = x;
        motorStates.motor_rset = yaw;
        motorStates.motor_lvel = x;
        motorStates.motor_rvel = yaw;
        return true;
    }
} // jetracerController