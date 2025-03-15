//
// Created by arno on 3/14/25.
//

#ifndef JETRACERMOCK_H
#define JETRACERMOCK_H
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "IJetracer.hpp"

namespace JetracerController {
    class JetracerMock : public IJetracer {
    public:
        explicit JetracerMock(JetracerCreateInfo create_info, rclcpp::Logger logger);
        ~JetracerMock() final;

        bool init() final;
        bool activate() final;
        bool deactivate() final;

        bool SetParams(int kp, int ki, int kd, double linear_correction, int servo_bias) final;
        bool SetCoefficient(float a, float b, float c, float d) final;

        bool setVelocity(double x, double y, double yaw) final;

        inline IMU getIMU() final {
            return imu;
        }

        inline Odom getOdometry() final {
            return odom;
        }
        inline MotorStates getMotorStates() final {
            return motorStates;
        }

    private:
        // config
        std::string serial_port;
        int baud_rate;
        int kp, ki, kd;
        double linear_correction;
        int servo_bias;
        float a, b, c, d;

        // states
        IMU imu;
        Odom odom;
        MotorStates motorStates;
    };
} // jetracerController

#endif //JETRACERMOCK_H
