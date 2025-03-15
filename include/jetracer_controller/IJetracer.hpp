//
// Created by arno on 3/14/25.
//

#ifndef IJETRACER_HPP
#define IJETRACER_HPP

namespace JetracerController {
    struct JetracerCreateInfo {
        std::string serial_port;
        int baud_rate;
        int kp, ki, kd;
        double linear_correction;
        int servo_bias;
        float a, b, c, d;
    };

    struct Vec3 {
        double x = 0.0, y = 0.0, z = 0.0;
    };

    struct Odom {
        Vec3 position;
        Vec3 orientation;
        Vec3 linear; // linear velocity
        Vec3 angular; // angular velocity
    };

    struct IMU {
        Vec3 gyro, acc, angle;
    };

    struct MotorStates {
        int16_t motor_lvel = 0.0, motor_rvel = 0.0, motor_lset = 0.0, motor_rset = 0.0;
    };

    struct CovarianceImu {
        float orientation[9] = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 0.05};
        float angularVelocity[9] = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6};
        float linearAcceleration[9] = {1e-2, 0, 0, 0, 0, 0, 0, 0, 0};
    };

    struct CovarianceOdometry {
        float twist[36] = {1e-9, 0,    0,    0,   0,   0,
                           0,    1e-3, 1e-9, 0,   0,   0,
                           0,    0,    1e6,  0,   0,   0,
                           0,    0,    0,    1e6, 0,   0,
                           0,    0,    0,    0,   1e6, 0,
                           0,    0,    0,    0,   0,   0.1 };

        float pose[36] = { 1e-9, 0,    0,    0,   0,   0,
                           0,    1e-3, 1e-9, 0,   0,   0,
                           0,    0,    1e6,  0,   0,   0,
                           0,    0,    0,    1e6, 0,   0,
                           0,    0,    0,    0,   1e6, 0,
                           0,    0,    0,    0,   0,   1e3 };
    };

    class IJetracer {
    public:
        virtual ~IJetracer() = default;

        virtual bool init() = 0;
        virtual bool activate() = 0;
        virtual bool deactivate() = 0;

        virtual bool SetParams(int kp, int ki, int kd, double linear_correction, int servo_bias) = 0;
        virtual bool SetCoefficient(float a, float b, float c, float d) = 0;
        virtual bool setVelocity(double x, double y, double yaw) = 0;

        CovarianceImu getCovarianceImu() {return CovarianceImu(); };
        CovarianceOdometry getCovarianceOdometry() {return CovarianceOdometry();};

        virtual IMU getIMU() = 0;
        virtual Odom getOdometry() = 0;
        virtual MotorStates getMotorStates() = 0;
    };
}
#endif //IJETRACER_HPP
