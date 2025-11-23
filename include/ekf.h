#pragma once

#include "kalman/MeasurementModel.hpp"
#include "kalman/SystemModel.hpp"
#include "kalman/UnscentedKalmanFilter.hpp"

#define declare_field(field_name, id) \
    static constexpr size_t _##field_name = (id); \
    double (field_name)() const { return (*this)[_##field_name]; } \
    double& (field_name)() { return (*this)[_##field_name]; } \

namespace model
{
    class state : public Kalman::Vector<double, 10>
    {
    public:
        void normalize_quaternion();

        KALMAN_VECTOR(state, double, 10)

            // Position
            declare_field(x, 0)
            declare_field(y, 1)
            declare_field(z, 2)

            // Linear velocity
            declare_field(dx, 3)
            declare_field(dy, 4)
            declare_field(dz, 5)

            // Orientation (quaternion)
            declare_field(qw, 6)
            declare_field(qx, 7)
            declare_field(qy, 8)
            declare_field(qz, 9)
    };

    // IMU accelerometer and gyroscope readings
    class control : public Kalman::Vector<double, 7>
    {
    public:
        KALMAN_VECTOR(control, double, 7)
            // Acceleration
            declare_field(ax, 0)
            declare_field(ay, 1)
            declare_field(az, 2)

            // Gyroscope
            declare_field(gx, 3)
            declare_field(gy, 4)
            declare_field(gz, 5)

            // Time step
            declare_field(dt, 6)
    };

    class system_model : public Kalman::SystemModel<state, control, Kalman::StandardBase>
    {
    public:
        // The non-linear state transition function
        state f(const state& current, const control& control) const;
    };

    class encoder_measurement : public Kalman::Vector<double, 1>
    {
    public:
        KALMAN_VECTOR(encoder_measurement, double, 1)
            // Linear velocity forwards
            declare_field(vel_forward, 0)
    };

    class encoder_measurement_model : public Kalman::MeasurementModel<state, encoder_measurement, Kalman::StandardBase>
    {
    public:
        // Predict sensor measurement from current state to limit accelerometer velocity error
        encoder_measurement h(const state& current) const;
    };
}

namespace robo
{
    struct imu_reading
    {
        double ax;
        double ay;
        double az;

        double gx;
        double gy;
        double gz;
    };

    // The angular velocities of a left and right wheel
    struct encoder_reading
    {
        double vel_left;
        double vel_right;
    };

    class ukf
    {
    private:
        model::system_model system;

        Kalman::UnscentedKalmanFilter<model::state> predictor;
        model::encoder_measurement_model encoder_measurement_model;

        double wheel_radius;
        double wheel_base_y; // Distance between left and right wheels

    public:
        ukf(double wheel_radius, double wheel_base_y);
        void predict(const imu_reading& imu, double delta_time);
        void update_encoders(const encoder_reading& encoders);
        Eigen::Affine3d get_transform() const;

        Eigen::Vector3d get_accel_bias() const;
        Eigen::Vector3d get_gyro_bias() const;
    };
}