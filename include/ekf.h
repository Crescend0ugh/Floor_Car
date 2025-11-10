#pragma once

#include "kalman/LinearizedMeasurementModel.hpp"
#include "kalman/LinearizedSystemModel.hpp"
#include "kalman/UnscentedKalmanFilter.hpp"

#include "vector.h"

#define declare_field(field_name, id) \
    static constexpr size_t _##field_name = (id); \
    float (field_name)() const { return (*this)[_##field_name]; } \
    float& (field_name)() { return (*this)[_##field_name]; } \

namespace model
{
    class state : public Kalman::Vector<float, 19>
    {
    public:
        void normalize_quaternion();

        KALMAN_VECTOR(state, float, 19)

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

            // Angular velocities about x, y and z axes
            declare_field(wx, 10)
            declare_field(wy, 11)
            declare_field(wz, 12)

            // IMU accelerometer biases
            declare_field(bias_ax, 13)
            declare_field(bias_ay, 14)
            declare_field(bias_az, 15)

            // IMU gyroscope biases
            declare_field(bias_gx, 16)
            declare_field(bias_gy, 17)
            declare_field(bias_gz, 18)
    };

    // IMU accelerometer and gyroscope readings
    class control : public Kalman::Vector<float, 6>
    {
    public:
        KALMAN_VECTOR(control, float, 6)
            // Acceleration
            declare_field(ax, 0)
            declare_field(ay, 1)
            declare_field(az, 2)

            // Gyroscope
            declare_field(gx, 3)
            declare_field(gy, 4)
            declare_field(gz, 5)
    };

    template<template<class> class CovarianceBase = Kalman::StandardBase>
    class system_model : public Kalman::LinearizedSystemModel<state, control, CovarianceBase>
    {
    public:
        // The non-linear state transition function
        state f(const state& current, const control& control) const;
    };

    class encoder_measurement : public Kalman::Vector<float, 2>
    {
    public:
        KALMAN_VECTOR(encoder_measurement, float, 2)
            // Linear velocity forwards
            declare_field(vel_forward, 0)
            // Angular velocity about yaw axis
            declare_field(wz, 1)
    };

    template<template<class> class CovarianceBase = Kalman::StandardBase>
    class encoder_measurement_model : public Kalman::LinearizedMeasurementModel<state, encoder_measurement, CovarianceBase>
    {
    public:
        encoder_measurement_model()
        {
            this->V.setIdentity();
        }

        // Predict sensor measurement from current state to limit accelerometer velocity error
        encoder_measurement h(const state& current) const;
    };
}

namespace robo
{
    // These are the changes over a discrete time step
    // So multiply the raw IMU readings by the time step (0.01)
    struct imu_reading
    {
        float ax;
        float ay;
        float az;

        float gx;
        float gy;
        float gz;
    };

    // The angular velocities of a left and right wheel
    struct encoder_reading
    {
        float vel_left;
        float vel_right;
    };

    class ukf
    {
    private:
        model::state state;
        model::control control;
        model::system_model<> system;

        Kalman::UnscentedKalmanFilter<model::state> predictor;
        model::encoder_measurement_model<> encoder_measurement_model;

        float wheel_radius;
        float wheel_base_y; // Distance between left and right wheels
    public:
        ukf(float wheel_radius, float wheel_base_y);
        void predict(const imu_reading& imu);
        void update_encoders(const encoder_reading& encoders);
        Eigen::Affine3f get_transform() const;
    };
}