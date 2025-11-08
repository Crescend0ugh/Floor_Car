#pragma once

#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/LinearizedMeasurementModel.hpp"
#include "kalman/LinearizedSystemModel.hpp"

#include "vector.h"

#define declare_field(field_name, id) \
    static constexpr size_t _##field_name = (id); \
    float (field_name)() const { return (*this)[_##field_name]; } \
    float& (field_name)() { return (*this)[_##field_name]; } \

namespace model
{
    class state : public Kalman::Vector<float, 12>
    {
    public:
        KALMAN_VECTOR(state, float, 12)

            // Position
            declare_field(x, 0)
            declare_field(y, 1)
            declare_field(z, 2)

            // Orientation (in RADIANS)
            declare_field(roll, 3)
            declare_field(pitch, 4)
            declare_field(yaw, 5)

            // Linear velocity
            declare_field(dx, 6)
            declare_field(dy, 7)
            declare_field(dz, 8)

            // Angular velocity
            declare_field(droll, 9)
            declare_field(dpitch, 10)
            declare_field(dyaw, 11)
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
        state f(const state& current, const control& control) const
        {
            state predicted;

            // New angle = old angle + delta angle
            auto new_roll = current.roll() + control.gx();
            auto new_pitch = current.pitch() + control.gy();
            auto new_yaw = current.yaw() + control.gz();

            
            Eigen::Matrix3f rotation_matrix = (Eigen::AngleAxisf(new_roll, Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(new_pitch, Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(new_yaw, Eigen::Vector3f::UnitZ())).toRotationMatrix();

            Eigen::Vector3f world_accel = rotation_matrix * Eigen::Vector3f(control.ax(), control.ay(), control.az());

            // Double integrate acceleration to derive velocity and position
            predicted.dx() = current.dx() + control.ax();
            predicted.dy() = current.dy() + control.ay();
            predicted.dz() = current.dz() + control.az();

            predicted.x() = current.x() + predicted.dx();
            predicted.y() = current.y() + predicted.dy();
            predicted.z() = current.z() + predicted.dz();

            predicted.roll() = new_roll;
            predicted.pitch() = new_pitch;
            predicted.yaw() = new_yaw;
            
            return predicted;
        }

    protected:
        void updateJacobians(const state& state, const control& control)
        {
            // F = df/dx (Jacobian of state transition w.r.t. the state)
            this->F.setZero();

            // partial derivative of x.x() w.r.t. x.x()
            this->F(state::_x, state::_x) = 1;

            this->F(state::_y, state::_y) = 1;

            this->F(state::_y, state::_z) = 1;

            this->W.setIdentity();
        }
    };

    class encoder_measurement : public Kalman::Vector<float, 2>
    {
    public:
        KALMAN_VECTOR(encoder_measurement, float, 2)
            // Linear velocity forwards
            declare_field(vel, 0)
            // Angular velocity about yaw axis
            declare_field(gz, 1)
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
        encoder_measurement h(const state& current) const
        {
            encoder_measurement measurement;

            float predicted_forward_vel = current.dx() * std::cos(current.yaw()) + current.dy() * std::sin(current.yaw());

            measurement.vel() = predicted_forward_vel;

            return measurement;
        }
    protected:
        void updateJacobians(const state& state, const control& control)
        {

        }
    };
}

namespace robo
{
    struct imu_reading
    {
        float ax;
        float ay;
        float az;

        float gx;
        float gy;
        float gz;
    };

    struct encoder_reading
    {

    };

    class ekf
    {
    private:
        model::state state;
        model::control control;
        model::system_model<> system;

        Kalman::ExtendedKalmanFilter<model::state> predictor;

        model::encoder_measurement_model<> encoder_measurement_model;

    public:
        ekf();
        Eigen::Affine3f update(const imu_reading& imu, const encoder_reading& encoder);
    };
}