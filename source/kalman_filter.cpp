#include <iostream>

#include "kalman_filter.h"

void model::state::normalize_quaternion()
{
    Eigen::Quaterniond q(qw(), qx(), qy(), qz());
    q.normalize();
    qw() = q.w();
    qx() = q.x();
    qy() = q.y();
    qz() = q.z();
}

// The non-linear state transition function
model::state model::system_model::f(const model::state& current, const model::control& control) const
{
    state predicted;

    Eigen::Quaterniond q_current(current.qw(), current.qx(), current.qy(), current.qz());

    Eigen::Vector3d accel_corrected(
        control.ax(),
        control.ay(),
        control.az()
    );

    Eigen::Vector3d gyro_corrected(
        control.gx(),
        control.gy(),
        control.gz()
    );

    double dt = control.dt();

    // Compute new quaternion given angular velocity
    double angle = gyro_corrected.norm() * dt;

    if (angle > 1e-6)
    {
        Eigen::Vector3d axis = gyro_corrected.normalized();

        Eigen::Quaterniond dq;
        dq.w() = std::cos(angle / 2.0);
        dq.vec() = std::sin(angle / 2.0) * axis;

        q_current = q_current * dq;
    }

    q_current.normalize();

    // Update velocity from accelerometer 
    Eigen::Matrix3d rotation_matrix = q_current.toRotationMatrix();

    // Subtract g from z axis, then rotate to body coordinate system
    Eigen::Vector3d accel_world = (rotation_matrix * (accel_corrected - Eigen::Vector3d(0.0, 0.0, 9.81)));

    Eigen::Vector3d vel_old(current.dx(), current.dy(), current.dz());
    Eigen::Vector3d vel_new = vel_old + accel_world * dt;

    // Trapezoidal rule integration
    Eigen::Vector3d v_avg = (vel_old + vel_new) / 2.0;
    Eigen::Vector3d pos_old(current.x(), current.y(), current.z());
    Eigen::Vector3d pos_new = pos_old + v_avg * dt;

    predicted.x() = pos_new(0);
    predicted.y() = pos_new(1);
    predicted.z() = pos_new(2);

    predicted.dx() = vel_new(0);
    predicted.dy() = vel_new(1);
    predicted.dz() = vel_new(2);

    predicted.qw() = q_current.w();
    predicted.qx() = q_current.x();
    predicted.qy() = q_current.y();
    predicted.qz() = q_current.z();

    return predicted;
}

model::encoder_measurement model::encoder_measurement_model::h(const model::state& current) const
{
    encoder_measurement measurement;

    Eigen::Quaterniond q(current.qw(), current.qx(), current.qy(), current.qz());
    Eigen::Vector3d vel_world(current.dx(), current.dy(), current.dz());

    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();

    // Transpose = inverse
    Eigen::Vector3d vel_body = rotation_matrix.transpose() * vel_world;

    // TODO: Might be different depending on how the IMU is oriented
    measurement.vel_forward() = vel_body.x();

    return measurement;
}

robo::ukf::ukf(double wheel_radius, double wheel_base_y) :
    predictor(100),
    wheel_radius(wheel_radius),
    wheel_base_y(wheel_base_y)
{
    model::state initial;
    initial.setZero();
    initial.qw() = 1.0;

    predictor.init(initial);

    // Initial covariance 
    Eigen::Matrix<double, model::state::RowsAtCompileTime, model::state::RowsAtCompileTime> P;
    P.setIdentity();

    P(model::state::_x, model::state::_x) = 0.1 * 0.1;
    P(model::state::_y, model::state::_y) = 0.1 * 0.1;
    P(model::state::_z, model::state::_z) = 0.1 * 0.1;

    P(model::state::_dx, model::state::_dx) = 0.5 * 0.5;
    P(model::state::_dy, model::state::_dy) = 0.5 * 0.5;
    P(model::state::_dz, model::state::_dz) = 0.5 * 0.5;

    P(model::state::_qw, model::state::_qw) = 0.1 * 0.1;
    P(model::state::_qx, model::state::_qx) = 0.1 * 0.1;
    P(model::state::_qy, model::state::_qy) = 0.1 * 0.1;
    P(model::state::_qz, model::state::_qz) = 0.1 * 0.1;

    predictor.setCovariance(P);

    // Process noise
    Eigen::Matrix<double, model::state::RowsAtCompileTime, model::state::RowsAtCompileTime> Q;
    Q.setZero();

    Q(model::state::_x, model::state::_x) = 1e-6;
    Q(model::state::_y, model::state::_y) = 1e-6;
    Q(model::state::_z, model::state::_z) = 1e-6;

    Q(model::state::_qw, model::state::_qw) = 1e-3;
    Q(model::state::_qx, model::state::_qx) = 1e-3;
    Q(model::state::_qy, model::state::_qy) = 1e-3;
    Q(model::state::_qz, model::state::_qz) = 1e-3;

    Q(model::state::_dx, model::state::_dx) = 1e-3;
    Q(model::state::_dy, model::state::_dy) = 1e-3;
    Q(model::state::_dz, model::state::_dz) = 1e-3;

    system.setCovariance(Q);

    // Measurement covariance
    Eigen::Matrix<double, model::encoder_measurement::RowsAtCompileTime, model::encoder_measurement::RowsAtCompileTime> R;
    R.setIdentity();
    R(model::encoder_measurement::_vel_forward, model::encoder_measurement::_vel_forward) = 1e-6;

    encoder_measurement_model.setCovariance(R);
}

void robo::ukf::predict(const imu_reading& imu, double delta_time)
{
    model::control control;

    // Set control from IMU readings
    control.ax() = imu.ax;
    control.ay() = imu.ay;
    control.az() = imu.az;
    control.gx() = imu.gx;
    control.gy() = imu.gy;
    control.gz() = imu.gz;
    control.dt() = delta_time;

    predictor.predict(system, control);
}

void robo::ukf::update_encoders(const encoder_reading& encoders)
{
    // Angular velocity -> linear velocity
    double vel_left = encoders.vel_left * wheel_radius;
    double vel_right = encoders.vel_right * wheel_radius;

    double vel_forward = (vel_left + vel_right) / 2.0f;

    model::encoder_measurement z;
    z.vel_forward() = vel_forward;

    predictor.update(encoder_measurement_model, z);
}

Eigen::Affine3d robo::ukf::get_transform() const
{
    const model::state& s = predictor.getState();

    Eigen::Quaterniond quaternion(s.qw(), s.qx(), s.qy(), s.qz());
    quaternion.normalize();  // Normalize before creating transform

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = quaternion.toRotationMatrix();
    transform.translation() = s.block(model::state::_x, 0, 3, 1);

    return transform;
}