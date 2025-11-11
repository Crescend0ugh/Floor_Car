#include "ekf.h"

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

    // Normalize input quaternion to prevent numerical issues
    Eigen::Quaterniond q_current(current.qw(), current.qx(), current.qy(), current.qz());

    Eigen::Vector3d accel_corrected(
        control.ax(),// - current.bias_ax(),
        control.ay(),// - current.bias_ay(),
        control.az()// - current.bias_az()
    );

    Eigen::Vector3d gyro_corrected(
        control.gx(),// - current.bias_gx(),
        control.gy(),// - current.bias_gy(),
        control.gz()// - current.bias_gz()
    );

    predicted.wx() = gyro_corrected.x();
    predicted.wy() = gyro_corrected.y();
    predicted.wz() = gyro_corrected.z();

    // Compute new quaternion given angular velocity
    double angle = gyro_corrected.norm();

    if (angle > 1e-4f)
    {
        Eigen::Vector3d axis = gyro_corrected / angle;

        Eigen::Quaterniond dq;
        dq.w() = std::cos(angle / 2.0f);
        dq.vec() = std::sin(angle / 2.0f) * axis;

        q_current = q_current * dq;
    }

    q_current.normalize();

    // Update velocity from accelerometer 
    Eigen::Matrix3d rotation_matrix = q_current.toRotationMatrix();

    Eigen::Vector3d accel_world = rotation_matrix * accel_corrected;
    Eigen::Vector3d gravity(0.0f, 0.0f, -9.81f);
    accel_world -= gravity;

    Eigen::Vector3d vel_new(current.dx(), current.dy(), current.dz());
    vel_new += accel_world;

    Eigen::Vector3d vel_old(current.dx(), current.dy(), current.dz());
    Eigen::Vector3d v_avg = (vel_old + vel_new) / 2.0f;

    Eigen::Vector3d pos_new(current.x(), current.y(), current.z());
    pos_new += v_avg;

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

    predicted.bias_ax() = current.bias_ax();
    predicted.bias_ay() = current.bias_ay();
    predicted.bias_az() = current.bias_az();
    predicted.bias_gx() = current.bias_gx();
    predicted.bias_gy() = current.bias_gy();
    predicted.bias_gz() = current.bias_gz();

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

    // Encoder yaw rate is the same as the state
    measurement.wz() = current.wz();

    return measurement;
}

robo::ukf::ukf(double wheel_radius, double wheel_base_y) :
    predictor(0.001f, 2.0f),
    wheel_radius(wheel_radius),
    wheel_base_y(wheel_base_y)
{
    model::state initial;
    initial.setZero();
    initial.qw() = 1.0f;

    predictor.init(initial);

    // Initial covariance 
    Eigen::Matrix<double, model::state::RowsAtCompileTime, model::state::RowsAtCompileTime> P;
    P.setIdentity();
    P *= 0.1f;

    P(model::state::_x, model::state::_x) = 0.1f;
    P(model::state::_y, model::state::_y) = 0.1f;
    P(model::state::_z, model::state::_z) = 0.1f;
    P(model::state::_qw, model::state::_qw) = 0.01f;
    P(model::state::_qx, model::state::_qx) = 0.01f;
    P(model::state::_qy, model::state::_qy) = 0.01f;
    P(model::state::_qz, model::state::_qz) = 0.01f;

    predictor.setCovariance(P);

    // Process noise
    Eigen::Matrix<double, model::state::RowsAtCompileTime, model::state::RowsAtCompileTime> Q;
    Q.setZero();

    Q(model::state::_x, model::state::_x) = 0.01f;
    Q(model::state::_y, model::state::_y) = 0.01f;
    Q(model::state::_z, model::state::_z) = 0.01f;
    Q(model::state::_qw, model::state::_qw) = 0.001f;
    Q(model::state::_qx, model::state::_qx) = 0.001f;
    Q(model::state::_qy, model::state::_qy) = 0.001f;
    Q(model::state::_qz, model::state::_qz) = 0.001f;

    Q(model::state::_dx, model::state::_dx) = 0.1f;
    Q(model::state::_dy, model::state::_dy) = 0.1f;
    Q(model::state::_dz, model::state::_dz) = 0.1f;
    Q(model::state::_wx, model::state::_wx) = 0.01f;
    Q(model::state::_wy, model::state::_wy) = 0.01f;
    Q(model::state::_wz, model::state::_wz) = 0.01f;

    Q(model::state::_bias_ax, model::state::_bias_ax) = 0.001f;
    Q(model::state::_bias_ay, model::state::_bias_ay) = 0.001f;
    Q(model::state::_bias_az, model::state::_bias_az) = 0.001f;
    Q(model::state::_bias_gx, model::state::_bias_gx) = 0.0001f;
    Q(model::state::_bias_gy, model::state::_bias_gy) = 0.0001f;
    Q(model::state::_bias_gz, model::state::_bias_gz) = 0.0001f;

    system.setCovariance(Q);

    // Measurement covariance
    Eigen::Matrix<double, model::encoder_measurement::RowsAtCompileTime, model::encoder_measurement::RowsAtCompileTime> R;
    R.setIdentity();
    R(model::encoder_measurement::_vel_forward, model::encoder_measurement::_vel_forward) = 0.05f * 0.05f;
    R(model::encoder_measurement::_wz, model::encoder_measurement::_wz) = 0.01f * 0.01f;
    encoder_measurement_model.setCovariance(R);
}

void robo::ukf::predict(const imu_reading& imu)
{
    model::control control;

    // Set control from IMU readings
    control.ax() = imu.ax;
    control.ay() = imu.ay;
    control.az() = imu.az;
    control.gx() = imu.gx;
    control.gy() = imu.gy;
    control.gz() = imu.gz;

    // Check eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 19, 19>> es(predictor.getCovariance());
    std::cout << "Eigenvalues: " << es.eigenvalues().transpose() << std::endl;
    std::cout << "Min eigenvalue: " << es.eigenvalues().minCoeff() << std::endl;

    predictor.predict(system, control);
}

void robo::ukf::update_encoders(const encoder_reading& encoders)
{
    // Angular velocity -> linear velocity
    double vel_left = encoders.vel_left * wheel_radius;
    double vel_right = encoders.vel_right * wheel_radius;

    double vel_forward = (vel_left + vel_right) / 2.0f;
    double wz = (vel_right - vel_left) / wheel_base_y;

    model::encoder_measurement z;
    z.vel_forward() = vel_forward;
    z.wz() = wz;

    predictor.update(encoder_measurement_model, z);
}

Eigen::Affine3d robo::ukf::get_transform() const
{
    const model::state& s = predictor.getState();

    Eigen::Quaterniond quaternion(s.qw(), s.qx(), s.qy(), s.qz());
    quaternion.normalize();  // Normalize before creating transform

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = quaternion.toRotationMatrix();
    transform.translation() = Eigen::Vector3d(s.x(), s.y(), s.z());

    return transform;
}