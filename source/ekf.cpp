#include "ekf.h"

void model::state::normalize_quaternion()
{
    Eigen::Vector4f q(qw(), qx(), qy(), qz());
    q.normalize();
    qw() = q(0);
    qx() = q(1);
    qy() = q(2);
    qz() = q(3);
}

// The non-linear state transition function
model::state model::system_model<>::f(const model::state& current, const model::control& control) const
{
    state predicted;

    Eigen::Vector3f accel_corrected(
        control.ax() - current.bias_ax(),
        control.ay() - current.bias_ay(),
        control.az() - current.bias_az()
    );

    Eigen::Vector3f gyro_corrected(
        control.gx() - current.bias_gx(),
        control.gy() - current.bias_gy(),
        control.gz() - current.bias_gz()
    );

    predicted.wx() = gyro_corrected.x();
    predicted.wy() = gyro_corrected.y();
    predicted.wz() = gyro_corrected.z();

    // Compute new quaternion given angular velocity
    float angle = gyro_corrected.norm();
    Eigen::Quaternionf q_current(current.qw(), current.qx(), current.qy(), current.qz());

    if (angle > 1e-6)
    {
        Eigen::Vector3f axis = gyro_corrected / angle;  // Rotation axis

        // q = [cos(angle/2), sin(angle/2) * axis]
        Eigen::Quaternionf dq;
        dq.w() = std::cos(angle / 2.0f);
        dq.vec() = std::sin(angle / 2.0f) * axis;

        q_current *= dq;
    }

    q_current.normalize();

    // Update velocity from accelerometer 
    // First, the acceleration must be rotated to the world frame
    Eigen::Matrix3f rotation_matrix = q_current.toRotationMatrix();
    Eigen::Vector3f accel_world = rotation_matrix * accel_corrected;

    Eigen::Vector3f gravity(0.0f, 0.0f, -9.81f);
    accel_world -= gravity;  // Subtract gravity to obtain true acceleration in world frame

    // Update velocity
    predicted.dx() = current.dx() + accel_world.x();
    predicted.dy() = current.dy() + accel_world.y();
    predicted.dz() = current.dz() + accel_world.z();

    // Trapezoidal integration
    predicted.x() = current.x() + (current.dx() + predicted.dx()) / 2.0f;
    predicted.y() = current.y() + (current.dy() + predicted.dy()) / 2.0f;
    predicted.z() = current.z() + (current.dz() + predicted.dz()) / 2.0f;

    predicted.qw() = q_current.w();
    predicted.qx() = q_current.x();
    predicted.qy() = q_current.y();
    predicted.qz() = q_current.z();

    // Heuristic: Biases remain constant
    predicted.bias_ax() = current.bias_ax();
    predicted.bias_ay() = current.bias_ay();
    predicted.bias_az() = current.bias_az();
    predicted.bias_gx() = current.bias_gx();
    predicted.bias_gy() = current.bias_gy();
    predicted.bias_gz() = current.bias_gz();

    return predicted;
}

model::encoder_measurement model::encoder_measurement_model<>::h(const model::state& current) const
{
    encoder_measurement measurement;

    Eigen::Quaternionf q(current.qw(), current.qx(), current.qy(), current.qz());
    Eigen::Vector3f vel_world(current.dx(), current.dy(), current.dz());

    Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();

    // Transpose = inverse
    Eigen::Vector3f vel_body = rotation_matrix.transpose() * vel_world;

    // TODO: Might be different depending on how the IMU is oriented
    measurement.vel_forward() = vel_body.x();

    // Encoder yaw rate is the same as the state
    measurement.wz() = current.wz();

    return measurement;
}

robo::ukf::ukf(float wheel_radius, float wheel_base_y):
	predictor(0.001f, 2.0f, 0.0f),
    wheel_radius(wheel_radius),
    wheel_base_y(wheel_base_y)
{
	state.setZero();
    state.qw() = 1;

	predictor.init(state);

    // TOOD: Set covariances
    auto& Q = system.getCovariance();

    // Accelerometer bias process noise
   /* Q(model::state::_bias_ax, model::state::_bias_ax) = 0.001 * 0.001;
    Q(model::state::_bias_ax, model::state::_bias_ax) = 0.001 * 0.001;
    Q(State::BIAS_AZ, State::BIAS_AZ) = 0.001 * 0.001;*/

    // Gyroscope bias process noise
    //Q(State::BIAS_GX, State::BIAS_GX) = 0.0001 * 0.0001;
    //Q(State::BIAS_GY, State::BIAS_GY) = 0.0001 * 0.0001;
    //Q(State::BIAS_GZ, State::BIAS_GZ) = 0.0001 * 0.0001;

    auto& R = encoder_measurement_model.getCovariance();
}

void robo::ukf::predict(const imu_reading& imu)
{
	// Set control from IMU readings
	control.ax() = imu.ax;
	control.ay() = imu.ay;
	control.az() = imu.az;
	control.gx() = imu.gx;
	control.gy() = imu.gy;
	control.gz() = imu.gz;

	// Propagate state
	predictor.predict(system, control);

    // Normalize
    state.normalize_quaternion();
}

void robo::ukf::update_encoders(const encoder_reading& encoders)
{
    // Angular velocity -> linear velocity
    float vel_left = encoders.vel_left * wheel_radius;
    float vel_right = encoders.vel_right * wheel_radius;

    float vel_forward = (vel_left + vel_right) / 2.0f;
    float wz = (vel_right - vel_left) / wheel_base_y;

    model::encoder_measurement z;
    z.vel_forward() = vel_forward;
    z.wz() = wz;

    predictor.update(encoder_measurement_model, z);

    // Normalize quaternion after update
    state.normalize_quaternion();
}

Eigen::Affine3f robo::ukf::get_transform() const
{
    const model::state& s = predictor.getState();

    Eigen::Quaternionf quaternion(s.qw(), s.qx(), s.qy(), s.qz());

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    transform.linear() = quaternion.toRotationMatrix();
    transform.translation() = Eigen::Vector3f(s.x(), s.y(), s.z());
    
    return transform;
}