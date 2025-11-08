#include "ekf.h"

robo::ekf::ekf()
{
	state.setZero();
	predictor.init(state);
}

Eigen::Affine3f robo::ekf::update(const imu_reading& imu, const encoder_reading& encoder)
{
	// Set control from IMU readings (already should have accounted for gravity at this point)
	control.ax() = imu.ax;
	control.ay() = imu.ay;
	control.az() = imu.az;
	control.gx() = imu.gx;
	control.gy() = imu.gy;
	control.gz() = imu.gz;

	// Simulate
	state = system.f(state, control);
	auto predicted_state = predictor.predict(system, control);

	model::encoder_measurement encoder_measurement = encoder_measurement_model.h(state);

	predicted_state = predictor.update(encoder_measurement_model, encoder_measurement);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(Eigen::AngleAxisf(predicted_state.roll(), Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(predicted_state.pitch(), Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(predicted_state.yaw(), Eigen::Vector3f::UnitZ()));

	transform.translation() << predicted_state.x(), predicted_state.y(), predicted_state.z();

	return transform;
}