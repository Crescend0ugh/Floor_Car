#pragma once

#ifdef __linux__

#include <ldlidar_driver.h>

static const ldlidar::LDType lidar_type = ldlidar::LDType::LD_19;

class lidar
{
private:
	ldlidar::LDLidarDriver node;

public:
	lidar();
	bool initialize_communication();
	void power_on();
	void power_off();
	void get_2d_points();
};

#endif