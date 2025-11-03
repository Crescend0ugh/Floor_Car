/*
	arduino_serial.h

	Establishes and enables two-way communication with an Arduino Uno through a serial port.

	Host-tp-Arduino messages:
		- RC car commands

	Arduino-to-host messages:
		- IMU odometry data (subject to change)
		- Microphone inputs
*/

#pragma once

#include "zpp_bits.h"
#include "serialib.h"

struct imu_data
{
	float x;
	float y;
	float z;

	float roll;
	float pitch;
	float yaw; // This is the heading
};

struct microphone_data
{

};

class arduino_serial
{
private:
	serialib port;

	std::optional<imu_data> latest_imu_data = std::nullopt;

	// Change this
	std::vector<microphone_data> microphone_inputs;

public:
	arduino_serial();
	std::optional<imu_data> read_imu_data();

	void read();
	void write(std::string data, bool at_front = false);

	bool is_connected()
	{
		return port.isDeviceOpen();
	}
};