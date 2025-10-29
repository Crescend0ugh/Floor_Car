#pragma once

#include "asio.hpp"

struct imu_data
{
	unsigned long delta_time;

	float a_x;
	float a_y;
	float a_z;

	float g_x;
	float g_y;
	float g_z;
};

struct microphone_data
{

};

class arduino_serial
{
private:
	std::optional<asio::serial_port> port = std::nullopt;
	asio::streambuf buffer;

	std::optional<imu_data> latest_imu_data = std::nullopt;

	// Change this
	std::vector<microphone_data> microphone_data;

	void read_loop();

public:
	arduino_serial(asio::io_context& io);
	
	std::optional<imu_data> read_imu_data();
	void write(std::string& message);

	bool is_connected()
	{
		return port.has_value();
	};
};