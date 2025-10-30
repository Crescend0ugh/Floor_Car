#pragma once

#include "asio.hpp"
#include "zpp_bits.h"

#include <list>

struct imu_data
{
	float x;
	float y;
	float z;

	float yaw;
	float pitch;
	float roll;
};

struct microphone_data
{

};

class arduino_serial
{
private:
	std::optional<asio::serial_port> port = std::nullopt;
	std::string string_buffer;
	std::list<std::string> send_queue;

	std::optional<imu_data> latest_imu_data = std::nullopt;

	// Change this
	std::vector<microphone_data> microphone_inputs;

	void read_loop();
	void write_loop();

	// Copy-paste jobs from network.h 
	bool enqueue(std::string data, bool at_front)
	{
		at_front &= !send_queue.empty();
		if (at_front)
			send_queue.insert(std::next(std::begin(send_queue)), std::move(data));
		else
			send_queue.push_back(std::move(data));

		return send_queue.size() == 1;
	}

	bool dequeue()
	{
		assert(!send_queue.empty());
		send_queue.pop_front();
		return !send_queue.empty();
	}

public:
	arduino_serial(asio::io_context& io);
	
	std::optional<imu_data> read_imu_data();

	void write(std::string data, bool at_front = false);

	bool is_connected()
	{
		return port.has_value();
	};
};