/*
	arduino_serial.h

	Establishes and enables two-way communication with an Arduino Uno through a serial port.

	Host-tp-Arduino messages:
		- RC car commands

	Arduino-to-host messages:
		- IMU odometry data (subject to change)
*/

#pragma once

#include "zpp_bits.h"
#include "network_data.h"
#include "serialib.h"
#include "Packet.h"

namespace robo
{
	
	class arduino_serial
	{
	private:
		Packet packet;
		int8_t status = -1;
		uint8_t bytes_read = 0;

		serialib port;

		uint8_t send_data(const uint16_t& len, const uint8_t packet_id = 0);
		uint8_t available();
		void reset();
		void write(std::string data);

		template <typename T>
		uint16_t tx(const T& val, const uint16_t& index = 0, const uint16_t& len = sizeof(T))
		{
			return packet.txObj(val, index, len);
		}

		template <typename T>
		uint16_t rx(const T& val, const uint16_t& index = 0, const uint16_t& len = sizeof(T))
		{
			return packet.rxObj(val, index, len);
		}

		template <typename T>
		uint8_t send_datum(const T& val, const uint16_t& len = sizeof(T))
		{
			return send_data(packet.txObj(val, 0, len));
		}

	public:
		arduino_serial();

		void read();
		void close();

		bool is_connected()
		{
			return port.isDeviceOpen();
		}

		void send_rc_command(robo::network::rc_command command);
	};
}