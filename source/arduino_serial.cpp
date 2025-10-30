#include "arduino_serial.h"

#include <iostream>

const unsigned char imu_data_header = 0x01;
const unsigned char microphone_data_header = 0x02;
const unsigned char log_string_header = 0x03;

arduino_serial::arduino_serial()
{
#ifdef RPI_UBUNTU
	std::string arduino_port_name = "/dev/ttyUSB0"; // Will always be the case for the Pi
#else
	std::string arduino_port_name = "COM3"; // Windows USB (CHANGE TO THE ONE THE ARDUINO IDE SAYS IT'S USING)
#endif

	char opened = port.openDevice(arduino_port_name.c_str(), 115200);
	if (opened != 1)
	{
		std::cerr << "Unable to open Arduino serial port." << std::endl;
	}
	else
	{
		std::cout << "Opened Arduino serial port!" << std::endl;
	}
}

void arduino_serial::read()
{
	if (!port.isDeviceOpen())
	{
		return;
	}

	while (port.available() > 0)
	{
		unsigned short payload_size;
		port.readBytes(&payload_size, sizeof(payload_size));

		// Some garbage data, probably
		if (payload_size > 256)
		{
			port.flushReceiver();
			break;
		}

		char buffer[256];
		port.readBytes(buffer, payload_size);

		zpp::bits::in in(buffer);
		unsigned char header = 0;
		in(header).or_throw();

		switch (header)
		{
		case (imu_data_header):
		{
			imu_data new_data;
			in(new_data).or_throw();
			latest_imu_data = std::move(new_data);

			break;
		}
		case (microphone_data_header):
		{
			// TODO
			break;
		}
		case (log_string_header):
		{
			std::cout << "[Arduino Log]: " << std::string(buffer + sizeof(header)) << std::endl;
			break;
		}
		default:
		{
			std::cerr << "Received unrecognized message from Arduino serial " << header << std::endl;
		}
		}
	}
}

void arduino_serial::write(std::string data, bool at_front)
{
	if (!port.isDeviceOpen())
	{
		return;
	}

	port.writeString(data.c_str());
}

std::optional<imu_data> arduino_serial::read_imu_data()
{
	if (!latest_imu_data.has_value())
	{
		return std::nullopt;
	}

	// Take the data out of the optional
	imu_data data = std::move(latest_imu_data.value());
	latest_imu_data = std::nullopt;
	return data;
}