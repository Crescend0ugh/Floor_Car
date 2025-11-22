#include "arduino_serial.h"

#include <iostream>
#include <thread>

const unsigned char microphone_data_header = 0x02;
const unsigned char log_string_header = 0x03;
const unsigned char rc_command_header = 0xFF;

robo::arduino_serial::arduino_serial()
{
#ifdef __linux__
	std::string arduino_port_name = "/dev/ttyACM0"; // Will always be the case for the Pi
#else
	std::string arduino_port_name = "COM5"; // Windows USB (CHANGE TO THE ONE THE ARDUINO IDE SAYS IT'S USING)
#endif

	char opened = port.openDevice(arduino_port_name.c_str(), 115200);
	if (opened != 1)
	{
		std::cerr << "Unable to open Arduino serial port." << std::endl;
	}
	else
	{
		std::cout << "Opened Arduino serial port!" << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(2));
	}
}

void robo::arduino_serial::read()
{
	if (!port.isDeviceOpen())
	{
		return;
	}

	char buffer[32];
	while (available())
	{
		uint16_t receive_size = rx(buffer);

		// Null terminate just in case
		buffer[sizeof(buffer) - 1] = '\0';

		zpp::bits::in in(buffer);
		uint8_t header = 0;
		in(header).or_throw();

		switch (header)
		{
		case (microphone_data_header):
		{
			// TODO
			break;
		}
		case (log_string_header):
		{
			uint8_t string_length = 0;
			std::cout << "[Arduino Log]: " << std::string(buffer + sizeof(header)) << std::endl;
			break;
		}
		default:
		{
			std::cerr << "Received unrecognized message from Arduino serial: " << (int)header << std::endl;
			std::cerr << "Full: " << std::string(buffer) << std::endl;
		}
		}
	}
}

void robo::arduino_serial::close()
{
	port.closeDevice();
}

void robo::arduino_serial::send_rc_command(robo::network::rc_command command)
{
	std::string serialized;
	zpp::bits::out out(serialized);
	//out(rc_command_header).or_throw();
	out(command).or_throw();

	write(serialized);
}

void robo::arduino_serial::write(std::string data)
{
	if (!port.isDeviceOpen())
	{
		return;
	}

	char message[8];
	strcpy(message, data.c_str());
	send_datum(message);
}

uint8_t robo::arduino_serial::send_data(const uint16_t& len, const uint8_t packet_id)
{
	uint8_t num_bytes;

	num_bytes = packet.constructPacket(len, packet_id);
	port.writeBytes(packet.preamble, sizeof(packet.preamble));
	port.writeBytes(packet.txBuff, num_bytes);
	port.writeBytes(packet.postamble, sizeof(packet.postamble));

	return num_bytes;
}

uint8_t robo::arduino_serial::available()
{
	bool valid = false;
	uint8_t recChar = 0xFF;

	if (port.available())
	{
		valid = true;

		while (port.available())
		{
			port.readChar((char*)&recChar);

			bytes_read = packet.parse(recChar, valid);
			status = packet.status;

			if (status != CONTINUE)
			{
				if (status <= 0)
				{
					reset();
				}

				break;
			}
		}
	}
	else
	{
		bytes_read = packet.parse(recChar, valid);
		status = packet.status;

		if (status <= 0)
			reset();
	}

	return bytes_read;
}

void robo::arduino_serial::reset()
{
	char dummy = 0;
	while (port.available())
		port.readChar(&dummy);

	packet.reset();
	status = packet.status;
}