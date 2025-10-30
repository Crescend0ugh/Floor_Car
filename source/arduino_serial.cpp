#include "arduino_serial.h"

#include <iostream>

const unsigned char imu_data_header = 0x01;
const unsigned char microphone_data_header = 0x02;
const unsigned char log_string_header = 0x03;

arduino_serial::arduino_serial(asio::io_context& io)
{
#ifdef RPI_UBUNTU
	std::string arduino_port_name = "/dev/ttyUSB0"; // Will always be the case for the Pi
#else
	std::string arduino_port_name = "COM3"; // Windows USB (CHANGE TO THE ONE THE ARDUINO IDE SAYS IT'S USING)
#endif

	// Close the Arduino IDE serial monitor or this won't work!
	try
	{
		port = asio::serial_port(io, arduino_port_name);

		port.value().set_option(asio::serial_port_base::baud_rate(115200));
		port.value().set_option(asio::serial_port_base::character_size(8));
		port.value().set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
		port.value().set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
		port.value().set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

		std::cout << "Opened Arduino serial port successfully!" << std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		read_loop();
	}
	catch (const asio::system_error& error)
	{
		std::cerr << "An error occurred while opening Arduino serial port at "
			<< arduino_port_name << ": " << error.what() << std::endl;
	}
}

void arduino_serial::read_loop()
{
	string_buffer.clear();
	asio::dynamic_string_buffer buffer(string_buffer);
	
	asio::async_read(port.value(), buffer, asio::transfer_exactly(sizeof(short)),
		[&, this](asio::error_code error, size_t bytes_transferred)
		{
			if (!error)
			{
				// Read payload size
				unsigned short payload_size = 0;
				std::memcpy(&payload_size, string_buffer.data(), sizeof(payload_size));
				string_buffer.clear();

				// Read payload
				asio::dynamic_string_buffer payload_buffer(string_buffer);
				asio::async_read(port.value(), payload_buffer, asio::transfer_exactly(payload_size),
					[&, this](asio::error_code /*error*/, size_t /*bytes_transferred*/)
					{
						zpp::bits::in in(string_buffer);
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
							std::cout << "[Arduino Log]: " << string_buffer << std::endl;
							break;
						}
						default:
							std::cerr << "Received unrecognized message from Arduino serial " << header << std::endl;
							std::cerr << "Data: " << string_buffer << std::endl;
						}

						read_loop();
					}
				);
			}
			else
			{
				std::cout << "Serial read error: " << error.message() << std::endl;
				return;
			}
		}
	);
}

void arduino_serial::write_loop()
{
	asio::async_write(port.value(), asio::buffer(send_queue.front()),
		[this](asio::error_code error, size_t bytes_written)
		{
			if (!error && dequeue()) {
				write_loop();
			}
		}
	);
}

void arduino_serial::write(std::string data, bool at_front)
{
	if (!port.has_value())
	{
		return;
	}

	post(port.value().get_executor(),
		[=, this]
		{
			if (enqueue(std::move(data), at_front))
			{
				write_loop();
			}
		}
	);
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