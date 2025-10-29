#include "arduino_serial.h"
#include "zpp_bits.h"

#include <iostream>

const unsigned char imu_data_header = 0x01;
const unsigned char microphone_data_header = 0x02;

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

		port.value().set_option(asio::serial_port_base::baud_rate(9600));
		port.value().set_option(asio::serial_port_base::character_size(8));
		port.value().set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
		port.value().set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
		port.value().set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

		std::cout << "Opened Arduino serial port successfully!" << std::endl;

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
	asio::async_read(port.value(), buffer, asio::transfer_exactly(sizeof(size_t)),
		[&, this](asio::error_code error, size_t bytes)
		{
			if (!error)
			{
				// Read payload size
				size_t payload_size = 0;
				std::memcpy(&payload_size, buffer.data().data(), sizeof(payload_size));
				buffer.consume(buffer.size());

				// Read payload
				asio::read(port.value(), buffer, asio::transfer_exactly(payload_size));

				std::vector<unsigned char> data(buffer.size());
				asio::buffer_copy(asio::buffer(data), buffer.data());
				buffer.consume(buffer.size());

				zpp::bits::in in(data);

				// Consume header
				unsigned char header = 0;
				in(header).or_throw();

				switch (data[0])
				{
				case (imu_data_header): 
				{
					imu_data new_data;
					in(new_data).or_throw();

					if (!latest_imu_data.has_value())
					{
						latest_imu_data = std::move(new_data);
					}
					else // If we're slow on reads, merge the new readings with old readings
					{
						imu_data& previous_data = latest_imu_data.value();
						previous_data.delta_time += new_data.delta_time;

						// Take averages, completely override old data, or do something entirely different. Should be tested?
						previous_data.a_x = (previous_data.a_x + new_data.a_x) / 2;
						previous_data.a_y = (previous_data.a_y + new_data.a_y) / 2;
						previous_data.a_z = (previous_data.a_z + new_data.a_z) / 2;

						previous_data.g_x = (previous_data.g_x + new_data.g_x) / 2;
						previous_data.g_y = (previous_data.g_y + new_data.g_y) / 2;
						previous_data.g_z = (previous_data.g_z + new_data.g_z) / 2;
					}

					break;
				}
				case (microphone_data_header):
				{
					// TODO
					break;
				}
				default:
					std::cerr << "Received unrecognized message from Arduino serial" << std::endl;
				}

				read_loop();
			}
			else
			{
				std::cout << "Serial read error: " << error.message() << std::endl;
				return;
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

void arduino_serial::write(std::string& message)
{
	if (!port.has_value())
	{
		std::cerr << "Warning: Tried writing to unopened Arduino serial port" << std::endl;
		return;
	}

	// Terminate message with \n
	if (message.back() != '\n')
	{
		message.append("\n");
	}

	// Hopefully, this does not need to be asynchronous
	asio::write(port.value(), asio::buffer(message));
}