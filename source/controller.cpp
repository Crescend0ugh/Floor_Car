#include "controller.h"

using namespace std::chrono_literals;

static constexpr auto seconds_to_chrono_nanoseconds(const float time_seconds)
{
	return std::chrono::round<std::chrono::nanoseconds>(std::chrono::duration<float>(time_seconds));
}

static float get_signed_angle(const float target_angle, const float start_angle)
{
	float angle = target_angle - start_angle;
	angle = fmod(angle + 180.0f, 360.0f) - 180.0f;

	return angle;
}

controller::controller(uint32_t update_rate_ms) :
	drive_state(remote_control_drive_state::not_driving),
	steer_state(remote_control_steer_state::not_steering),
	is_remote_controlled(false),
	update_rate(std::chrono::milliseconds(update_rate_ms)),
	next_update_time(std::chrono::seconds(0)),
	previous_update_time(std::chrono::seconds(0)),
	acceleration(maid::vector3f()),
	position(maid::vector3f()),
	velocity(maid::vector3f()),
	heading(0.0f),
	angular_velocity(maid::vector3f())
{
}

bool controller::connect_to_arduino(asio::io_context& io)
{

#ifdef RPI_UBUNTU
	std::string arduino_port_name = "/dev/ttyUSB0";
#else
	std::string arduino_port_name = "COM3"; // Windows USB (CHANGE TO THE ONE THE ARDUINO IDE SAYS IT'S USING)
#endif
	
	// Close the Arduino IDE serial monitor or this won't work!
	try
	{
		arduino_serial_port = asio::serial_port(io, arduino_port_name);

		asio::serial_port& port = arduino_serial_port.value();

		port.set_option(asio::serial_port_base::baud_rate(9600));
		port.set_option(asio::serial_port_base::character_size(8));
		port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
		port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
		port.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

		std::cout << "Opened Arduino serial port successfully!" << std::endl;
	}
	catch (const asio::system_error& error)
	{
		arduino_serial_port = std::nullopt;

		std::cerr << "An error occurred while opening Arduino serial port at " 
			<< arduino_port_name << ": " << error.what() << std::endl;
		
		return false;
	}

	return true;
}

// Send actual commands to the Arduino here
void controller::write_to_arduino(std::string& message)
{
	if (arduino_serial_port.has_value())
	{
		// Terminate message with \n
		if (message.back() != '\n')
		{
			message.append("\n");
		}

		asio::write(arduino_serial_port.value(), asio::buffer(message));
	}
}

void controller::clear_command_queue()
{
	while (!command_queue.empty()) 
	{
		command_queue.pop();
	}
}

void controller::update()
{
	auto current_time = std::chrono::steady_clock::now();

	if (update_rate > 0ms && current_time < next_update_time)
	{
		return;
	}

	auto delta_time = std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(current_time - previous_update_time).count() / 1000.0f;

	next_update_time = current_time + update_rate;
	previous_update_time = current_time;

	// Update acceleration and angular velocity here, possibly (they're public, so it's not necessary here)

	// Estimate position, velocity, orientation
	velocity += acceleration * delta_time;
	position += velocity * delta_time;

	// orientation += angular_velocity as quaternion * delta_time

	// Get next command
	if (!current_command.has_value() && !command_queue.empty())
	{
		next_command();
	}

	// If there's no command active, listen to remote controls
	if (is_remote_controlled && !current_command.has_value())
	{
		switch (drive_state)
		{
		case remote_control_drive_state::forward: {
			// PLACEHOLDER
			float velocity_magnitude = 1;
			auto direction = maid::vector3f(sin(heading * std::numbers::pi / 180.0f), 0, -cos(heading * std::numbers::pi / 180.0f));
			position += direction * velocity_magnitude;

			break;
		}

		case remote_control_drive_state::not_driving: {
			break;
		}
		}

		switch (steer_state)
		{
		case remote_control_steer_state::left: {
			// PLACEHOLDER
			float angular_velocity_magnitude = 0.5;
			heading -= angular_velocity_magnitude;

			break;
		}

		case remote_control_steer_state::right: {
			// PLACEHOLDER
			float angular_velocity_magnitude = 0.5;
			heading += angular_velocity_magnitude;

			break;
		}

		default: {
			break;
		}
		}

	}

	if (!current_command.has_value())
	{
		return;
	}

	auto command_ptr = &current_command.value();
	if (command::delay* command = std::get_if<command::delay>(command_ptr))
	{
		if (current_time < current_command_context.start_time + std::chrono::milliseconds(command->milliseconds))
		{
			return;
		}

		current_command.reset();
	}
	else if (command::move_for_seconds* command = std::get_if<command::move_for_seconds>(command_ptr))
	{
		// TODO
		
		// Done moving
		if (current_time >= current_command_context.start_time + seconds_to_chrono_nanoseconds(command->seconds))
		{
			next_command();
		}
	}
	else if (command::rotate_for_seconds* command = std::get_if<command::rotate_for_seconds>(command_ptr))
	{
		// TODO
		
		// Done rotating
		if (current_time >= current_command_context.start_time + seconds_to_chrono_nanoseconds(command->seconds))
		{
			next_command();
		}
	}
	else if (command::move_distance* command = std::get_if<command::move_distance>(command_ptr))
	{
		// TODO
		
		// PLACEHOLDER
		float velocity_magnitude = 0.5;
		auto direction = maid::vector3f(sin(heading * std::numbers::pi / 180.0f), 0, -cos(heading * std::numbers::pi / 180.0f));
		position += direction * velocity_magnitude;

		// TODO: Decide on an epsilon
		if ((position - current_command_context.start_position).length() > command->distance * 0.999)
		{
			next_command();
		}
	}
	else if (command::rotate_to_heading* command = std::get_if<command::rotate_to_heading>(command_ptr))
	{
		float signed_angle = get_signed_angle(command->heading, heading);
		// Rotate clockwise (negative) or counterclockwise (positive)?
		int sign = signed_angle < 0.0f ? -1 : 1;

		// TODO: Decide on an epsilon
		if (abs(signed_angle) < 0.005)
		{
			next_command();
		}
		else
		{
			// PLACEHOLDER
			float angular_velocity_magnitude = 0.5;
			heading += sign * angular_velocity_magnitude;
		}
	}
	else if (command::rotate_by* command = std::get_if<command::rotate_by>(command_ptr))
	{
		// TODO
		int sign = command->delta_heading < 0.0f ? -1 : 1;

		// PLACEHOLDER
		float angular_velocity_magnitude = 0.5;

		// progress = sign * angular velocity * delta time
		float progress = sign * angular_velocity_magnitude;
		heading += progress;

		command->progress += progress;

		// TODO: Decide on an epsilon
		if (abs(command->progress - command->delta_heading) < 0.05)
		{
			next_command();
		}
	}
}