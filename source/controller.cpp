#include "controller.h"
#include "zpp_bits.h"

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

controller::controller() :
	drive_state(remote_control_drive_state::not_driving),
	steer_state(remote_control_steer_state::not_steering),
	is_remote_controlled(false),
	position(maid::vector3f()),
	heading(0.0f)
{
}

void controller::clear_command_queue()
{
	while (!command_queue.empty()) 
	{
		command_queue.pop();
	}
}

void controller::send_command_to_arduino(command::command command_to_send)
{
	std::string serialized;
	zpp::bits::out out(serialized);
	// Amazing! It handles std::variant perfectly!
	out(command_to_send).or_throw();

	arduino_serial.write(serialized);
}

void controller::update()
{
	auto current_time = std::chrono::steady_clock::now();

	arduino_serial.read();

	std::optional<imu_data> imu_option = arduino_serial.read_imu_data();
	if (imu_option.has_value())
	{
		imu_data& data = imu_option.value();
		position = maid::vector3f(data.x, data.y, data.z);
		// Something for orientation
	}

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
	if (command::move_for_seconds* command = std::get_if<command::move_for_seconds>(command_ptr))
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