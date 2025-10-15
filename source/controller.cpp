#include "controller.h"

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

controller::controller():
	update_rate(std::chrono::milliseconds(10)),
	next_update_time(std::chrono::seconds(0)),
	acceleration(maid::vector3f()),
	position(maid::vector3f()),
	velocity(maid::vector3f()),
	heading(0.0f),
	angular_velocity(maid::vector3f())
{

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
	auto current_time = std::chrono::high_resolution_clock::now();

	if (current_time < next_update_time)
	{
		return;
	}

	auto delta_time = std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(current_time - next_update_time).count() / 1000.0f;
	next_update_time = current_time + update_rate;

	// Update acceleration and angular velocity here, possibly (they're public, so)

	// Estimate position, velocity, orientation
	velocity += acceleration * delta_time;
	position += velocity * delta_time;

	// orientation += angular_velocity as quaternion * delta_time

	// Get next command
	if (!current_command.has_value())
	{
		if (command_queue.empty())
		{
			return;
		}

		current_command = command_queue.front();
		command_queue.pop();

		current_command_context.start_time = current_time;
		current_command_context.start_position = position;
		current_command_context.start_heading = heading;
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
		// Done moving
		if (current_time >= current_command_context.start_time + seconds_to_chrono_nanoseconds(command->seconds))
		{
			current_command.reset();
			return;
		}

		// TODO
	}
	else if (command::rotate_for_seconds* command = std::get_if<command::rotate_for_seconds>(command_ptr))
	{
		// Done rotating
		if (current_time >= current_command_context.start_time + seconds_to_chrono_nanoseconds(command->seconds))
		{
			current_command.reset();
			return;
		}

		// TODO
	}
	else if (command::move_distance* command = std::get_if<command::move_distance>(command_ptr))
	{
		// TODO: Decide on an epsilon
		if ((position - current_command_context.start_position).length() > command->distance * 0.99)
		{
			current_command.reset();
			return;
		}

		// TODO
		
		// PLACEHOLDER
		float velocity_magnitude = 1;
		auto direction = maid::vector3f(sin(heading * std::numbers::pi / 180.0f), 0, -cos(heading * std::numbers::pi / 180.0f));
		position += direction * velocity_magnitude;
	}
	else if (command::rotate_to_heading* command = std::get_if<command::rotate_to_heading>(command_ptr))
	{
		float signed_angle = get_signed_angle(command->heading, heading);

		// TODO: Decide on an epsilon
		if (abs(signed_angle) < 0.005)
		{
			current_command.reset();
			return;
		}

		// Rotate clockwise (negative) or counterclockwise (positive)?
		int sign = signed_angle < 0.0f ? -1 : 1;

		// TODO
	}
	else if (command::rotate_by* command = std::get_if<command::rotate_by>(command_ptr))
	{
		float signed_angle = get_signed_angle(heading, current_command_context.start_heading);

		// TODO: Decide on an epsilon
		if (abs(signed_angle) >= 0.99 * abs(command->delta_heading))
		{
			current_command.reset();
			return;
		}

		// TODO
		
		// PLACEHOLDER
		float angular_velocity_magnitude = 0.1;
		heading += (command->delta_heading < 0 ? -1 : 1) * angular_velocity_magnitude;
	}
}