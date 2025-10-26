#pragma once

#include <queue>
#include <variant>
#include <chrono>
#include <optional>
#include <numbers>

#include "vector.h"
#include "asio.hpp"

namespace command
{
	struct move_distance
	{
		float distance;
	};

	struct move_for_seconds
	{
		float seconds;
	};

	struct rotate_to_heading
	{
		float heading;
	};

	struct rotate_by
	{
		float delta_heading;
		float progress;
	};

	struct rotate_for_seconds
	{
		float seconds;
	};

	struct delay
	{
		unsigned int milliseconds;
	};

	using command = std::variant<move_distance, move_for_seconds, rotate_to_heading, rotate_by, rotate_for_seconds, delay>;
}

enum remote_control_drive_state
{
	not_driving = 0,
	forward,
};

enum remote_control_steer_state
{
	not_steering = 0,
	left,
	right,
};

class controller
{
	struct command_context
	{
		std::chrono::time_point<std::chrono::steady_clock> start_time;
		maid::vector3f start_position = maid::vector3f();
		float start_heading = 0;
	};

private:
	// An update rate of 0 ms means the controller will not try to rate-limit itself
	std::chrono::milliseconds update_rate;
	std::chrono::time_point<std::chrono::steady_clock> next_update_time;
	std::chrono::time_point<std::chrono::steady_clock> previous_update_time;

	std::optional<command::command> current_command = std::nullopt;
	command_context current_command_context;

	std::queue<command::command> command_queue;
	
	std::optional<asio::serial_port> arduino_serial_port = std::nullopt;

	void write_to_arduino(std::string& message);

	// front left motor, front right motor, back left motor, back right motor

public:
	remote_control_drive_state drive_state;
	remote_control_steer_state steer_state;
	bool is_remote_controlled;

	// Read from the gyroscope's accelerometer
	maid::vector3f acceleration;

	// Estimated from acceleration using Euler's method
	maid::vector3f position;
	maid::vector3f velocity;

	// Read from the gyroscope's gyro (Might want to convert this to a quaternion)
	maid::vector3f angular_velocity;

	// Would be estimated using Euler's method
	// quaternion orientation;

	// Convert orientation to yaw-pitch-roll Euler angles and heading should be yaw
	// -179 to 180 degrees
	float heading;

	controller(uint32_t update_rate_ms = 0);

	bool connect_to_arduino(asio::io_context& io);

	void clear_command_queue();
	void update();

	// Motor setting stuff

	auto get_command_queue()
	{
		return command_queue;
	}

	auto get_current_command()
	{
		return current_command;
	}

	bool next_command()
	{
		current_command.reset();

		if (command_queue.empty())
		{
			return false;
		}

		current_command = command_queue.front();
		command_queue.pop();

		current_command_context.start_time = std::chrono::steady_clock::now();
		current_command_context.start_position = position;
		current_command_context.start_heading = heading;

		return true;
	}

	void normalize_heading()
	{
		heading = fmod(heading, 360.0f);

		heading = fmod(heading + 360.0f, 360.0f);

		if (heading > 180.0f)
		{
			heading -= 360.0f;
		}
	}

	void move_forward_distance(float distance)
	{
		if (current_command.has_value())
		{
			auto command_ptr = &current_command.value();

			// Add on to existing command
			if (command::move_distance* command = std::get_if<command::move_distance>(command_ptr))
			{
				command->distance += distance;
				return;
			}
		}

		command_queue.push(command::move_distance{ distance });
	}

	void move_forward_for(float seconds)
	{
		command_queue.push(command::move_for_seconds{ seconds });
	}

	void rotate_to(float heading)
	{
		if (current_command.has_value())
		{
			auto command_ptr = &current_command.value();

			// Add on to existing command
			if (command::rotate_to_heading* command = std::get_if<command::rotate_to_heading>(command_ptr))
			{
				// Redundant command
				if (heading == command->heading)
				{
					return;
				}
			}
		}

		command_queue.push(command::rotate_to_heading{ heading });
	}

	void rotate_by(float delta_heading)
	{
		if (current_command.has_value())
		{
			auto command_ptr = &current_command.value();

			// Add on to existing command
			if (command::rotate_by* command = std::get_if<command::rotate_by>(command_ptr))
			{
				// Normalize goal angle
				float goal_heading = command->delta_heading + delta_heading;
				goal_heading = fmod(goal_heading, 360.0f);

				goal_heading = fmod(goal_heading + 360.0f, 360.0f);

				if (goal_heading > 180.0f)
				{
					goal_heading -= 360.0f;
				}

				command->delta_heading = goal_heading;
				return;
			}
		}

		command_queue.push(command::rotate_by{ delta_heading, 0.0f });
	}

	void rotate_for(float seconds)
	{
		command_queue.push(command::rotate_for_seconds{ seconds });
	}
};