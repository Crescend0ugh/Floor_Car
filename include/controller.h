#pragma once

#include <queue>
#include <variant>
#include <chrono>
#include <optional>
#include <numbers>

#include "vector.h"

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

class controller
{
	struct command_context
	{
		std::chrono::time_point<std::chrono::steady_clock> start_time;
		maid::vector3f start_position;
		float start_heading;
	};

private:
	std::chrono::milliseconds update_rate;
	std::chrono::time_point<std::chrono::steady_clock> next_update_time;

	std::optional<command::command> current_command = std::nullopt;
	command_context current_command_context;

	std::queue<command::command> command_queue;

	// front left motor, front right motor, back left motor, back right motor

public:
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
	// This is in degrees?
	// -179 to 180
	float heading;

	controller();

	size_t get_command_queue_size()
	{
		return command_queue.size();
	}

	void move_forward_distance(float distance)
	{
		command_queue.push(command::move_distance{ distance });
	}

	void move_forward_for(float seconds)
	{
		command_queue.push(command::move_for_seconds{ seconds });
	}

	void rotate_to(float heading)
	{
		command_queue.push(command::rotate_to_heading{ heading });
	}

	void rotate_by(float delta_heading)
	{
		// Normalize
		//float goal_heading = heading + delta_heading;
		//goal_heading = fmod(goal_heading, 360.0f);

		//goal_heading = fmod(goal_heading + 360.0f, 360.0f);

		//if (goal_heading > 180.0f)
		//{
		//	goal_heading -= 360.0f;
		//}

		command_queue.push(command::rotate_by{ delta_heading });
	}

	void rotate_for(float seconds)
	{
		command_queue.push(command::rotate_for_seconds{ seconds });
	}

	void clear_command_queue();
	void update();
};