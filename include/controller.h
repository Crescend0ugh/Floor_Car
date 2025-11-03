/*
	controller.h

	Defines commands to be sent to the Arduino for robot movement.

	Opens and manages the Arduino serial (TODO: Would be better to detach the two)

	Records and updates IMU odometry data. The class is the source of truth for the position and rotation matrix of the car
	with respect to world coordinates.
*/

#pragma once

#include "vector.h"
#include "network_data.h"
#include "arduino_serial.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <queue>
#include <variant>
#include <chrono>
#include <optional>
#include <numbers>

namespace robo
{
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

		using command = std::variant<move_distance, move_for_seconds, rotate_to_heading, rotate_by, rotate_for_seconds>;
	}

	class controller
	{
		struct command_context
		{
			std::chrono::time_point<std::chrono::steady_clock> start_time;
			Eigen::Vector3f start_position = Eigen::Vector3f();
			float start_heading = 0;
		};

	private:
		std::optional<command::command> current_command = std::nullopt;
		command_context current_command_context;

		std::queue<command::command> command_queue;

		void send_command_to_arduino(command::command command_to_send);

		bool next_command();

	public:
		class arduino_serial arduino_serial;

		bool is_remote_controlled;

		Eigen::Vector3f position; // The car center position
		Eigen::Vector3f imu_position; // Position according to the IMU
		Eigen::Matrix3f imu_rotation = Eigen::Matrix3f::Identity();

		// The yaw: -179 to 180 degrees
		float heading;

		controller();

		void clear_command_queue();
		void update();
		void send_rc_command_to_arduino(robo::network::rc_command command);

		auto get_command_queue()
		{
			return command_queue;
		}

		auto get_current_command()
		{
			return current_command;
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
}
