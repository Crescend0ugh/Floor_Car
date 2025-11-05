#include "controller.h"
#include "zpp_bits.h"

using namespace std::chrono_literals;

static const unsigned char rc_command_header = 0xFF;

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

// For the IMU coordinate system, which is right-handed
static Eigen::Matrix3f euler_angles_to_rotation_matrix(float roll, float pitch, float yaw)
{
	Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());

	// Combine in Z-Y-X order
	Eigen::Quaternionf q = yaw_angle * pitch_angle * roll_angle;

	Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();

	return rotation_matrix;
}

robo::controller::controller() :
	is_remote_controlled(false),
	imu_position(Eigen::Vector3f()),
	heading(0.0f)
{
}

void robo::controller::clear_command_queue()
{
	while (!command_queue.empty()) 
	{
		command_queue.pop();
	}
}

void robo::controller::send_command_to_arduino(robo::command::command command_to_send)
{
	std::string serialized;
	zpp::bits::out out(serialized);
	// Amazing! It handles std::variant perfectly!
	out(command_to_send).or_throw();
	
	arduino_serial.write(serialized);
}

void robo::controller::send_rc_command_to_arduino(robo::network::rc_command command)
{
	std::string serialized;
	zpp::bits::out out(serialized);
	out(rc_command_header).or_throw();
	out(command).or_throw();

	arduino_serial.write(serialized);
}

bool robo::controller::next_command()
{
	current_command.reset();

	if (command_queue.empty())
	{
		return false;
	}

	current_command = command_queue.front();
	command_queue.pop();

	current_command_context.start_time = std::chrono::steady_clock::now();
	current_command_context.start_position = imu_position;
	current_command_context.start_heading = heading;

	send_command_to_arduino(current_command.value());

	return true;
}

void robo::controller::update()
{
	auto current_time = std::chrono::steady_clock::now();

	arduino_serial.read();

	std::optional<imu_data> imu_option = arduino_serial.read_imu_data();
	if (imu_option.has_value())
	{
		imu_data& data = imu_option.value();
		imu_position = Eigen::Vector3f(data.x, data.y, data.z);
		imu_rotation = euler_angles_to_rotation_matrix(data.roll, data.pitch, data.yaw);
		heading = data.yaw;
	}

	// Get next command
	if (!current_command.has_value() && !command_queue.empty())
	{
		next_command();
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


		// TODO: Decide on an epsilon
		if ((imu_position - current_command_context.start_position).norm() > command->distance * 0.999)
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