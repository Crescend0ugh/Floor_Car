/*
	Collect IMU biases
*/

#include "MPU6050.h"

#include <iostream>
#include <chrono>

constexpr uint32_t sample_rate_hz = 100;
constexpr uint32_t sleep_time_ms = 1000 / sample_rate_hz;

int main(int argc, char** argv)
{
	std::cout << "=== IMU Bias Collection ===" << std::endl;
	std::cout << "Make sure IMU is stationary and on a stable surface!" << std::endl;
	std::cout << "\nStarting in 3 seconds...\n" << std::endl;

	std::this_thread::sleep_for(std::chrono::seconds(3));

	MPU6050 mpu(0x68, false);

	constexpr uint32_t sample_count = sample_rate_hz * 5;

	float biases[6] = { 0 };
	float totals[6] = { 0 };

	for (size_t i = 0; i < sample_count; ++i)
	{
		mpu.getOffsets(&biases[0], &biases[1], &biases[2], &biases[3], &biases[4], &biases[5]);

		for (size_t i = 0; i < 6; ++i)
		{
			totals[i] += biases[i];
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
	}

	// Compute average of cumulative bias readings
	for (size_t i = 0; i < 6; ++i)
	{
		totals[i] /= sample_count;
	}

	std::cout << "\n=== Collection Complete ===" << std::endl;
	std::cout << "Total samples collected: " << sample_count << std::endl;
	std::cout << "\nBiases (ax, ay, az, gx, gy, gz): " << std::endl;

	for (float average : totals)
	{
		std::cout << average << " ";
	}
	std::cout << "\n" << std::endl;

	std::cout << "Paste these values into the offset macros defined in third_party/mpu6050/MPU6050.h" << std::endl;

	return 0;
}