#include "MPU6050.h"

#include <chrono>

constexpr uint8_t mpu_address = 0x68;
constexpr uint32_t poll_rate = 100; // 100 Hz

static void compute_biases(MPU6050& mpu)
{
	std::cout << "Computing biases" << std::endl;
	std::cout << "Make sure the IMU is as still as possible." << std::endl;

	constexpr uint32_t sample_count = poll_rate * 5;

	float biases[6] = { 0 };
	float totals[6] = { 0 };

	for (size_t i = 0; i < sample_count; ++i)
	{
		mpu.getOffsets(&biases[0], &biases[1], &biases[2], &biases[3], &biases[4], &biases[5]);

		for (size_t i = 0; i < 6; ++i)
		{
			totals[i] += biases[i];
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	// Compute average of cumulative bias readings
	for (size_t i = 0; i < 6; ++i)
	{
		totals[i] /= sample_count;
	}

	std::cout << "Biases (ax, ay, az, gx, gy, gz): " << std::endl;
	for (float average : totals) 
	{
		std::cout << average << " ";
	}
	std::cout << std::endl;
	std::cout << "Paste these values into the offset macros defined in third_party/mpu6050/MPU6050.h." << std::endl;
}

int main(int argc, char** argv)
{
	MPU6050 mpu(mpu_address);

	compute_biases(mpu);

	return 0;
}