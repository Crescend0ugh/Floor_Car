#pragma once

#include <stdint.h>

namespace arduino
{
	constexpr uint32_t baud_rate = 115200;

	constexpr uint8_t log_string_header = 0x03;

	// Unused
	constexpr uint8_t rc_command_header = 0xFF;

	// The maximum buffer size is 64 - 8 = 56 bytes
	// The Arduino serial send/receive buffer is 64 bytes, and SerialTransfer uses 8 bytes to packetize data

	// The size of the buffer sent to the Arduino from the host, in bytes
	constexpr uint8_t host_send_buffer_size = 8;

	// The size of the buffer sent to the host from the Arduino, in bytes
	constexpr uint8_t host_recv_buffer_size = 32;
}
