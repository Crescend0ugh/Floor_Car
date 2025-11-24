#pragma once

#include <Arduino.h>

#include <string.h>

#include "arduino_shared_defs.h"

struct write_buffer
{
	unsigned char buffer[arduino::host_recv_buffer_size];
	size_t offset = 0;

	write_buffer(unsigned char payload_type)
	{
		write(&payload_type, sizeof(unsigned char));
	}

	void write(void* data, size_t data_size)
	{
		if (offset + data_size > arduino::host_recv_buffer_size)
		{
			data_size = arduino::host_recv_buffer_size - offset; // Truncate to prevent overflow
		}

		memcpy(buffer + offset, data, data_size);
		offset += data_size;
	}
};