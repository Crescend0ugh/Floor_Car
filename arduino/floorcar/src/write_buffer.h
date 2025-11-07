#pragma once

#include "Arduino.h"

#include <string.h>

#define max_capacity 32

struct write_buffer
{
	unsigned char buffer[max_capacity];
	size_t offset = 0;

	write_buffer(unsigned char payload_type)
	{
		write(&payload_type, sizeof(unsigned char));
	}

	void write(void* data, size_t data_size)
	{
		if (offset + data_size > max_capacity)
		{
			data_size = max_capacity - offset; // Truncate to prevent overflow
		}

		memcpy(buffer + offset, data, data_size);
		offset += data_size;
	}
};