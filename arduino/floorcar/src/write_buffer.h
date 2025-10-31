#pragma once

#include "Arduino.h"

#include <string.h>

#define max_capacity 258

struct write_buffer
{
	unsigned char buffer[max_capacity];
	size_t offset;

	write_buffer(unsigned char payload_type) :
		offset(sizeof(unsigned short))
	{
		memset(buffer, 0, sizeof(unsigned short));
		write(&payload_type, sizeof(unsigned char));
	}

	void write(void* data, size_t data_size)
	{
		// If we overflow, we overflow
		memcpy(buffer + offset, data, data_size);
		offset += data_size;

		// Update payload size
		unsigned short new_payload_size = offset - sizeof(unsigned short);
		memcpy(buffer, &new_payload_size, sizeof(unsigned short));
	}

	void transmit()
	{
		if (Serial.availableForWrite() > 0)
		{
			Serial.write(buffer, offset);
		}
	}
};