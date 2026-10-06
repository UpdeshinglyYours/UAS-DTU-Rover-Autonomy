#pragma once

#include "common.h"

class lwSerialPort {
	public:
		virtual ~lwSerialPort() = default;
		virtual bool connect(const char* Name, int BitRate) = 0;
		virtual bool disconnect() = 0;
		virtual int writeData(uint8_t *Buffer, int32_t BufferSize) = 0;
		virtual int32_t readData(uint8_t *Buffer, int32_t BufferSize) = 0;
};