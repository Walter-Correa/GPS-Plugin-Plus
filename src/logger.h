#pragma once

#include "common.h"

namespace GPSLogger
{
	void InitMainThread();
	void Log(const char* format, ...);
	void FlushQueuedLogs();
}
