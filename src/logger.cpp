#include "logger.h"
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

namespace GPSLogger
{
	namespace
	{
		std::thread::id g_main_thread_id;
		std::atomic<bool> g_has_main_thread{ false };
		std::mutex g_log_mutex;
		std::vector<std::string> g_queued_logs;
	}

	void InitMainThread()
	{
		g_main_thread_id = std::this_thread::get_id();
		g_has_main_thread.store(true, std::memory_order_release);
	}

	void Log(const char* format, ...)
	{
		char buffer[1024];
		va_list args;
		va_start(args, format);
		vsnprintf(buffer, sizeof(buffer), format, args);
		va_end(args);

		if (g_has_main_thread.load(std::memory_order_acquire) && std::this_thread::get_id() == g_main_thread_id)
		{
			if (logprintf != nullptr)
			{
				logprintf("%s", buffer);
			}
		}
		else
		{
			std::lock_guard<std::mutex> lock(g_log_mutex);
			g_queued_logs.emplace_back(buffer);
		}
	}

	void FlushQueuedLogs()
	{
		std::vector<std::string> logs_to_print;
		{
			std::lock_guard<std::mutex> lock(g_log_mutex);
			if (g_queued_logs.empty())
			{
				return;
			}
			logs_to_print.swap(g_queued_logs);
		}

		for (const auto& msg : logs_to_print)
		{
			if (logprintf != nullptr)
			{
				logprintf("%s", msg.c_str());
			}
		}
	}
}
