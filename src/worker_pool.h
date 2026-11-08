#pragma once

#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include "common.h"
#include "callback.h"

struct PathJob
{
	int start_id;
	int target_id;
	PathType type;
	RoutePreference pref;
	VehicleCategory cat;
	AMX* amx;
	uint64_t amx_serial;
	Callback* callback;
	int yield_count{ 0 };
	std::chrono::steady_clock::time_point first_attempt_time{};
};

struct PathJobResult
{
	AMX* amx;
	uint64_t amx_serial;
	Callback* callback;
	bool success;
	std::vector<int> node_ids;
	float length;
};

namespace WorkerPool
{
	void Init(int thread_count = 0);
	void Shutdown();
	bool EnqueueJob(PathJob job);
	void ProcessResults();
	void DiscardForAmx(AMX* amx);
	int GetThreadCount();
}
