#include <unordered_map>
#include <vector>
#include <mutex>
#include <memory>

#include "amx.h"
#include "container.h"
#include "worker_pool.h"
#include "logger.h"


namespace AmxHandler
{
	namespace
	{
		std::unordered_map<AMX*, std::unique_ptr<Amx>> amx_list;
		std::mutex amx_list_mutex;
		std::atomic<uint64_t> global_load_serial{1};
	}


	void LoadAmx(AMX* amx)
	{
		std::lock_guard<std::mutex> lock(amx_list_mutex);
		uint64_t serial = global_load_serial.fetch_add(1, std::memory_order_relaxed);
		amx_list[amx] = std::make_unique<Amx>(amx, serial);
	}


	void UnloadAmx(AMX* amx)
	{
		WorkerPool::DiscardForAmx(amx);
		Container::Paths::DestroyByOwner(amx);

		std::lock_guard<std::mutex> lock(amx_list_mutex);
		amx_list.erase(amx);
	}


	Amx* GetAmx(AMX* amx)
	{
		std::lock_guard<std::mutex> lock(amx_list_mutex);
		const auto it = amx_list.find(amx);
		if (it == amx_list.end())
		{
			return nullptr;
		}
		return it->second.get();
	}


	bool IsAmxValid(AMX* amx, uint64_t serial)
	{
		std::lock_guard<std::mutex> lock(amx_list_mutex);
		const auto it = amx_list.find(amx);
		if (it == amx_list.end() || it->second == nullptr)
		{
			return false;
		}
		return it->second->getLoadSerial() == serial;
	}


	uint64_t GetAmxSerial(AMX* amx)
	{
		std::lock_guard<std::mutex> lock(amx_list_mutex);
		const auto it = amx_list.find(amx);
		if (it == amx_list.end() || it->second == nullptr)
		{
			return 0;
		}
		return it->second->getLoadSerial();
	}


	void ProcessTick()
	{
		try
		{
			GPSLogger::FlushQueuedLogs();
			WorkerPool::ProcessResults();

			std::vector<Amx*> active_amx;
			{
				std::lock_guard<std::mutex> lock(amx_list_mutex);
				active_amx.reserve(amx_list.size());
				for (const auto& pair : amx_list)
				{
					if (pair.second != nullptr)
					{
						active_amx.push_back(pair.second.get());
					}
				}
			}

			for (auto amx : active_amx)
			{
				if (amx != nullptr)
				{
					amx->processCallbacks();
				}
			}

			Container::Paths::ProcessTick();
		}
		catch (...)
		{
		}
	}
}
