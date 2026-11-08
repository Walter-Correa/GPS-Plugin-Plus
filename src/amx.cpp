#include "amx.h"
#include "callback.h"


Amx::Amx(AMX* amx, uint64_t serial)
	: internal_amx_(amx), load_serial_(serial)
{
}


Amx::~Amx()
{
	std::lock_guard<std::mutex> lock(callback_queue_lock_);
	callback_queue_.clear();
}


uint64_t Amx::getLoadSerial() const
{
	return load_serial_;
}


void Amx::queueCallback(Callback* callback)
{
	if (callback == nullptr) return;
	std::lock_guard<std::mutex> lock(callback_queue_lock_);
	callback_queue_.emplace_back(callback);
}


void Amx::processCallbacks()
{
	std::vector<std::unique_ptr<Callback>> local_queue;
	{
		std::lock_guard<std::mutex> lock(callback_queue_lock_);
		local_queue.swap(callback_queue_);
	}

	for (auto& callback : local_queue)
	{
		if (callback)
		{
			callback->call();
		}
	}
}


AMX* Amx::getInternalAmx() const
{
	return internal_amx_;
}
