#pragma once

#include <vector>
#include <memory>
#include <mutex>

#include "common.h"


class Amx
{
public:
	Amx(AMX* amx, uint64_t serial = 0);
	~Amx();

	void queueCallback(Callback* callback);
	void processCallbacks();
	AMX* getInternalAmx() const;
	uint64_t getLoadSerial() const;

private:
	std::vector<std::unique_ptr<Callback>> callback_queue_;
	std::mutex callback_queue_lock_;
	AMX* internal_amx_;
	uint64_t load_serial_{0};
};
