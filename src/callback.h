#pragma once

#include <string>
#include <deque>
#include <variant>

#include "common.h"


class Callback
{
public:
	Callback(AMX* amx, uint64_t amx_serial, const std::string& function, const char* format, cell* params, unsigned int param_offset);
	void setResult(int value);
	AMX* getAmxRaw() const;
	uint64_t getAmxSerial() const;
	bool isValid() const;
	bool call();

private:
	AMX* amx_raw_;
	uint64_t amx_serial_;
	std::string function_;
	std::deque<std::variant<int, std::string>> arguments_;
	int result_;
	bool valid_;
};
