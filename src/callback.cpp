#include <iostream>

#include "callback.h"
#include "amx.h"
#include "amx_handler.h"
#include "logger.h"


Callback::Callback(AMX* amx, uint64_t amx_serial, const std::string& function, const char* format, cell* params, const unsigned int param_offset)
{
	amx_raw_ = amx;
	amx_serial_ = amx_serial;
	function_ = function;
	result_ = 0;
	valid_ = true;

	if (amx == nullptr || params == nullptr)
	{
		valid_ = false;
		return;
	}

	if (format != nullptr && *format != '\0')
	{
		const size_t total_params = static_cast<size_t>(params[0] / sizeof(cell));
		auto param_index = param_offset + 1;

		while (*format != '\0')
		{
			if (param_index > total_params)
			{
				GPSLogger::Log("Callback \"%s\": format specifier count exceeds parameters passed (%d)", function_.c_str(), (int)total_params);
				valid_ = false;
				break;
			}

			switch (*format)
			{
			case 's':
			{
				cell* str_addr = nullptr;
				if (amx_GetAddr(amx_raw_, params[param_index++], &str_addr) == AMX_ERR_NONE && str_addr != nullptr)
				{
					int str_len = 0;
					if (amx_StrLen(str_addr, &str_len) == AMX_ERR_NONE && str_len >= 0)
					{
						std::string str(str_len, '\0');
						if (str_len > 0)
						{
							amx_GetString(&str[0], str_addr, 0, str_len + 1);
						}
						arguments_.push_front(str);
					}
					else
					{
						valid_ = false;
					}
				}
				else
				{
					valid_ = false;
				}
				break;
			}

			case 'i':
			case 'd':
			{
				cell* int_addr = nullptr;
				if (amx_GetAddr(amx_raw_, params[param_index++], &int_addr) == AMX_ERR_NONE && int_addr != nullptr)
				{
					arguments_.push_front(*int_addr);
				}
				else
				{
					valid_ = false;
				}
				break;
			}

			default:
			{
				GPSLogger::Log("Invalid format specifier '%c' in callback", *format);
				valid_ = false;
				break;
			}
			}

			if (!valid_)
			{
				break;
			}

			format++;
		}
	}
}


void Callback::setResult(const int value)
{
	result_ = value;
}


AMX* Callback::getAmxRaw() const
{
	return amx_raw_;
}


uint64_t Callback::getAmxSerial() const
{
	return amx_serial_;
}


bool Callback::isValid() const
{
	return valid_;
}


bool Callback::call()
{
	if (!valid_ || amx_raw_ == nullptr)
	{
		return false;
	}

	Amx* amx = AmxHandler::GetAmx(amx_raw_);
	if (amx == nullptr || amx->getLoadSerial() != amx_serial_)
	{
		return false;
	}

	const auto internal_amx = amx->getInternalAmx();
	if (internal_amx == nullptr)
	{
		return false;
	}

	auto amx_idx = 0;
	cell amx_ret;

	const auto error = amx_FindPublic(internal_amx, function_.c_str(), &amx_idx);

	if (error != AMX_ERR_NONE)
	{
		GPSLogger::Log("Failed to locate public function \"%s\" in AMX, error: %d", function_.c_str(), error);
		return false;
	}

	auto amx_address = -1;

	for (const auto& argument : arguments_)
	{
		if (argument.index() == 1)
		{
			cell tmp_address;

			amx_PushString(internal_amx, &tmp_address, nullptr, std::get<std::string>(argument).c_str(), 0, 0);

			if (amx_address == -1)
			{
				amx_address = tmp_address;
			}
		}
		else
		{
			amx_Push(internal_amx, std::get<int>(argument));
		}
	}

	amx_Push(internal_amx, result_);
	amx_Exec(internal_amx, &amx_ret, amx_idx);

	if (amx_address != -1)
	{
		amx_Release(internal_amx, amx_address);
	}

	return true;
}
