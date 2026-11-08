#pragma once

#include "amx.h"
#include <cstddef>
#include <cstring>
#include <type_traits>

namespace PawnUtils
{
	static inline cell FloatToCell(float f) noexcept
	{
		cell c = 0;
		std::memcpy(&c, &f, sizeof(cell));
		return c;
	}

	static inline float CellToFloat(cell c) noexcept
	{
		float f = 0.0f;
		std::memcpy(&f, &c, sizeof(float));
		return f;
	}

	static inline bool IsValidAmxAddress(AMX* amx, const void* ptr, size_t size_bytes = sizeof(cell))
	{
		if (amx == nullptr || ptr == nullptr) return false;
		unsigned char* data = (amx->data != nullptr) ? amx->data : (amx->base + reinterpret_cast<AMX_HEADER*>(amx->base)->dat);
		if (!data) return false;
		const unsigned char* p = reinterpret_cast<const unsigned char*>(ptr);
		const unsigned char* data_end = data + amx->stp;
		return (p >= data && p < data_end && size_bytes <= static_cast<size_t>(data_end - p));
	}

	static inline cell* GetArray(AMX* amx, cell amx_addr, size_t cell_count)
	{
		if (cell_count == 0) return nullptr;
		cell* ptr = nullptr;
		if (amx_GetAddr(amx, amx_addr, &ptr) != AMX_ERR_NONE || ptr == nullptr)
		{
			return nullptr;
		}
		if (!IsValidAmxAddress(amx, ptr, cell_count * sizeof(cell)))
		{
			return nullptr;
		}
		return ptr;
	}

	static inline cell* GetAddress(AMX* amx, cell amx_addr)
	{
		return GetArray(amx, amx_addr, 1);
	}

	static inline cell* GetPawnSubArray(AMX* amx, cell* base, int row, size_t row_cells = 0)
	{
		if (base == nullptr) return nullptr;
		cell* indirection_cell = &base[row];
		if (!IsValidAmxAddress(amx, indirection_cell, sizeof(cell)))
		{
			return nullptr;
		}
		cell* sub = reinterpret_cast<cell*>(reinterpret_cast<unsigned char*>(indirection_cell) + *indirection_cell);
		if (row_cells > 0 && !IsValidAmxAddress(amx, sub, row_cells * sizeof(cell)))
		{
			return nullptr;
		}
		return sub;
	}

	template <typename T>
	static inline bool WriteRef(AMX* amx, cell amx_addr, T val)
	{
		cell* ptr = nullptr;
		if (amx_GetAddr(amx, amx_addr, &ptr) != AMX_ERR_NONE || ptr == nullptr)
		{
			return false;
		}
		if (!IsValidAmxAddress(amx, ptr, sizeof(cell)))
		{
			return false;
		}
		if constexpr (std::is_floating_point_v<T>)
		{
			*ptr = FloatToCell(static_cast<float>(val));
		}
		else
		{
			*ptr = static_cast<cell>(val);
		}
		return true;
	}
}
