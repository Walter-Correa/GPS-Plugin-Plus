#pragma once


namespace AmxHandler
{
	void LoadAmx(AMX* amx);
	void UnloadAmx(AMX* amx);
	Amx* GetAmx(AMX* amx);
	bool IsAmxValid(AMX* amx, uint64_t serial);
	uint64_t GetAmxSerial(AMX* amx);
	void ProcessTick();
};