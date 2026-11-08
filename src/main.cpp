#include <algorithm>
#include <cstdlib>
#include "common.h"
#include "natives.h"
#include "file.h"
#include "amx_handler.h"
#include "config_reader.h"
#include "worker_pool.h"
#include "logger.h"


logprintf_t logprintf;

static void CheckComponentConflict()
{
	if (std::getenv("GPS_COMPONENT_LOADED") != nullptr)
	{
		GPSLogger::Log("[GPS plugin]: WARNING: Native open.mp GPS component is also loaded! Running both GPS plugin and component concurrently may cause conflicts.");
	}
}


PLUGIN_EXPORT unsigned int PLUGIN_CALL Supports()
{
	return SUPPORTS_VERSION | SUPPORTS_AMX_NATIVES | SUPPORTS_PROCESS_TICK;
}


PLUGIN_EXPORT bool PLUGIN_CALL Load(void **ppData)
{
	pAMXFunctions = ppData[PLUGIN_DATA_AMX_EXPORTS];
	logprintf = (logprintf_t)(ppData[PLUGIN_DATA_LOGPRINTF]);

	GPSLogger::InitMainThread();

#if defined(_WIN32) || defined(WIN32)
	_putenv("GPS_LEGACY_PLUGIN_LOADED=1");
#else
	setenv("GPS_LEGACY_PLUGIN_LOADED", "1", 1);
#endif

	GPSLogger::Log("[GPS-Plugin-Plus]: v2.0.0 loaded.");
	CheckComponentConflict();

	auto config = ConfigReader("server.cfg");
	const int worker_threads = std::clamp(config.getValue<int>("gps_worker_threads", 0), 0, 16);
	WorkerPool::Init(worker_threads);

	if (config.getValue<int>("gps_load_nodes", 1) != 0)
	{
		auto line_count = 0, node_count = 0, connection_count = 0;

		if (!File::LoadNodes("GPS.dat", line_count, node_count, connection_count))
		{
			GPSLogger::Log("[GPS plugin]: Failed to open \"GPS.dat\"!");
		}
		else
		{
			GPSLogger::Log("[GPS plugin]: Read %i lines, loaded %i nodes and %i connections.", line_count, node_count, connection_count);
		}
	}

	return true;
}


PLUGIN_EXPORT void PLUGIN_CALL Unload()
{
	WorkerPool::Shutdown();
	GPSLogger::Log("[GPS plugin]: Unloaded.");
}


PLUGIN_EXPORT int PLUGIN_CALL AmxLoad(AMX *amx)
{
	CheckComponentConflict();
	AmxHandler::LoadAmx(amx);

	return amx_Register(amx, PluginNatives, -1);
}


PLUGIN_EXPORT int PLUGIN_CALL AmxUnload(AMX* amx)
{
	AmxHandler::UnloadAmx(amx);

	return 1;
}


PLUGIN_EXPORT void PLUGIN_CALL ProcessTick()
{
	AmxHandler::ProcessTick();
}
