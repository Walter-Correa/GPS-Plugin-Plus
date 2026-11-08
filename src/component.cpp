#include <exception>
#include <stdexcept>
#include <cstdlib>
#include <cstdarg>
#include <cstdio>

#include <sdk.hpp>
#include <Server/Components/Pawn/pawn.hpp>

#include "common.h"
#include "natives.h"
#include "file.h"
#include "amx_handler.h"
#include "worker_pool.h"
#include "logger.h"

logprintf_t logprintf = nullptr;
extern AMX_NATIVE_INFO PluginNatives[];

static ICore* g_core = nullptr;

static void ComponentLogprintf(const char* format, ...)
{
	char buffer[1024];
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	if (g_core != nullptr)
	{
		g_core->printLn("%s", buffer);
	}
}

static void CheckPluginConflict()
{
	if (std::getenv("GPS_LEGACY_PLUGIN_LOADED") != nullptr)
	{
		if (g_core != nullptr)
		{
			g_core->printLn("%s", "[GPS component]: WARNING: Legacy GPS plugin is also loaded! Running both GPS plugin and component concurrently may cause conflicts.");
		}
	}
}

static const UID GPSComponent_UID = UID(0x6198c2574e89f10a);

class GPSComponent final : public IComponent, public CoreEventHandler, public PawnEventHandler
{
private:
	ICore* core_ = nullptr;
	IPawnComponent* pawn_ = nullptr;

public:
	PROVIDE_UID(GPSComponent_UID);

	StringView componentName() const override
	{
		return "GPS-Plugin-Plus";
	}

	SemanticVersion componentVersion() const override
	{
		return SemanticVersion(2, 0, 0, 0);
	}

	void onLoad(ICore* c) override
	{
		core_ = c;
		g_core = c;
		logprintf = ComponentLogprintf;

		GPSLogger::InitMainThread();

#if defined(_WIN32) || defined(WIN32)
		_putenv("GPS_COMPONENT_LOADED=1");
#else
		setenv("GPS_COMPONENT_LOADED", "1", 1);
#endif

		CheckPluginConflict();

		IConfig& cfg = core_->getConfig();
		bool load_nodes = true;
		int* load_nodes_int = cfg.getInt("gps.load_nodes");
		if (load_nodes_int != nullptr)
		{
			load_nodes = (*load_nodes_int != 0);
		}
		else
		{
			bool* load_nodes_bool = cfg.getBool("gps.load_nodes");
			if (load_nodes_bool != nullptr)
			{
				load_nodes = *load_nodes_bool;
			}
		}

		int worker_threads = 0;
		int* worker_threads_cfg = cfg.getInt("gps.worker_threads");
		if (worker_threads_cfg != nullptr)
		{
			worker_threads = std::clamp(*worker_threads_cfg, 0, 16);
		}

		WorkerPool::Init(worker_threads);

		core_->getEventDispatcher().addEventHandler(this);

		if (load_nodes)
		{
			auto line_count = 0, node_count = 0, connection_count = 0;
			if (!File::LoadNodes("GPS.dat", line_count, node_count, connection_count))
			{
				core_->printLn("%s", "[GPS component]: Failed to open \"GPS.dat\"!");
			}
			else
			{
				core_->printLn("[GPS component]: Read %i lines, loaded %i nodes and %i connections.", line_count, node_count, connection_count);
			}
		}
	}

	void onInit(IComponentList* components) override
	{
		pawn_ = components->queryComponent<IPawnComponent>();
		if (pawn_ != nullptr)
		{
			pAMXFunctions = const_cast<void**>(pawn_->getAmxFunctions().data());
			pawn_->getEventDispatcher().addEventHandler(this);
		}
	}

	void onReady() override
	{
	}

	void onFree(IComponent* component) override
	{
		if (component == pawn_)
		{
			pawn_ = nullptr;
		}
	}

	void free() override
	{
		WorkerPool::Shutdown();
		if (core_ != nullptr)
		{
			core_->getEventDispatcher().removeEventHandler(this);
		}
		if (pawn_ != nullptr)
		{
			pawn_->getEventDispatcher().removeEventHandler(this);
		}
		delete this;
	}

	void reset() override
	{
	}

	// CoreEventHandler
	void onTick(Microseconds elapsed, TimePoint now) override
	{
		AmxHandler::ProcessTick();
	}

	// PawnEventHandler
	void onAmxLoad(IPawnScript& script) override
	{
		CheckPluginConflict();
		AMX* amx = script.GetAMX();
		AmxHandler::LoadAmx(amx);
		amx_Register(amx, PluginNatives, -1);
	}

	void onAmxUnload(IPawnScript& script) override
	{
		AMX* amx = script.GetAMX();
		AmxHandler::UnloadAmx(amx);
	}
};

COMPONENT_ENTRY_POINT()
{
	return new GPSComponent();
}
