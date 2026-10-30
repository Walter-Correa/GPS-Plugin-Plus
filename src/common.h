#pragma once

#include <cstdint>
#include <exception>

#include "../lib/sdk/amx/amx.h"
#include "../lib/sdk/amx/amx2.h"
#include "../lib/sdk/plugincommon.h"


#if defined(OMP_COMPONENT)
#include <values.hpp>
#else
const int INVALID_NODE_ID = -1;
const int INVALID_PATH_ID = -1;
#endif
const int INVALID_CONNECTION_ID = -1;


enum PathType : uint8_t
{
	PATH_TYPE_VEHICLE = 0,
	PATH_TYPE_PEDESTRIAN = 1,
	PATH_TYPE_BOAT = 2
};

enum PathNodeFlags : uint32_t
{
	NODE_FLAG_NONE = 0,
	NODE_FLAG_HIGHWAY = 1 << 0,
	NODE_FLAG_EMERGENCY_ONLY = 1 << 1,
	NODE_FLAG_ROADBLOCK = 1 << 2,
	NODE_FLAG_CROSSWALK = 1 << 3,
	NODE_FLAG_ONEWAY = 1 << 4
};

enum TurnDirection : uint8_t
{
	TURN_NONE = 0,
	TURN_STRAIGHT = 1,
	TURN_RIGHT = 2,
	TURN_LEFT = 3,
	TURN_SHARP_RIGHT = 4,
	TURN_SHARP_LEFT = 5,
	TURN_UTURN = 6
};

enum RoutePreference : uint8_t
{
	ROUTE_PREF_SHORTEST = 0,
	ROUTE_PREF_HIGHWAYS = 1,
	ROUTE_PREF_AVOID_HIGHWAYS = 2
};

enum VehicleCategory : uint8_t
{
	VEHICLE_CAT_STANDARD = 0,
	VEHICLE_CAT_HEAVY_TRUCK = 1,
	VEHICLE_CAT_MOTORCYCLE = 2
};


typedef void(*logprintf_t)(const char* format, ...);
extern logprintf_t logprintf;
extern void *pAMXFunctions;

class Path;
class Node;
class Connection;
class Amx;
class Callback;
