#pragma once

#include "container.h"
#include "callback.h"
#include "common.h"


namespace Pathfinder
{
	Path* FindPathInternal(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST);
	int FindPath(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST);
	void FindPathThreaded(Node* start, Node* target, Callback* callback, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST);
	Path* FindMultiStopPathInternal(const std::vector<Node*>& stops, bool optimize_order, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST);
	int FindMultiStopPath(const std::vector<Node*>& stops, bool optimize_order, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST);
};
