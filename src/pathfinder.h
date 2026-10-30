#pragma once

#include "container.h"
#include "callback.h"
#include "common.h"


namespace Pathfinder
{
	Path* FindPathInternal(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST, VehicleCategory cat = VEHICLE_CAT_STANDARD);
	int FindPath(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST, VehicleCategory cat = VEHICLE_CAT_STANDARD);
	void FindPathThreaded(Node* start, Node* target, Callback* callback, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST, VehicleCategory cat = VEHICLE_CAT_STANDARD);
	Path* FindMultiStopPathInternal(const std::vector<Node*>& stops, bool optimize_order, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST);
	int FindMultiStopPath(const std::vector<Node*>& stops, bool optimize_order, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST);

	int FindInterceptPath(Node* pursuer, Node* target, float target_speed_kmh, float pursuer_speed_kmh, int& out_intercept_node_id, float& out_time_to_intercept, PathType type = PATH_TYPE_VEHICLE);
	std::vector<int> GetReachableNodes(Node* source, float limit_value, bool by_time, float speed_kmh, PathType type = PATH_TYPE_VEHICLE);
};
