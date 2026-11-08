#pragma once

#include "container.h"
#include "callback.h"
#include "common.h"
#include <exception>

struct PathfinderYieldException : public std::exception
{
	const char* what() const noexcept override { return "Pathfinder yielded for graph writer"; }
};

namespace Pathfinder
{
	bool FindPathNodes(Node* start, Node* target, std::vector<Node*>& out_nodes, float& out_length, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST, VehicleCategory cat = VEHICLE_CAT_STANDARD, float danger_penalty = 0.0f, float max_slope_gradient = 0.0f, bool avoid_tolls = false, bool evasive = false, float vehicle_height = 0.0f, bool allow_yield = true);
	Path* FindPathInternal(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST, VehicleCategory cat = VEHICLE_CAT_STANDARD, float danger_penalty = 0.0f, float max_slope_gradient = 0.0f, bool avoid_tolls = false, bool evasive = false, float vehicle_height = 0.0f);
	int FindPath(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST, VehicleCategory cat = VEHICLE_CAT_STANDARD, AMX* amx = nullptr);
	Path* FindMultiStopPathInternal(const std::vector<Node*>& stops, bool optimize_order, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST);
	int FindMultiStopPath(const std::vector<Node*>& stops, bool optimize_order, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST, AMX* amx = nullptr);

	int FindSafePath(Node* start, Node* target, float danger_penalty = 5.0f, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST, AMX* amx = nullptr);
	int FindEcoPath(Node* start, Node* target, float max_slope_gradient = 0.12f, PathType type = PATH_TYPE_VEHICLE, AMX* amx = nullptr);
	int FindEvasivePath(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE, AMX* amx = nullptr);
	int FindPathAvoidTolls(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST, AMX* amx = nullptr);
	int FindPathPavedOnly(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE, AMX* amx = nullptr);
	int FindPathWithHeightLimit(Node* start, Node* target, float vehicle_height, PathType type = PATH_TYPE_VEHICLE, RoutePreference pref = ROUTE_PREF_SHORTEST, AMX* amx = nullptr);

	int FindNearestPointByRoad(float from_x, float from_y, float from_z, const std::vector<std::tuple<float, float, float>>& destinations, int& out_nearest_index, float& out_road_distance, int& out_path_id, PathType type = PATH_TYPE_VEHICLE, AMX* amx = nullptr, bool create_path = false);

	bool IsMapNodeDeadEnd(Node* node);
	bool GetPathElevationStats(Path* path, float& out_min_z, float& out_max_z, float& out_total_climb, float& out_max_incline_percent);
	bool GetPathOffsetPosition(Path* path, float distance_along_path, float lateral_offset, float& out_x, float& out_y, float& out_z, float& out_angle);

	bool GetSmoothPathPoints(Path* path, float spacing_meters, std::vector<std::tuple<float, float, float>>& out_points, int max_points);
	bool GetPathRoundaboutInfo(Path* path, int index, bool& is_roundabout, int& exit_number, float& roundabout_radius);
	bool GetPathRejoinPoint(Path* old_path, Path* new_path, int& diverge_step, int& rejoin_old_step, int& rejoin_new_step);

	int FindInterceptPath(Node* pursuer, Node* target, float target_speed_kmh, float pursuer_speed_kmh, int& out_intercept_node_id, float& out_time_to_intercept, PathType type = PATH_TYPE_VEHICLE, AMX* amx = nullptr);
	std::vector<int> GetReachableNodes(Node* source, float limit_value, bool by_time, float speed_kmh, PathType type = PATH_TYPE_VEHICLE);
};
