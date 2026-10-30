#pragma once

#include <unordered_map>
#include <mutex>
#include "common.h"

class Path;

namespace Radars
{
	struct RoadRadar
	{
		int id;
		float x, y, z;
		float speed_limit_kmh;
		float alert_radius;
	};

	bool RegisterRadar(int id, float x, float y, float z, float speed_limit_kmh, float alert_radius = 300.0f);
	bool UnregisterRadar(int id);
	void ClearRadars();

	bool GetNextPathRadar(Path* path, float current_distance, int& out_radar_id, float& out_distance_to_radar, float& out_speed_limit);
}
