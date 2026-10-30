#include <cmath>
#include <limits>
#include <algorithm>

#include "radars.h"
#include "path.h"
#include "node.h"

namespace Radars
{
	namespace
	{
		std::unordered_map<int, RoadRadar> radars;
		std::mutex radars_lock;
	}

	bool RegisterRadar(const int id, const float x, const float y, const float z, const float speed_limit_kmh, const float alert_radius)
	{
		std::lock_guard<std::mutex> guard(radars_lock);
		RoadRadar r;
		r.id = id;
		r.x = x;
		r.y = y;
		r.z = z;
		r.speed_limit_kmh = speed_limit_kmh;
		r.alert_radius = (alert_radius > 10.0f) ? alert_radius : 10.0f;
		radars[id] = r;
		return true;
	}

	bool UnregisterRadar(const int id)
	{
		std::lock_guard<std::mutex> guard(radars_lock);
		return radars.erase(id) > 0;
	}

	void ClearRadars()
	{
		std::lock_guard<std::mutex> guard(radars_lock);
		radars.clear();
	}

	bool GetNextPathRadar(Path* path, const float current_distance, int& out_radar_id, float& out_distance_to_radar, float& out_speed_limit)
	{
		out_radar_id = -1;
		out_distance_to_radar = 0.0f;
		out_speed_limit = 0.0f;

		if (path == nullptr) return false;

		const auto& nodes = path->getNodes();
		if (nodes.size() < 2) return false;

		std::lock_guard<std::mutex> guard(radars_lock);
		if (radars.empty()) return false;

		float accum_dist = 0.0f;
		float earliest_dist_ahead = std::numeric_limits<float>::infinity();
		int chosen_radar_id = -1;
		float chosen_limit = 0.0f;

		for (size_t i = 0; i < nodes.size() - 1; ++i)
		{
			const float ax = nodes[i]->getX();
			const float ay = nodes[i]->getY();
			const float az = nodes[i]->getZ();

			const float bx = nodes[i + 1]->getX();
			const float by = nodes[i + 1]->getY();
			const float bz = nodes[i + 1]->getZ();

			const float seg_len = nodes[i]->getDistanceFromNode(nodes[i + 1]);
			const float seg_start = accum_dist;
			const float seg_end = accum_dist + seg_len;
			accum_dist = seg_end;

			if (seg_end < current_distance)
			{
				continue;
			}

			for (const auto& pair : radars)
			{
				const auto& r = pair.second;

				const float abx = bx - ax;
				const float aby = by - ay;
				const float abz = bz - az;
				const float apx = r.x - ax;
				const float apy = r.y - ay;
				const float apz = r.z - az;

				const float ab_len_sq = abx * abx + aby * aby + abz * abz;
				float t = 0.0f;
				if (ab_len_sq > 0.0001f)
				{
					t = (apx * abx + apy * aby + apz * abz) / ab_len_sq;
					t = std::max(0.0f, std::min(1.0f, t));
				}

				const float qx = ax + t * abx;
				const float qy = ay + t * aby;
				const float qz = az + t * abz;

				const float dx = r.x - qx;
				const float dy = r.y - qy;
				const float dz = r.z - qz;
				const float dist_to_radar_point = std::sqrt(dx * dx + dy * dy + dz * dz);

				if (dist_to_radar_point <= r.alert_radius)
				{
					const float radar_path_dist = seg_start + (t * seg_len);
					if (radar_path_dist >= current_distance && radar_path_dist < earliest_dist_ahead)
					{
						earliest_dist_ahead = radar_path_dist;
						chosen_radar_id = r.id;
						chosen_limit = r.speed_limit_kmh;
					}
				}
			}
		}

		if (chosen_radar_id != -1)
		{
			out_radar_id = chosen_radar_id;
			out_distance_to_radar = earliest_dist_ahead - current_distance;
			out_speed_limit = chosen_limit;
			return true;
		}

		return false;
	}
}
