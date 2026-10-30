#include <algorithm>
#include <unordered_map>
#include <cmath>

#include "tunnels.h"
#include "node.h"
#include "path.h"

namespace Tunnels
{
	namespace
	{
		std::vector<TunnelArea> default_tunnels = {
			// Market Station Subway & Train Underground
			{ 1, 750.0f, -1450.0f, -25.0f, 860.0f, -1280.0f, 5.0f },
			// Verdant Bluffs Highway Tunnel
			{ 2, 970.0f, -2080.0f, 0.0f, 1180.0f, -1900.0f, 25.0f },
			// Easter Bay Airport & Freeway Tunnel (San Fierro)
			{ 3, -1590.0f, 700.0f, -15.0f, -1370.0f, 1060.0f, 25.0f },
			// Las Venturas Airport Runway Freeway Underpass (Julius Thruway)
			{ 4, 1300.0f, 1300.0f, 0.0f, 1750.0f, 1500.0f, 20.0f },
			// Mulholland Intersection Cut / Tunnel
			{ 5, 780.0f, -760.0f, 10.0f, 960.0f, -640.0f, 32.0f },
			// Garver Bridge North Approach Cut / Tunnel
			{ 6, -1420.0f, 2020.0f, 20.0f, -1350.0f, 2160.0f, 55.0f }
		};

		std::unordered_map<int, TunnelArea> custom_tunnels;
		int next_custom_id = 100;
		std::mutex tunnels_lock;
	}

	void Initialize()
	{
		// Default tunnels are statically initialized.
	}

	int RegisterTunnelArea(float minx, float miny, float minz, float maxx, float maxy, float maxz)
	{
		std::lock_guard<std::mutex> guard(tunnels_lock);
		int id = ++next_custom_id;
		TunnelArea area;
		area.id = id;
		area.minx = std::min(minx, maxx);
		area.maxx = std::max(minx, maxx);
		area.miny = std::min(miny, maxy);
		area.maxy = std::max(miny, maxy);
		area.minz = std::min(minz, maxz);
		area.maxz = std::max(minz, maxz);
		custom_tunnels[id] = area;
		return id;
	}

	bool UnregisterTunnelArea(int id)
	{
		std::lock_guard<std::mutex> guard(tunnels_lock);
		return custom_tunnels.erase(id) > 0;
	}

	void ClearCustomTunnelAreas()
	{
		std::lock_guard<std::mutex> guard(tunnels_lock);
		custom_tunnels.clear();
	}

	static inline bool PointInsideBox(float x, float y, float z, const TunnelArea& box)
	{
		return (x >= box.minx && x <= box.maxx &&
			    y >= box.miny && y <= box.maxy &&
			    z >= box.minz && z <= box.maxz);
	}

	bool IsPointInTunnel(float x, float y, float z)
	{
		for (const auto& box : default_tunnels)
		{
			if (PointInsideBox(x, y, z, box))
			{
				return true;
			}
		}

		std::lock_guard<std::mutex> guard(tunnels_lock);
		for (const auto& pair : custom_tunnels)
		{
			if (PointInsideBox(x, y, z, pair.second))
			{
				return true;
			}
		}

		return false;
	}

	bool IsMapNodeUnderground(Node* node)
	{
		if (node == nullptr) return false;
		return IsPointInTunnel(node->getX(), node->getY(), node->getZ());
	}

	bool GetPathTunnelSegments(Path* path, float& out_entry_distance, float& out_exit_distance)
	{
		out_entry_distance = 0.0f;
		out_exit_distance = 0.0f;
		if (path == nullptr) return false;

		const auto& nodes = path->getNodes();
		if (nodes.size() < 2) return false;

		float accum_dist = 0.0f;
		bool in_tunnel = false;
		bool found_tunnel = false;

		for (size_t i = 0; i < nodes.size(); ++i)
		{
			if (i > 0)
			{
				accum_dist += nodes[i - 1]->getDistanceFromNode(nodes[i]);
			}

			const bool current_inside = IsMapNodeUnderground(nodes[i]);
			if (current_inside && !in_tunnel)
			{
				if (!found_tunnel)
				{
					out_entry_distance = accum_dist;
					found_tunnel = true;
				}
				in_tunnel = true;
			}
			else if (!current_inside && in_tunnel)
			{
				out_exit_distance = accum_dist;
				in_tunnel = false;
			}
		}

		if (found_tunnel && in_tunnel)
		{
			out_exit_distance = accum_dist;
		}

		return found_tunnel;
	}
}
