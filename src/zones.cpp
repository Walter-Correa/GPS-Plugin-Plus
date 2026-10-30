#include "zones.h"
#include "node.h"

#include "zones_data.inl"

namespace Zones
{
	std::string GetZoneNameAt(const float x, const float y, const float z)
	{
		for (const auto& zone : g_SanAndreasZones)
		{
			if (x >= zone.minx && x <= zone.maxx &&
			    y >= zone.miny && y <= zone.maxy &&
			    z >= zone.minz && z <= zone.maxz)
			{
				return zone.name;
			}
		}

		for (const auto& zone : g_SanAndreasZones)
		{
			if (x >= zone.minx && x <= zone.maxx &&
			    y >= zone.miny && y <= zone.maxy)
			{
				return zone.name;
			}
		}

		return "San Andreas";
	}

	std::vector<std::string> GetPathZones(const std::vector<Node*>& nodes)
	{
		std::vector<std::string> result;
		std::string last_zone = "";

		for (const auto node : nodes)
		{
			if (node == nullptr)
			{
				continue;
			}

			std::string current_zone = GetZoneNameAt(node->getX(), node->getY(), node->getZ());
			if (current_zone != last_zone && !current_zone.empty())
			{
				result.push_back(current_zone);
				last_zone = current_zone;
			}
		}

		return result;
	}
}
