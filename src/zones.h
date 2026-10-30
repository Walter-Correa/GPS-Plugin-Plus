#pragma once

#include <string>
#include <vector>

class Node;

struct ZoneInfo
{
	const char* name;
	float minx, miny, minz;
	float maxx, maxy, maxz;
};

namespace Zones
{
	std::string GetZoneNameAt(float x, float y, float z);
	std::vector<std::string> GetPathZones(const std::vector<Node*>& nodes);
}
