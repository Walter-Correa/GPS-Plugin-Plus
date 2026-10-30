#pragma once

#include <vector>
#include <mutex>
#include "common.h"

class Node;
class Path;

namespace Tunnels
{
	struct TunnelArea
	{
		int id;
		float minx, miny, minz;
		float maxx, maxy, maxz;
	};

	void Initialize();
	int RegisterTunnelArea(float minx, float miny, float minz, float maxx, float maxy, float maxz);
	bool UnregisterTunnelArea(int id);
	void ClearCustomTunnelAreas();

	bool IsPointInTunnel(float x, float y, float z);
	bool IsMapNodeUnderground(Node* node);
	bool GetPathTunnelSegments(Path* path, float& out_entry_distance, float& out_exit_distance);
}
