#pragma once

#include <unordered_map>

#include "path.h"
#include "common.h"


namespace Container
{
	void LockExclusive();
	void UnlockExclusive();

	void LockShared();
	void UnlockShared();

	namespace Nodes
	{
		int AllocateID();
		bool Add(int id, float x, float y, float z, PathType type = PATH_TYPE_VEHICLE, uint32_t flags = NODE_FLAG_NONE);
		Node* Find(int id);
		Node* FindClosest(float x, float y, float z, PathType type = PATH_TYPE_VEHICLE, int ignored_id = -1);
		const std::unordered_map<int, Node*>& GetAll();
		int GetHighestID(PathType type = static_cast<PathType>(-1));
		void SetForDeletion(Node* node);
		bool TryDelete(Node* node);
	}

	namespace Connections
	{
		int Add(int sourceID, int targetID, float road_width = 8.0f, int left_lanes = 1, int right_lanes = 1, bool one_way = false);
		void Delete(Connection* connection);
		Connection* Find(int id);
		bool FindClosestRoadPoint(float x, float y, float z, float& out_x, float& out_y, float& out_z, float& out_angle, float& out_road_width, int& out_conn_id, PathType type = PATH_TYPE_VEHICLE);
		const std::unordered_map<int, Connection*>& GetAll();
		int GetHighestID();
	}
	
	namespace Paths
	{
		int Add(Path* path);
		void Delete(Path* path);
		Path* Find(int id);
		std::unordered_map<int, Path*> GetAll();
		int GetHighestID();
	}
};
