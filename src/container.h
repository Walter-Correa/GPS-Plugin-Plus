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

		void SetDangerWeight(int id, float weight);
		float GetDangerWeight(int id);
		void SetAreaDangerWeight(float minx, float miny, float maxx, float maxy, float weight);
		void ResetDangerWeights();
	}

	namespace Connections
	{
		int Add(int sourceID, int targetID, float road_width = 8.0f, int left_lanes = 1, int right_lanes = 1, bool one_way = false);
		void Delete(Connection* connection);
		Connection* Find(int id);
		bool FindClosestRoadPoint(float x, float y, float z, float& out_x, float& out_y, float& out_z, float& out_angle, float& out_road_width, int& out_conn_id, PathType type = PATH_TYPE_VEHICLE);
		const std::unordered_map<int, Connection*>& GetAll();
		int GetHighestID();

		void SetTrafficMultiplier(int id, float multiplier);
		float GetTrafficMultiplier(int id);
		void SetAreaTrafficMultiplier(float minx, float miny, float maxx, float maxy, float multiplier);
		void ResetTrafficMultipliers();

		void SetToll(int id, bool has_toll, int cost);
		bool GetToll(int id, int& out_cost);

		void SetPaved(int id, bool paved);
		bool IsPaved(int id);

		RoadClass GetRoadClass(int id);
		float GetSpeedLimit(int id);
		void SetSpeedLimit(int id, float speed);

		float GetMaxHeight(int id);
		void SetMaxHeight(int id, float height);
		void SetAreaMaxHeight(float minx, float miny, float maxx, float maxy, float height);
	}
	
	namespace Paths
	{
		int Add(Path* path);
		void Delete(Path* path);
		Path* Find(int id);
		std::unordered_map<int, Path*> GetAll();
		int GetHighestID();
		void ProcessTick();
	}
};
