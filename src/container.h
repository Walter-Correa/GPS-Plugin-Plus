#pragma once

#include <unordered_map>
#include <shared_mutex>
#include <mutex>
#include <functional>
#include <chrono>
#include "path.h"
#include "common.h"

// THREAD SAFETY INVARIANTS & WRITER-PREFERRING CONCURRENCY ARCHITECTURE:
// 1. graph_mutex (std::shared_mutex) protects the graph structure:
//    nodes, connections, spatial_grid, and Node::connections_.
//    - Background workers acquire shared_lock(graph_mutex) during pathfinding searches.
//    - Structural mutations (CreateMapNode, DestroyMapNode, CreateConnection,
//      DestroyConnection, LoadNodes) acquire exclusive write access via GraphWriterLock.
//    - Concurrency Choice C (Writer Priority with Cooperative Search Yield):
//      Main-thread writers increment an atomic pending-writer counter before attempting lock acquisition.
//      Background A* workers poll HasPendingWriters() periodically (every 64 node expansions).
//      When a writer is detected, workers immediately release shared_lock and yield, waiting on
//      writers_done_cv until structural mutations complete before resuming.
//      To prevent unbounded retry latency during continuous graph mutations, a job yields at most
//      3 times (or 100 ms from first attempt); subsequent execution runs without yielding to completion.
//      Worst-case writer wait time is bounded by the duration of a single uninterrupted search.
// 2. paths_mutex (std::mutex) protects the paths container (paths map, highest_path_id,
//    and TTL operations).
//    - Workers NEVER touch paths directly; completed routes pass as node ID vectors to the main thread.
//    - All path registration, lookup, and deletion happen under paths_mutex on the main server thread.

namespace Container
{
	std::shared_mutex& GetGraphMutex();
	std::mutex& GetPathsMutex();

	void IncrementPendingWriters();
	void DecrementPendingWriters();
	bool HasPendingWriters();
	void WaitForWritersDone(std::function<bool()> stop_condition = nullptr);

	class GraphWriterLock
	{
	public:
		explicit GraphWriterLock(std::shared_mutex& mtx);
		~GraphWriterLock();

		GraphWriterLock(const GraphWriterLock&) = delete;
		GraphWriterLock& operator=(const GraphWriterLock&) = delete;

	private:
		std::shared_mutex& mtx_;
	};

	namespace Nodes
	{
		int AllocateID();
		bool Add(int id, float x, float y, float z, PathType type = PATH_TYPE_VEHICLE, uint32_t flags = NODE_FLAG_NONE);
		Node* Find(int id);
		Node* FindIncludingPending(int id);
		Node* FindClosest(float x, float y, float z, PathType type = PATH_TYPE_VEHICLE, int ignored_id = -1);
		const std::unordered_map<int, Node*>& GetAll();
		int GetHighestID(PathType type = static_cast<PathType>(-1));
		void SetForDeletion(Node* node);
		bool TryDelete(Node* node);

		void SetDangerWeight(int id, float weight);
		float GetDangerWeight(int id);
		void SetAreaDangerWeight(float minx, float miny, float maxx, float maxy, float weight);
		void ResetDangerWeights();

		Node* GetRandomNode(PathType type, float min_z, float max_z);
		Node* GetRandomNodeInArea(float min_x, float min_y, float max_x, float max_y, PathType type, float min_z, float max_z);
		void RebuildTypeIndices();
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
		float GetMinTrafficMultiplier();
	}
	
	namespace Paths
	{
		Path* Register(const std::vector<Node*>& nodes, float length, AMX* owner);
		int Add(Path* path);
		void Delete(Path* path);
		Path* Find(int id);
		Path* FindLocked(int id);
		std::unordered_map<int, Path*> GetAll();
		int GetHighestID();
		void ProcessTick();
		void DestroyByOwner(AMX* amx);
		void RegisterExpiry(int id, std::chrono::steady_clock::time_point expiry_time);
	}
};
