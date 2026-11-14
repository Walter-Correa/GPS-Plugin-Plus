#include <shared_mutex>
#include <cmath>
#include <limits>
#include <vector>
#include <queue>
#include <random>
#include <algorithm>
#include <cstdint>
#include <unordered_set>
#include <condition_variable>
#include <chrono>

#include "container.h"
#include "connection.h"
#include "node.h"


namespace Container
{
	namespace
	{
		std::unordered_map<int, Connection*> connections;
		int highest_connection_id;

		std::unordered_map<int, Node*> nodes;
		std::vector<Node*> nodes_by_type[3];
		int highest_node_id;
		int highest_node_id_by_type[3] = { -1, -1, -1 };

		std::mutex comp_mutex;
		bool comp_dirty[4] = { true, true, true, true };
		std::unordered_map<int, int> comp_ids[4];

		constexpr float GRID_CELL_SIZE = 100.0f;

		inline int64_t GetGridKey(const float x, const float y)
		{
			const auto cx = static_cast<int>(std::floor(x / GRID_CELL_SIZE));
			const auto cy = static_cast<int>(std::floor(y / GRID_CELL_SIZE));
			return (static_cast<int64_t>(cx) << 32) | static_cast<uint32_t>(cy);
		}

		std::unordered_map<int64_t, std::vector<Node*>> spatial_grid;

		std::unordered_map<int, Path*> paths;
		int highest_path_id;

		// San Andreas interior altitude threshold. Standard GTA:SA exterior map is below 500m Z,
		// while interior world spaces (e.g. casinos, interiors) are placed at Z >= 500m.
		constexpr float INTERIOR_ALTITUDE_THRESHOLD = 500.0f;

		// Water / sea level threshold. Nodes at Z <= 1.0f are generally water level / boat nodes.
		// When searching for ground vehicles/pedestrians at normal terrain (Z > 1.5f), exclude submerged / sea nodes.
		constexpr float GROUND_SEARCH_MIN_Z = 1.5f;
		constexpr float WATER_SURFACE_MAX_Z = 1.0f;

		inline bool IsNodeEligible(const Node* node, const float search_z, const PathType type)
		{
			const float nz = node->getZ();
			if (search_z < INTERIOR_ALTITUDE_THRESHOLD)
			{
				if (nz >= INTERIOR_ALTITUDE_THRESHOLD)
				{
					return false;
				}
				const PathType eff_type = (type == PATH_TYPE_ANY) ? node->getType() : type;
				if ((eff_type == PATH_TYPE_VEHICLE || eff_type == PATH_TYPE_PEDESTRIAN) && search_z > GROUND_SEARCH_MIN_Z && nz <= WATER_SURFACE_MAX_Z)
				{
					return false;
				}
			}
			else
			{
				if (nz < INTERIOR_ALTITUDE_THRESHOLD)
				{
					return false;
				}
			}
			return true;
		}

		std::shared_mutex graph_mutex;
		std::mutex paths_mutex;
		std::atomic<int> pending_writers{0};
		std::condition_variable writers_done_cv;
		std::mutex writers_done_mutex;
	}

	std::shared_mutex& GetGraphMutex()
	{
		return graph_mutex;
	}

	void IncrementPendingWriters()
	{
		pending_writers.fetch_add(1, std::memory_order_release);
	}

	void DecrementPendingWriters()
	{
		pending_writers.fetch_sub(1, std::memory_order_release);
	}

	bool HasPendingWriters()
	{
		return pending_writers.load(std::memory_order_acquire) > 0;
	}

	void WaitForWritersDone(std::function<bool()> stop_condition)
	{
		std::unique_lock<std::mutex> lock(writers_done_mutex);
		writers_done_cv.wait_for(lock, std::chrono::milliseconds(20), [&]() {
			return !HasPendingWriters() || (stop_condition && stop_condition());
		});
	}

	GraphWriterLock::GraphWriterLock(std::shared_mutex& mtx)
		: mtx_(mtx)
	{
		IncrementPendingWriters();
		mtx_.lock();
	}

	GraphWriterLock::~GraphWriterLock()
	{
		mtx_.unlock();
		DecrementPendingWriters();
		{
			std::lock_guard<std::mutex> lock(writers_done_mutex);
			writers_done_cv.notify_all();
		}
	}


	namespace Nodes
	{
		int AllocateID()
		{
			return ++highest_node_id;
		}

		bool Add(const int id, const float x, const float y, const float z, const PathType type, const uint32_t flags)
		{
			if (FindIncludingPending(id) != nullptr)
			{
				return false;
			}

			auto new_node = new Node(id, x, y, z, type, flags);
			nodes[id] = new_node;
			spatial_grid[GetGridKey(x, y)].push_back(new_node);

			if (id > highest_node_id)
			{
				highest_node_id = id;
			}

			const int type_idx = static_cast<int>(type);
			if (type_idx >= 0 && type_idx < 3)
			{
				nodes_by_type[type_idx].push_back(new_node);
				if (id > highest_node_id_by_type[type_idx])
				{
					highest_node_id_by_type[type_idx] = id;
				}
			}

			MarkComponentsDirty();
			return true;
		}

		Node* Find(const int id)
		{
			const auto node = nodes.find(id);

			if (node == nodes.end() || node->second == nullptr || node->second->isSetForDeletion())
			{
				return nullptr;
			}

			return node->second;
		}

		Node* FindIncludingPending(const int id)
		{
			const auto node = nodes.find(id);

			if (node == nodes.end() || node->second == nullptr)
			{
				return nullptr;
			}

			return node->second;
		}

		Node* FindClosest(const float x, const float y, const float z, const PathType type, const int ignored_id)
		{
			const auto cx = static_cast<int>(std::floor(x / GRID_CELL_SIZE));
			const auto cy = static_cast<int>(std::floor(y / GRID_CELL_SIZE));

			Node* best_node = nullptr;
			float best_dist_sq = std::numeric_limits<float>::infinity();

			const int max_rings = 20;
			for (int ring = 0; ring <= max_rings; ++ring)
			{
				if (ring > 0 && best_node != nullptr)
				{
					const float ring_min_dist = (ring - 1) * GRID_CELL_SIZE;
					if (best_dist_sq <= ring_min_dist * ring_min_dist)
					{
						break;
					}
				}

				for (int dx = -ring; dx <= ring; ++dx)
				{
					for (int dy = -ring; dy <= ring; ++dy)
					{
						if (ring > 0 && std::abs(dx) < ring && std::abs(dy) < ring)
						{
							continue;
						}

						const int64_t key = (static_cast<int64_t>(cx + dx) << 32) | static_cast<uint32_t>(cy + dy);
						const auto it = spatial_grid.find(key);
						if (it != spatial_grid.end())
						{
							for (Node* node : it->second)
							{
								if (node->getID() == ignored_id || node->isSetForDeletion() || (type != PATH_TYPE_ANY && node->getType() != type) || !IsNodeEligible(node, z, type))
								{
									continue;
								}

								const float nx = node->getX() - x;
								const float ny = node->getY() - y;
								const float nz = node->getZ() - z;
								const float d_sq = nx * nx + ny * ny + nz * nz;

								if (d_sq < best_dist_sq)
								{
									best_dist_sq = d_sq;
									best_node = node;
								}
							}
						}
					}
				}
			}

			if (best_node == nullptr)
			{
				for (const auto& pair : nodes)
				{
					Node* node = pair.second;
					if (node->getID() == ignored_id || node->isSetForDeletion() || (type != PATH_TYPE_ANY && node->getType() != type) || !IsNodeEligible(node, z, type))
					{
						continue;
					}

					const float nx = node->getX() - x;
					const float ny = node->getY() - y;
					const float nz = node->getZ() - z;
					const float d_sq = nx * nx + ny * ny + nz * nz;

					if (d_sq < best_dist_sq)
					{
						best_dist_sq = d_sq;
						best_node = node;
					}
				}
			}

			return best_node;
		}

		Node* FindClosestWithAngle(const float x, const float y, const float z, const float angle, const float max_dist, const PathType type)
		{
			const float radius = (max_dist > 0.0f) ? max_dist : 0.0f;
			if (radius > 0.0f)
			{
				const float max_dist_sq = radius * radius;
				const int cell_radius = std::max(1, static_cast<int>(std::ceil(radius / GRID_CELL_SIZE)));
				const int cx = static_cast<int>(std::floor(x / GRID_CELL_SIZE));
				const int cy = static_cast<int>(std::floor(y / GRID_CELL_SIZE));

				auto angle_delta = [](float a, float b)
				{
					float diff = std::abs(a - b);
					while (diff > 180.0f) diff = std::abs(diff - 360.0f);
					return diff;
				};

				Node* best_aligned = nullptr;
				float best_dist_sq = std::numeric_limits<float>::infinity();

				for (int dx = -cell_radius; dx <= cell_radius; ++dx)
				{
					for (int dy = -cell_radius; dy <= cell_radius; ++dy)
					{
						const int64_t key = (static_cast<int64_t>(cx + dx) << 32) | static_cast<uint32_t>(cy + dy);
						const auto it = spatial_grid.find(key);
						if (it == spatial_grid.end()) continue;

						for (Node* node : it->second)
						{
							if (node == nullptr || node->isSetForDeletion() || node->isBlocked() || (type != PATH_TYPE_ANY && node->getType() != type) || !IsNodeEligible(node, z, type))
							{
								continue;
							}

							const float nx = node->getX() - x;
							const float ny = node->getY() - y;
							const float nz = node->getZ() - z;
							const float d_sq = nx * nx + ny * ny + nz * nz;
							if (d_sq > max_dist_sq || d_sq >= best_dist_sq)
							{
								continue;
							}

							bool has_aligned_legal_out = false;
							for (auto conn : node->getConnections())
							{
								if (conn == nullptr || conn->isBlocked() || conn->isWrongWay() || conn->getSource() != node)
								{
									continue;
								}
								Node* tgt = conn->getTarget();
								if (tgt == nullptr || tgt == node || tgt->isSetForDeletion() || tgt->isBlocked() || (type != PATH_TYPE_ANY && tgt->getType() != type))
								{
									continue;
								}
								if (angle_delta(angle, node->getAngleFromNode(tgt, true)) <= 90.0f)
								{
									has_aligned_legal_out = true;
									break;
								}
							}

							if (has_aligned_legal_out)
							{
								best_dist_sq = d_sq;
								best_aligned = node;
							}
						}
					}
				}

				if (best_aligned != nullptr)
				{
					return best_aligned;
				}
			}

			return FindClosest(x, y, z, type, -1);
		}

		const std::unordered_map<int, Node*>& GetAll()
		{
			return nodes;
		}

		int GetHighestID(const PathType type)
		{
			const int type_idx = static_cast<int>(type);
			if (type_idx >= 0 && type_idx < 3)
			{
				return highest_node_id_by_type[type_idx];
			}

			return highest_node_id;
		}

		void SetForDeletion(Node* node)
		{
			if (!TryDelete(node))
			{
				node->setForDeletion();
			}
		}

		bool TryDelete(Node* node)
		{
			if (node->getPathCount() > 0)
			{
				return false;
			}

			const auto conns = node->getConnections();
			for (auto connection : conns)
			{
				Connections::Delete(connection);
			}

			const auto key = GetGridKey(node->getX(), node->getY());
			auto it = spatial_grid.find(key);
			if (it != spatial_grid.end())
			{
				auto& cell_nodes = it->second;
				cell_nodes.erase(std::remove(cell_nodes.begin(), cell_nodes.end(), node), cell_nodes.end());
				if (cell_nodes.empty())
				{
					spatial_grid.erase(it);
				}
			}

			nodes.erase(node->getID());

			const int type_idx = static_cast<int>(node->getType());
			if (type_idx >= 0 && type_idx < 3)
			{
				auto& vec = nodes_by_type[type_idx];
				vec.erase(std::remove(vec.begin(), vec.end(), node), vec.end());
			}

			delete node;
			MarkComponentsDirty();
			return true;
		}

		void SetDangerWeight(const int id, const float weight)
		{
			auto node = Find(id);
			if (node != nullptr)
			{
				node->setDangerWeight(weight);
			}
		}

		float GetDangerWeight(const int id)
		{
			auto node = Find(id);
			if (node != nullptr)
			{
				return node->getDangerWeight();
			}
			return 0.0f;
		}

		void SetAreaDangerWeight(const float minx, const float miny, const float maxx, const float maxy, const float weight)
		{
			std::shared_lock<std::shared_mutex> lock(graph_mutex);
			const float x1 = std::min(minx, maxx);
			const float x2 = std::max(minx, maxx);
			const float y1 = std::min(miny, maxy);
			const float y2 = std::max(miny, maxy);

			for (auto& pair : nodes)
			{
				auto node = pair.second;
				if (node == nullptr) continue;
				const float nx = node->getX();
				const float ny = node->getY();
				if (nx >= x1 && nx <= x2 && ny >= y1 && ny <= y2)
				{
					node->setDangerWeight(weight);
				}
			}
		}

		void ResetDangerWeights()
		{
			std::shared_lock<std::shared_mutex> lock(graph_mutex);
			for (auto& pair : nodes)
			{
				if (pair.second != nullptr)
				{
					pair.second->setDangerWeight(0.0f);
				}
			}
		}

		static int ComponentSlotFromType(const PathType type)
		{
			const int idx = static_cast<int>(type);
			return (idx >= 0 && idx < 3) ? idx : 3;
		}

		static void BuildComponentsLocked(const int slot, const PathType type)
		{
			comp_ids[slot].clear();
			comp_ids[slot].reserve(nodes.size());

			int next_comp = 1;
			std::queue<Node*> bfs_queue;

			for (const auto& pair : nodes)
			{
				Node* start = pair.second;
				if (start == nullptr || start->isSetForDeletion() || start->isBlocked())
				{
					continue;
				}
				if (type != PATH_TYPE_ANY && start->getType() != type)
				{
					continue;
				}
				if (comp_ids[slot].find(start->getID()) != comp_ids[slot].end())
				{
					continue;
				}

				const int comp_id = next_comp++;
				comp_ids[slot][start->getID()] = comp_id;
				bfs_queue.push(start);

				while (!bfs_queue.empty())
				{
					Node* curr = bfs_queue.front();
					bfs_queue.pop();

					for (Connection* conn : curr->getConnections())
					{
						if (conn == nullptr || conn->isBlocked())
						{
							continue;
						}
						Node* other = (conn->getSource() == curr) ? conn->getTarget() : conn->getSource();
						if (other == nullptr || other->isSetForDeletion() || other->isBlocked())
						{
							continue;
						}
						if (type != PATH_TYPE_ANY && other->getType() != type)
						{
							continue;
						}
						if (comp_ids[slot].emplace(other->getID(), comp_id).second)
						{
							bfs_queue.push(other);
						}
					}
				}
			}

			comp_dirty[slot] = false;
		}

		void MarkComponentsDirty()
		{
			std::lock_guard<std::mutex> lock(comp_mutex);
			for (bool& dirty : comp_dirty)
			{
				dirty = true;
			}
		}

		int GetConnectedComponent(Node* node, const PathType type)
		{
			if (node == nullptr || node->isSetForDeletion() || node->isBlocked())
			{
				return -1;
			}
			const int slot = ComponentSlotFromType(type);
			std::lock_guard<std::mutex> lock(comp_mutex);
			if (comp_dirty[slot])
			{
				BuildComponentsLocked(slot, type);
			}
			const auto it = comp_ids[slot].find(node->getID());
			return (it != comp_ids[slot].end()) ? it->second : -1;
		}

		void RebuildTypeIndices()
		{
			for (int i = 0; i < 3; ++i)
			{
				nodes_by_type[i].clear();
			}
			for (const auto& pair : nodes)
			{
				Node* n = pair.second;
				if (n == nullptr || n->isSetForDeletion()) continue;
				const int t = static_cast<int>(n->getType());
				if (t >= 0 && t < 3)
				{
					nodes_by_type[t].push_back(n);
				}
			}
			MarkComponentsDirty();
		}

		Node* GetRandomNode(PathType type, float min_z, float max_z)
		{
			std::shared_lock<std::shared_mutex> lock(graph_mutex);
			static thread_local std::mt19937 rng{ std::random_device{}() };

			if (type == PATH_TYPE_ANY)
			{
				const size_t total = nodes_by_type[0].size() + nodes_by_type[1].size() + nodes_by_type[2].size();
				if (total == 0) return nullptr;

				std::uniform_int_distribution<size_t> dist(0, total - 1);
				for (int i = 0; i < 30; ++i)
				{
					size_t idx = dist(rng);
					Node* n = nullptr;
					if (idx < nodes_by_type[0].size())
					{
						n = nodes_by_type[0][idx];
					}
					else if (idx < nodes_by_type[0].size() + nodes_by_type[1].size())
					{
						n = nodes_by_type[1][idx - nodes_by_type[0].size()];
					}
					else
					{
						n = nodes_by_type[2][idx - nodes_by_type[0].size() - nodes_by_type[1].size()];
					}

					if (n != nullptr && !n->isSetForDeletion())
					{
						const float z = n->getZ();
						if (z >= min_z && z <= max_z)
						{
							return n;
						}
					}
				}

				Node* chosen = nullptr;
				size_t count = 0;
				for (int t = 0; t < 3; ++t)
				{
					for (Node* n : nodes_by_type[t])
					{
						if (n != nullptr && !n->isSetForDeletion())
						{
							const float z = n->getZ();
							if (z >= min_z && z <= max_z)
							{
								++count;
								std::uniform_int_distribution<size_t> pick(1, count);
								if (pick(rng) == 1)
								{
									chosen = n;
								}
							}
						}
					}
				}
				return chosen;
			}

			const int type_idx = static_cast<int>(type);
			if (type_idx < 0 || type_idx >= 3) return nullptr;

			const auto& pool = nodes_by_type[type_idx];
			if (pool.empty()) return nullptr;

			// Fast trial: try up to 30 random draws in O(1)
			std::uniform_int_distribution<size_t> dist(0, pool.size() - 1);
			for (int i = 0; i < 30; ++i)
			{
				Node* n = pool[dist(rng)];
				if (n != nullptr && !n->isSetForDeletion())
				{
					const float z = n->getZ();
					if (z >= min_z && z <= max_z)
					{
						return n;
					}
				}
			}

			// Fallback: reservoir sampling over the type pool without allocations
			Node* chosen = nullptr;
			size_t count = 0;
			for (Node* n : pool)
			{
				if (n != nullptr && !n->isSetForDeletion())
				{
					const float z = n->getZ();
					if (z >= min_z && z <= max_z)
					{
						++count;
						std::uniform_int_distribution<size_t> pick(1, count);
						if (pick(rng) == 1)
						{
							chosen = n;
						}
					}
				}
			}
			return chosen;
		}

		Node* GetRandomNodeInArea(float min_x, float min_y, float max_x, float max_y, PathType type, float min_z, float max_z)
		{
			std::shared_lock<std::shared_mutex> lock(graph_mutex);
			const float x1 = std::min(min_x, max_x);
			const float x2 = std::max(min_x, max_x);
			const float y1 = std::min(min_y, max_y);
			const float y2 = std::max(min_y, max_y);

			const int cx_min = static_cast<int>(std::floor(x1 / GRID_CELL_SIZE));
			const int cx_max = static_cast<int>(std::floor(x2 / GRID_CELL_SIZE));
			const int cy_min = static_cast<int>(std::floor(y1 / GRID_CELL_SIZE));
			const int cy_max = static_cast<int>(std::floor(y2 / GRID_CELL_SIZE));

			static thread_local std::mt19937 rng{ std::random_device{}() };
			Node* chosen = nullptr;
			size_t count = 0;

			for (int cx = cx_min; cx <= cx_max; ++cx)
			{
				for (int cy = cy_min; cy <= cy_max; ++cy)
				{
					const int64_t key = (static_cast<int64_t>(cx) << 32) | static_cast<uint32_t>(cy);
					auto it = spatial_grid.find(key);
					if (it == spatial_grid.end()) continue;

					for (Node* n : it->second)
					{
						if (n == nullptr || n->isSetForDeletion() || (type != PATH_TYPE_ANY && n->getType() != type)) continue;
						const float x = n->getX();
						const float y = n->getY();
						const float z = n->getZ();
						if (x >= x1 && x <= x2 && y >= y1 && y <= y2 && z >= min_z && z <= max_z)
						{
							++count;
							std::uniform_int_distribution<size_t> pick(1, count);
							if (pick(rng) == 1)
							{
								chosen = n;
							}
						}
					}
				}
			}

			return chosen;
		}
	}


	namespace Connections
	{
		std::atomic<float> min_traffic_multiplier_{ 1.0f };

		float GetMinTrafficMultiplier()
		{
			return min_traffic_multiplier_.load(std::memory_order_relaxed);
		}

		bool Exists(const int sourceID, const int targetID)
		{
			const auto source = Nodes::Find(sourceID);
			const auto target = Nodes::Find(targetID);
			if (source == nullptr || target == nullptr)
			{
				return false;
			}
			for (const auto conn : source->getConnections())
			{
				if (conn != nullptr && conn->getSource() == source && conn->getTarget() == target)
				{
					return true;
				}
			}
			return false;
		}

		int Add(const int sourceID, const int targetID, const float road_width, const int left_lanes, const int right_lanes, const bool one_way)
		{
			if (sourceID == targetID)
			{
				return -1;
			}

			auto source = Nodes::Find(sourceID);

			if (source == nullptr)
			{
				return -1;
			}

			const auto target = Nodes::Find(targetID);

			if (target == nullptr)
			{
				return -1;
			}

			const auto id = highest_connection_id++;
			const auto connection = new Connection(id, source, target, source->getDistanceFromNode(target), road_width, left_lanes, right_lanes, one_way);

			connections[id] = connection;

			source->addConnection(connection);
			target->addConnection(connection);

			Nodes::MarkComponentsDirty();
			return id;
		}

		void Delete(Connection* connection)
		{
			connection->getSource()->removeConnection(connection);
			connection->getTarget()->removeConnection(connection);

			connections.erase(connection->getId());

			delete connection;
			Nodes::MarkComponentsDirty();
		}

		Connection* Find(const int id)
		{
			const auto connection = connections.find(id);

			if (connection == connections.end())
			{
				return nullptr;
			}

			return connection->second;
		}

		static bool ProjectPointOnConnection(const float x, const float y, const Connection* conn, float& out_qx, float& out_qy, float& out_qz, float& out_dist_sq)
		{
			if (conn == nullptr)
			{
				return false;
			}
			const Node* a = conn->getSource();
			const Node* b = conn->getTarget();
			if (a == nullptr || b == nullptr)
			{
				return false;
			}

			const float ax = a->getX(), ay = a->getY(), az = a->getZ();
			const float bx = b->getX(), by = b->getY(), bz = b->getZ();
			const float vx = bx - ax;
			const float vy = by - ay;
			const float len_sq = vx * vx + vy * vy;
			if (len_sq < 0.0001f)
			{
				return false;
			}

			const float apx = x - ax;
			const float apy = y - ay;
			float t = (apx * vx + apy * vy) / len_sq;
			if (t < 0.0f) t = 0.0f;
			else if (t > 1.0f) t = 1.0f;

			out_qx = ax + t * vx;
			out_qy = ay + t * vy;
			out_qz = az + t * (bz - az);

			const float dx = x - out_qx;
			const float dy = y - out_qy;
			out_dist_sq = dx * dx + dy * dy;
			return true;
		}

		static void CollectConnectionsInRadius(const float x, const float y, const float z, const float radius, const PathType type, std::vector<Connection*>& out_conns)
		{
			const float r_sq = radius * radius;
			const int cell_radius = std::max(1, static_cast<int>(std::ceil(radius / GRID_CELL_SIZE)));
			const int cx = static_cast<int>(std::floor(x / GRID_CELL_SIZE));
			const int cy = static_cast<int>(std::floor(y / GRID_CELL_SIZE));

			std::unordered_set<Connection*> seen_conns;
			for (int dx = -cell_radius; dx <= cell_radius; ++dx)
			{
				for (int dy = -cell_radius; dy <= cell_radius; ++dy)
				{
					const int64_t key = (static_cast<int64_t>(cx + dx) << 32) | static_cast<uint32_t>(cy + dy);
					const auto it = spatial_grid.find(key);
					if (it == spatial_grid.end()) continue;

					for (Node* node : it->second)
					{
						if (node == nullptr || node->isSetForDeletion() || (type != PATH_TYPE_ANY && node->getType() != type) || !IsNodeEligible(node, z, type))
						{
							continue;
						}
						const float nx = node->getX() - x;
						const float ny = node->getY() - y;
						if ((nx * nx + ny * ny) > r_sq)
						{
							continue;
						}
						for (auto conn : node->getConnections())
						{
							if (conn != nullptr && seen_conns.insert(conn).second)
							{
								out_conns.push_back(conn);
							}
						}
					}
				}
			}
		}

		bool FindClosestRoadPoint(const float x, const float y, const float z, float& out_x, float& out_y, float& out_z, float& out_angle, float& out_road_width, int& out_conn_id, const PathType type)
		{
			auto evaluate_candidates = [&](const std::vector<Connection*>& candidates, Connection*& best_conn, float& best_dist_sq, float& best_qx, float& best_qy, float& best_qz)
			{
				for (auto conn : candidates)
				{
					if (conn == nullptr) continue;
					Node* a = conn->getSource();
					Node* b = conn->getTarget();

					if (a == nullptr || b == nullptr || a->isSetForDeletion() || b->isSetForDeletion())
					{
						continue;
					}

					if (type != PATH_TYPE_ANY && a->getType() != type && b->getType() != type)
					{
						continue;
					}

					if (!IsNodeEligible(a, z, type) || !IsNodeEligible(b, z, type))
					{
						continue;
					}

					float qx = 0.0f, qy = 0.0f, qz = 0.0f, dist_sq = 0.0f;
					if (!ProjectPointOnConnection(x, y, conn, qx, qy, qz, dist_sq))
					{
						continue;
					}

					const bool better = (dist_sq + 0.0001f < best_dist_sq) ||
						(std::abs(dist_sq - best_dist_sq) <= 0.0001f && best_conn != nullptr && best_conn->isWrongWay() && !conn->isWrongWay());
					if (better)
					{
						best_dist_sq = dist_sq;
						best_conn = conn;
						best_qx = qx;
						best_qy = qy;
						best_qz = qz;
					}
				}
			};

			Connection* best_conn = nullptr;
			float best_dist_sq = std::numeric_limits<float>::infinity();
			float best_qx = 0.0f, best_qy = 0.0f, best_qz = 0.0f;

			const float search_radii[] = { 30.0f, 80.0f };
			for (float radius : search_radii)
			{
				std::vector<Connection*> candidate_conns;
				CollectConnectionsInRadius(x, y, z, radius, type, candidate_conns);
				evaluate_candidates(candidate_conns, best_conn, best_dist_sq, best_qx, best_qy, best_qz);
				if (best_conn != nullptr && best_dist_sq <= 10.0f * 10.0f)
				{
					break;
				}
			}

			if (best_conn == nullptr)
			{
				Node* closest_node = Nodes::FindClosest(x, y, z, type, -1);
				if (closest_node != nullptr)
				{
					std::vector<Connection*> fallback_conns = closest_node->getConnections();
					for (auto conn : closest_node->getConnections())
					{
						Node* neighbor = (conn->getSource() == closest_node) ? conn->getTarget() : conn->getSource();
						if (neighbor != nullptr)
						{
							for (auto n_conn : neighbor->getConnections())
							{
								fallback_conns.push_back(n_conn);
							}
						}
					}
					evaluate_candidates(fallback_conns, best_conn, best_dist_sq, best_qx, best_qy, best_qz);
				}
			}

			if (best_conn == nullptr)
			{
				return false;
			}

			out_x = best_qx;
			out_y = best_qy;
			out_z = best_qz;
			out_angle = best_conn->getSource()->getAngleFromNode(best_conn->getTarget(), true);
			out_road_width = best_conn->getRoadWidth();
			out_conn_id = best_conn->getId();
			return true;
		}

		std::vector<RoadCandidate> FindRoadCandidates(const float x, const float y, const float z, const float max_radius, const PathType type, const int required_component)
		{
			std::vector<RoadCandidate> result;
			if (max_radius <= 0.0f)
			{
				return result;
			}

			// Search slightly beyond max_radius for endpoint nodes so segments whose interiors pass inside max_radius are included.
			const float node_search_radius = max_radius + 80.0f;
			std::vector<Connection*> raw_conns;
			CollectConnectionsInRadius(x, y, z, node_search_radius, type, raw_conns);

			const float max_r_sq = max_radius * max_radius;
			for (Connection* conn : raw_conns)
			{
				if (conn == nullptr || conn->isBlocked())
				{
					continue;
				}
				Node* a = conn->getSource();
				Node* b = conn->getTarget();
				if (a == nullptr || b == nullptr || a->isSetForDeletion() || b->isSetForDeletion() || a->isBlocked() || b->isBlocked())
				{
					continue;
				}
				if (type != PATH_TYPE_ANY && (a->getType() != type || b->getType() != type))
				{
					continue;
				}
				if (!IsNodeEligible(a, z, type) || !IsNodeEligible(b, z, type))
				{
					continue;
				}
				if (required_component != -1 && Nodes::GetConnectedComponent(a, type) != required_component)
				{
					continue;
				}

				float qx = 0.0f, qy = 0.0f, qz = 0.0f, dist_sq = 0.0f;
				if (!ProjectPointOnConnection(x, y, conn, qx, qy, qz, dist_sq) || dist_sq > max_r_sq)
				{
					continue;
				}

				RoadCandidate cand;
				cand.conn = conn;
				cand.qx = qx;
				cand.qy = qy;
				cand.qz = qz;
				cand.dist2d_sq = dist_sq;
				result.push_back(cand);
			}

			std::sort(result.begin(), result.end(), [z](const RoadCandidate& lhs, const RoadCandidate& rhs)
			{
				const bool lhs_z_ok = std::abs(lhs.qz - z) <= 15.0f;
				const bool rhs_z_ok = std::abs(rhs.qz - z) <= 15.0f;
				if (lhs_z_ok != rhs_z_ok)
				{
					return lhs_z_ok;
				}
				if (lhs.dist2d_sq != rhs.dist2d_sq)
				{
					return lhs.dist2d_sq < rhs.dist2d_sq;
				}
				return !lhs.conn->isWrongWay() && rhs.conn->isWrongWay();
			});

			return result;
		}

		const std::unordered_map<int, Connection*>& GetAll()
		{
			return connections;
		}

		int GetHighestID()
		{
			return highest_connection_id;
		}

		void SetTrafficMultiplier(const int id, const float multiplier)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				conn->setTrafficMultiplier(multiplier);
				const float clamped = std::max(0.01f, multiplier);
				float current_min = min_traffic_multiplier_.load(std::memory_order_relaxed);
				while (clamped < current_min && !min_traffic_multiplier_.compare_exchange_weak(current_min, clamped, std::memory_order_relaxed))
				{
				}
			}
		}

		float GetTrafficMultiplier(const int id)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				return conn->getTrafficMultiplier();
			}
			return 1.0f;
		}

		void SetAreaTrafficMultiplier(const float minx, const float miny, const float maxx, const float maxy, const float multiplier)
		{
			std::shared_lock<std::shared_mutex> lock(graph_mutex);
			const float x1 = std::min(minx, maxx);
			const float x2 = std::max(minx, maxx);
			const float y1 = std::min(miny, maxy);
			const float y2 = std::max(miny, maxy);

			const float clamped = std::max(0.01f, multiplier);
			float current_min = min_traffic_multiplier_.load(std::memory_order_relaxed);
			while (clamped < current_min && !min_traffic_multiplier_.compare_exchange_weak(current_min, clamped, std::memory_order_relaxed))
			{
			}

			for (auto& pair : connections)
			{
				auto conn = pair.second;
				if (conn == nullptr) continue;
				auto src = conn->getSource();
				auto tgt = conn->getTarget();
				if (src == nullptr || tgt == nullptr) continue;
				const float mx = (src->getX() + tgt->getX()) * 0.5f;
				const float my = (src->getY() + tgt->getY()) * 0.5f;
				if (mx >= x1 && mx <= x2 && my >= y1 && my <= y2)
				{
					conn->setTrafficMultiplier(multiplier);
				}
			}
		}

		void ResetTrafficMultipliers()
		{
			min_traffic_multiplier_.store(1.0f, std::memory_order_relaxed);
			std::shared_lock<std::shared_mutex> lock(graph_mutex);
			for (auto& pair : connections)
			{
				if (pair.second != nullptr)
				{
					pair.second->setTrafficMultiplier(1.0f);
				}
			}
		}

		void SetToll(const int id, const bool has_toll, const int cost)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				conn->setToll(has_toll, cost);
			}
		}

		bool GetToll(const int id, int& out_cost)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				out_cost = conn->getTollCost();
				return conn->hasToll();
			}
			out_cost = 0;
			return false;
		}

		void SetPaved(const int id, const bool paved)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				conn->setPaved(paved);
			}
		}

		bool IsPaved(const int id)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				return conn->isPaved();
			}
			return true;
		}

		RoadClass GetRoadClass(const int id)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				return conn->getRoadClass();
			}
			return ROAD_CLASS_RESIDENTIAL;
		}

		float GetSpeedLimit(const int id)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				return conn->getSpeedLimit();
			}
			return 50.0f;
		}

		void SetSpeedLimit(const int id, const float speed)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				conn->setSpeedLimit(speed);
			}
		}

		float GetMaxHeight(const int id)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				return conn->getMaxHeight();
			}
			return 0.0f;
		}

		void SetMaxHeight(const int id, const float height)
		{
			auto conn = Find(id);
			if (conn != nullptr)
			{
				conn->setMaxHeight(height);
			}
		}

		void SetAreaMaxHeight(const float minx, const float miny, const float maxx, const float maxy, const float height)
		{
			std::shared_lock<std::shared_mutex> lock(graph_mutex);
			const float x1 = std::min(minx, maxx);
			const float x2 = std::max(minx, maxx);
			const float y1 = std::min(miny, maxy);
			const float y2 = std::max(miny, maxy);

			for (auto& pair : connections)
			{
				auto conn = pair.second;
				if (conn == nullptr) continue;
				auto src = conn->getSource();
				auto tgt = conn->getTarget();
				if (src == nullptr || tgt == nullptr) continue;
				const float mx = (src->getX() + tgt->getX()) * 0.5f;
				const float my = (src->getY() + tgt->getY()) * 0.5f;
				if (mx >= x1 && mx <= x2 && my >= y1 && my <= y2)
				{
					conn->setMaxHeight(height);
				}
			}
		}
	}


	namespace Paths
	{
		Path* Register(const std::vector<Node*>& nodes, float length, AMX* owner)
		{
			auto path = new Path(length, owner);
			for (size_t i = 0; i < nodes.size(); ++i)
			{
				if (nodes[i] != nullptr)
				{
					path->addNodeToEnd(nodes[i]);
					nodes[i]->addToPath(path, static_cast<int>(i));
				}
			}
			Add(path);
			return path;
		}

		int Add(Path* path)
		{
			std::lock_guard<std::mutex> lock(paths_mutex);
			const auto id = highest_path_id++;

			paths[id] = path;
			path->setID(id);

			return id;
		}

		void Delete(Path* path)
		{
			if (path == nullptr) return;

			{
				std::lock_guard<std::mutex> lock(paths_mutex);
				paths.erase(path->getID());
			}

			std::unordered_set<Node*> unique_nodes;
			for (auto node : path->getNodes())
			{
				if (node != nullptr)
				{
					unique_nodes.insert(node);
				}
			}

			delete path;

			bool needs_graph_write = false;
			for (auto node : unique_nodes)
			{
				if (node != nullptr && node->isSetForDeletion())
				{
					needs_graph_write = true;
					break;
				}
			}

			if (needs_graph_write)
			{
				GraphWriterLock lock(graph_mutex);
				for (auto node : unique_nodes)
				{
					if (node != nullptr && node->isSetForDeletion())
					{
						Nodes::TryDelete(node);
					}
				}
			}
		}

		Path* Find(const int id)
		{
			std::lock_guard<std::mutex> lock(paths_mutex);
			const auto path = paths.find(id);

			if (path == paths.end())
			{
				return nullptr;
			}

			return path->second;
		}

		std::unordered_map<int, Path*> GetAll()
		{
			std::lock_guard<std::mutex> lock(paths_mutex);
			return paths;
		}

		int GetHighestID()
		{
			std::lock_guard<std::mutex> lock(paths_mutex);
			return highest_path_id;
		}

		struct PathExpiryEntry
		{
			std::chrono::steady_clock::time_point expiry_time;
			int path_id;

			bool operator>(const PathExpiryEntry& other) const
			{
				return expiry_time > other.expiry_time;
			}
		};

		std::priority_queue<PathExpiryEntry, std::vector<PathExpiryEntry>, std::greater<PathExpiryEntry>> expiry_queue;

		void RegisterExpiry(const int id, const std::chrono::steady_clock::time_point expiry_time)
		{
			std::lock_guard<std::mutex> lock(paths_mutex);
			expiry_queue.push({ expiry_time, id });
		}

		void ProcessTick()
		{
			const auto now = std::chrono::steady_clock::now();
			std::vector<Path*> expired_paths;
			{
				std::lock_guard<std::mutex> lock(paths_mutex);
				while (!expiry_queue.empty())
				{
					const auto& top = expiry_queue.top();
					if (top.expiry_time > now)
					{
						break;
					}

					const int id = top.path_id;
					expiry_queue.pop();

					auto it = paths.find(id);
					if (it != paths.end() && it->second != nullptr && it->second->isExpired())
					{
						expired_paths.push_back(it->second);
					}
				}
			}

			for (auto path : expired_paths)
			{
				Delete(path);
			}
		}

		void DestroyByOwner(AMX* amx)
		{
			if (amx == nullptr) return;

			std::vector<Path*> to_destroy;
			{
				std::lock_guard<std::mutex> lock(paths_mutex);
				for (auto& pair : paths)
				{
					if (pair.second != nullptr && pair.second->getOwnerAmx() == amx)
					{
						to_destroy.push_back(pair.second);
					}
				}
			}

			for (auto path : to_destroy)
			{
				Delete(path);
			}
		}
	}
}
