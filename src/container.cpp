#include <shared_mutex>
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>
#include <cstdint>

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
		int highest_node_id;
		int highest_node_id_by_type[3] = { -1, -1, -1 };

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

		inline bool IsNodeEligible(const Node* node, const float search_z, const PathType type)
		{
			const float nz = node->getZ();
			if (search_z < 500.0f)
			{
				if (nz >= 500.0f)
				{
					return false;
				}
				if ((type == PATH_TYPE_VEHICLE || type == PATH_TYPE_PEDESTRIAN) && search_z > 1.5f && nz <= 1.0f)
				{
					return false;
				}
			}
			else
			{
				if (nz < 500.0f)
				{
					return false;
				}
			}
			return true;
		}

		std::shared_mutex lock;
	}


	void LockExclusive()
	{
		lock.lock();
	}


	void UnlockExclusive()
	{
		lock.unlock();
	}


	void LockShared()
	{
		lock.lock_shared();
	}


	void UnlockShared()
	{
		lock.unlock_shared();
	}


	namespace Nodes
	{
		int AllocateID()
		{
			return ++highest_node_id;
		}

		bool Add(const int id, const float x, const float y, const float z, const PathType type, const uint32_t flags)
		{
			if (Find(id) != nullptr)
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
				if (id > highest_node_id_by_type[type_idx])
				{
					highest_node_id_by_type[type_idx] = id;
				}
			}

			return true;
		}

		Node* Find(const int id)
		{
			const auto node = nodes.find(id);

			if (node == nodes.end())
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
								if (node->getID() == ignored_id || node->isSetForDeletion() || node->getType() != type || !IsNodeEligible(node, z, type))
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
					if (node->getID() == ignored_id || node->isSetForDeletion() || node->getType() != type || !IsNodeEligible(node, z, type))
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

			for (auto connection : node->getConnections())
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

			delete node;
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
			for (auto& pair : nodes)
			{
				if (pair.second != nullptr)
				{
					pair.second->setDangerWeight(0.0f);
				}
			}
		}
	}


	namespace Connections
	{
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

			return id;
		}

		void Delete(Connection* connection)
		{
			connection->getSource()->removeConnection(connection);
			connection->getTarget()->removeConnection(connection);

			connections.erase(connection->getId());

			delete connection;
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

		bool FindClosestRoadPoint(const float x, const float y, const float z, float& out_x, float& out_y, float& out_z, float& out_angle, float& out_road_width, int& out_conn_id, const PathType type)
		{
			Node* closest_node = Nodes::FindClosest(x, y, z, type, -1);
			if (closest_node == nullptr)
			{
				return false;
			}

			std::vector<Connection*> candidate_conns = closest_node->getConnections();
			for (auto conn : closest_node->getConnections())
			{
				Node* neighbor = (conn->getSource() == closest_node) ? conn->getTarget() : conn->getSource();
				if (neighbor != nullptr)
				{
					for (auto n_conn : neighbor->getConnections())
					{
						candidate_conns.push_back(n_conn);
					}
				}
			}

			Connection* best_conn = nullptr;
			float best_dist_sq = std::numeric_limits<float>::infinity();
			float best_qx = 0.0f, best_qy = 0.0f, best_qz = 0.0f;

			for (auto conn : candidate_conns)
			{
				Node* a = conn->getSource();
				Node* b = conn->getTarget();

				if (a == nullptr || b == nullptr || a->isSetForDeletion() || b->isSetForDeletion())
				{
					continue;
				}

				if (a->getType() != type && b->getType() != type)
				{
					continue;
				}

				if (!IsNodeEligible(a, z, type) || !IsNodeEligible(b, z, type))
				{
					continue;
				}

				const float ax = a->getX(), ay = a->getY(), az = a->getZ();
				const float bx = b->getX(), by = b->getY(), bz = b->getZ();

				const float vx = bx - ax;
				const float vy = by - ay;
				const float len_sq = vx * vx + vy * vy;

				if (len_sq < 0.0001f)
				{
					continue;
				}

				const float apx = x - ax;
				const float apy = y - ay;
				float t = (apx * vx + apy * vy) / len_sq;

				if (t < 0.0f) t = 0.0f;
				else if (t > 1.0f) t = 1.0f;

				const float qx = ax + t * vx;
				const float qy = ay + t * vy;
				const float qz = az + t * (bz - az);

				const float dx = x - qx;
				const float dy = y - qy;
				const float dz = z - qz;
				const float dist_sq = dx * dx + dy * dy + dz * dz;

				if (dist_sq < best_dist_sq)
				{
					best_dist_sq = dist_sq;
					best_conn = conn;
					best_qx = qx;
					best_qy = qy;
					best_qz = qz;
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
					conn->setTrafficMultiplier(multiplier);
				}
			}
		}

		void ResetTrafficMultipliers()
		{
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
	}


	namespace Paths
	{
		int Add(Path* path)
		{
			const auto id = highest_path_id++;

			paths[id] = path;
			path->setID(id);

			return id;
		}

		void Delete(Path* path)
		{
			for (auto node : path->getNodes())
			{
				node->removeFromPath(path);

				if (node->isSetForDeletion())
				{
					Nodes::TryDelete(node);
				}
			}

			paths.erase(path->getID());

			delete path;
		}

		Path* Find(const int id)
		{
			const auto path = paths.find(id);

			if (path == paths.end())
			{
				return nullptr;
			}

			return path->second;
		}

		std::unordered_map<int, Path*> GetAll()
		{
			return paths;
		}

		int GetHighestID()
		{
			return highest_path_id;
		}

		void ProcessTick()
		{
			std::vector<Path*> expired_paths;
			for (auto& pair : paths)
			{
				if (pair.second != nullptr && pair.second->isExpired())
				{
					expired_paths.push_back(pair.second);
				}
			}

			for (auto path : expired_paths)
			{
				Delete(path);
			}
		}
	}
}
