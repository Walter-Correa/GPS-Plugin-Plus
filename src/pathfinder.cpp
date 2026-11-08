#include <cmath>
#include <queue>
#include <exception>
#include <functional>
#include <unordered_set>
#include <algorithm>
#include <limits>

#include "pathfinder.h"
#include "connection.h"
#include "node.h"
#include "amx.h"


namespace Pathfinder
{
	float infinity = std::numeric_limits<float>::infinity();

	struct VisitedNode
	{
		uint32_t generation = 0;
		bool closed = false;
		Node* node = nullptr;
		VisitedNode* previous = nullptr;
		float distance = infinity;
		float target_distance = -1.0f;
		float total_distance = infinity;
	};

	struct VisitedNodeComparator
	{
		inline bool operator()(const VisitedNode* lhs, const VisitedNode* rhs) const noexcept
		{
			return lhs->total_distance > rhs->total_distance;
		}
	};

	thread_local std::vector<VisitedNode> tl_visited_nodes;
	thread_local uint32_t tl_visited_generation = 0;


	bool FindPathNodes(Node* start, Node* target, std::vector<Node*>& out_nodes, float& out_length, const PathType type, const RoutePreference pref, const VehicleCategory cat, const float danger_penalty, const float max_slope_gradient, const bool avoid_tolls, const bool evasive, const float vehicle_height, const bool allow_yield)
	{
		out_nodes.clear();
		out_length = 0.0f;

		if (start == nullptr || target == nullptr || start->isBlocked() || target->isBlocked())
		{
			return false;
		}

		if (start->isSetForDeletion() || target->isSetForDeletion())
		{
			return false;
		}

		if (start == target)
		{
			out_nodes.push_back(start);
			out_length = 0.0f;
			return true;
		}

		const int highest_id = Container::Nodes::GetHighestID();
		const int start_id = start->getID();
		const int target_id = target->getID();
		if (start_id < 0 || start_id > highest_id || target_id < 0 || target_id > highest_id)
		{
			return false;
		}

		if (tl_visited_nodes.size() <= static_cast<size_t>(highest_id))
		{
			tl_visited_nodes.resize(highest_id + 1);
		}

		++tl_visited_generation;
		if (tl_visited_generation == 0)
		{
			std::fill(tl_visited_nodes.begin(), tl_visited_nodes.end(), VisitedNode{});
			tl_visited_generation = 1;
		}

		auto get_node = [&](int id, Node* n) -> VisitedNode* {
			auto& vn = tl_visited_nodes[id];
			if (vn.generation != tl_visited_generation)
			{
				vn.generation = tl_visited_generation;
				vn.closed = false;
				vn.node = n;
				vn.previous = nullptr;
				vn.distance = infinity;
				vn.target_distance = -1.0f;
				vn.total_distance = infinity;
			}
			return &vn;
		};

		// Heuristic scale factor to ensure strict admissibility
		float min_factor = 1.0f;
		if (pref == ROUTE_PREF_HIGHWAYS)
		{
			min_factor *= 0.55f;
		}
		if (evasive || pref == ROUTE_PREF_EVASIVE)
		{
			min_factor *= (0.65f * 0.85f);
		}
		if (cat == VEHICLE_CAT_MOTORCYCLE)
		{
			min_factor *= 0.8f;
		}
		const float min_traffic = Container::Connections::GetMinTrafficMultiplier();
		const float heuristic_scale = min_factor * min_traffic;

		std::priority_queue<VisitedNode*, std::vector<VisitedNode*>, VisitedNodeComparator> queue;
		VisitedNode* solution = nullptr;

		auto first_node = get_node(start_id, start);
		first_node->distance = 0.0f;
		first_node->target_distance = start->getDistanceFromNode(target) * heuristic_scale;
		first_node->total_distance = first_node->target_distance;
		queue.push(first_node);

		int iter_count = 0;
		while (!queue.empty())
		{
			if (allow_yield && (++iter_count & 63) == 0 && Container::HasPendingWriters())
			{
				throw PathfinderYieldException();
			}

			const auto current_node = queue.top();
			queue.pop();

			if (current_node->closed)
			{
				continue;
			}
			current_node->closed = true;

			if (current_node->node == target)
			{
				solution = current_node;
				break;
			}

			for (auto connection : current_node->node->getConnections())
			{
				if (connection->isBlocked())
				{
					continue;
				}

				if (cat == VEHICLE_CAT_HEAVY_TRUCK && connection->getRoadWidth() < 6.0f)
				{
					continue;
				}

				// Height clearance / low bridge restriction
				if (vehicle_height > 0.0f && connection->getMaxHeight() > 0.0f)
				{
					if (vehicle_height > connection->getMaxHeight())
					{
						continue;
					}
				}

				const auto connection_target = connection->getTarget();

				if (connection_target == current_node->node || connection_target->isSetForDeletion() || connection_target->isBlocked())
				{
					continue;
				}

				if (type != PATH_TYPE_ANY && connection_target->getType() != type)
				{
					continue;
				}

				// Evasive routing (tactical anti-dead-end)
				if ((evasive || pref == ROUTE_PREF_EVASIVE) && connection_target != target)
				{
					if (IsMapNodeDeadEnd(connection_target))
					{
						continue;
					}
				}

				const int conn_tgt_id = connection_target->getID();
				if (conn_tgt_id < 0 || conn_tgt_id > highest_id)
				{
					continue;
				}

				auto next_node = get_node(conn_tgt_id, connection_target);
				if (next_node->closed)
				{
					continue;
				}

				const auto edge_dist = connection->getDistance();

				float weight_factor = 1.0f;
				if (pref == ROUTE_PREF_HIGHWAYS)
				{
					if ((connection_target->getFlags() & NODE_FLAG_HIGHWAY) != 0 || connection->getRightLanes() >= 2)
					{
						weight_factor = 0.55f;
					}
				}
				else if (pref == ROUTE_PREF_AVOID_HIGHWAYS)
				{
					if ((connection_target->getFlags() & NODE_FLAG_HIGHWAY) != 0)
					{
						weight_factor = 3.5f;
					}
				}
				else if (pref == ROUTE_PREF_PAVED_ONLY)
				{
					if (!connection->isPaved())
					{
						weight_factor = 25.0f;
					}
				}

				if (cat == VEHICLE_CAT_MOTORCYCLE && connection->getRoadWidth() < 6.0f)
				{
					weight_factor *= 0.8f;
				}

				// Avoid Tolls
				if ((avoid_tolls || pref == ROUTE_PREF_AVOID_TOLLS) && connection->hasToll())
				{
					weight_factor *= 15.0f;
				}

				// Evasive preferences
				if (evasive || pref == ROUTE_PREF_EVASIVE)
				{
					if ((connection_target->getFlags() & NODE_FLAG_HIGHWAY) != 0 || connection->getRightLanes() >= 2)
					{
						weight_factor *= 2.5f;
					}
					if (connection->getRoadWidth() < 6.5f)
					{
						weight_factor *= 0.65f;
					}
					if (connection_target->getConnections().size() >= 3)
					{
						weight_factor *= 0.85f;
					}
				}

				// Eco slope penalty
				if (max_slope_gradient > 0.0f || pref == ROUTE_PREF_ECO)
				{
					const float threshold = (max_slope_gradient > 0.0f) ? max_slope_gradient : 0.12f;
					const float dz = connection_target->getZ() - current_node->node->getZ();
					const float dx = connection_target->getX() - current_node->node->getX();
					const float dy = connection_target->getY() - current_node->node->getY();
					const float horiz = std::sqrt(dx * dx + dy * dy);
					if (dz > 0.0f && horiz > 0.1f)
					{
						const float slope = dz / horiz;
						if (slope > threshold)
						{
							weight_factor *= (1.0f + (slope - threshold) * 25.0f);
						}
					}
				}

				// Danger / crime area penalty
				if (danger_penalty > 0.0f || pref == ROUTE_PREF_SAFEST)
				{
					const float penalty = (danger_penalty > 0.0f) ? danger_penalty : 5.0f;
					const float danger = connection_target->getDangerWeight();
					if (danger > 0.0f)
					{
						weight_factor *= (1.0f + danger * penalty);
					}
				}

				const float traffic_mult = connection->getTrafficMultiplier();
				const auto travel_cost = edge_dist * weight_factor * traffic_mult;
				const auto distance = current_node->distance + travel_cost;

				if (next_node->distance > distance)
				{
					next_node->distance = distance;
					next_node->previous = current_node;
					if (next_node->target_distance < 0.0f)
					{
						next_node->target_distance = target->getDistanceFromNode(connection_target) * heuristic_scale;
					}
					next_node->total_distance = distance + next_node->target_distance;

					queue.push(next_node);
				}
			}
		}

		if (solution == nullptr)
		{
			return false;
		}

		std::vector<Node*> rev_nodes;
		auto curr = solution;
		while (curr != nullptr)
		{
			rev_nodes.push_back(curr->node);
			curr = curr->previous;
		}
		out_nodes.assign(rev_nodes.rbegin(), rev_nodes.rend());

		float actual_length = 0.0f;
		for (size_t i = 1; i < out_nodes.size(); ++i)
		{
			actual_length += out_nodes[i - 1]->getDistanceFromNode(out_nodes[i]);
		}
		out_length = actual_length;
		return true;
	}


	Path* FindPathInternal(Node* start, Node* target, const PathType type, const RoutePreference pref, const VehicleCategory cat, const float danger_penalty, const float max_slope_gradient, const bool avoid_tolls, const bool evasive, const float vehicle_height)
	{
		std::vector<Node*> nodes;
		float length = 0.0f;
		if (!FindPathNodes(start, target, nodes, length, type, pref, cat, danger_penalty, max_slope_gradient, avoid_tolls, evasive, vehicle_height))
		{
			return nullptr;
		}

		return Container::Paths::Register(nodes, length, nullptr);
	}


	int FindPath(Node* start, Node* target, const PathType type, const RoutePreference pref, const VehicleCategory cat, AMX* amx)
	{
		try
		{
			std::vector<Node*> nodes;
			float length = 0.0f;
			{
				std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
				if (!FindPathNodes(start, target, nodes, length, type, pref, cat))
				{
					return INVALID_PATH_ID;
				}
			}

			auto path = Container::Paths::Register(nodes, length, amx);
			return path->getID();
		}
		catch (...)
		{
			return INVALID_PATH_ID;
		}
	}


	Path* FindMultiStopPathInternal(const std::vector<Node*>& stops, const bool optimize_order, const PathType type, const RoutePreference pref)
	{
		if (stops.size() < 2)
		{
			return nullptr;
		}

		std::vector<Node*> ordered_stops;
		if (!optimize_order || stops.size() <= 2)
		{
			ordered_stops = stops;
		}
		else
		{
			// Greedy Nearest Neighbor TSP from stops[0] through intermediate stops, ending at stops.back()
			ordered_stops.push_back(stops.front());
			std::vector<Node*> pool(stops.begin() + 1, stops.end() - 1);

			while (!pool.empty())
			{
				Node* current = ordered_stops.back();
				size_t best_idx = 0;
				float best_dist = infinity;

				for (size_t i = 0; i < pool.size(); ++i)
				{
					const float d = current->getDistanceFromNode(pool[i]);
					if (d < best_dist)
					{
						best_dist = d;
						best_idx = i;
					}
				}

				ordered_stops.push_back(pool[best_idx]);
				pool.erase(pool.begin() + best_idx);
			}

			ordered_stops.push_back(stops.back());
		}

		std::vector<Node*> combined_nodes;
		float total_length = 0.0f;

		for (size_t i = 0; i < ordered_stops.size() - 1; ++i)
		{
			std::vector<Node*> leg_nodes;
			float leg_length = 0.0f;
			if (!FindPathNodes(ordered_stops[i], ordered_stops[i + 1], leg_nodes, leg_length, type, pref))
			{
				return nullptr;
			}

			total_length += leg_length;
			const size_t start_n = (i == 0) ? 0 : 1;
			for (size_t n = start_n; n < leg_nodes.size(); ++n)
			{
				combined_nodes.push_back(leg_nodes[n]);
			}
		}

		return Container::Paths::Register(combined_nodes, total_length, nullptr);
	}


	int FindMultiStopPath(const std::vector<Node*>& stops, const bool optimize_order, const PathType type, const RoutePreference pref, AMX* amx)
	{
		try
		{
			std::vector<Node*> combined_nodes;
			float total_length = 0.0f;
			{
				std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
				if (stops.size() < 2) return INVALID_PATH_ID;

				std::vector<Node*> ordered_stops = stops;
				if (optimize_order && stops.size() > 2)
				{
					std::vector<bool> visited(stops.size(), false);
					std::vector<Node*> optimized;
					optimized.push_back(stops[0]);
					visited[0] = true;

					for (size_t step = 1; step < stops.size(); ++step)
					{
						Node* current = optimized.back();
						float best_dist = std::numeric_limits<float>::infinity();
						size_t best_idx = 0;

						for (size_t candidate = 1; candidate < stops.size(); ++candidate)
						{
							if (!visited[candidate])
							{
								const float d = current->getDistanceFromNode(stops[candidate]);
								if (d < best_dist)
								{
									best_dist = d;
									best_idx = candidate;
								}
							}
						}

						visited[best_idx] = true;
						optimized.push_back(stops[best_idx]);
					}
					ordered_stops = optimized;
				}

				for (size_t i = 0; i < ordered_stops.size() - 1; ++i)
				{
					std::vector<Node*> leg_nodes;
					float leg_length = 0.0f;
					if (!FindPathNodes(ordered_stops[i], ordered_stops[i + 1], leg_nodes, leg_length, type, pref))
					{
						return INVALID_PATH_ID;
					}

					total_length += leg_length;
					const size_t start_n = (i == 0) ? 0 : 1;
					for (size_t n = start_n; n < leg_nodes.size(); ++n)
					{
						combined_nodes.push_back(leg_nodes[n]);
					}
				}
			}

			auto path = Container::Paths::Register(combined_nodes, total_length, amx);
			return path->getID();
		}
		catch (...)
		{
			return INVALID_PATH_ID;
		}
	}


	int FindInterceptPath(Node* pursuer, Node* target, float target_speed_kmh, float pursuer_speed_kmh, int& out_intercept_node_id, float& out_time_to_intercept, const PathType type, AMX* amx)
	{
		if (pursuer == nullptr || target == nullptr || pursuer->isSetForDeletion() || target->isSetForDeletion())
		{
			return INVALID_PATH_ID;
		}

		const float v_target = std::max(2.0f, (target_speed_kmh * 1000.0f) / 3600.0f);
		const float v_pursuer = std::max(2.0f, (pursuer_speed_kmh * 1000.0f) / 3600.0f);

		std::vector<Node*> projected_nodes;
		std::vector<float> projected_times;

		Node* curr = target;
		projected_nodes.push_back(curr);
		projected_times.push_back(0.0f);

		float accumulated_target_dist = 0.0f;
		std::unordered_set<int> visited_target_ids;
		visited_target_ids.insert(curr->getID());

		const int max_horizon = 30;
		for (int step = 0; step < max_horizon; ++step)
		{
			Connection* best_conn = nullptr;
			float best_conn_score = -1.0f;

			for (auto conn : curr->getConnections())
			{
				if (conn->isBlocked()) continue;
				auto tgt = conn->getTarget();
				if (tgt == nullptr || tgt->isBlocked() || tgt->isSetForDeletion() || (type != PATH_TYPE_ANY && tgt->getType() != type)) continue;
				if (visited_target_ids.find(tgt->getID()) != visited_target_ids.end()) continue;

				float score = conn->getRoadWidth() + conn->getDistance();
				if (score > best_conn_score)
				{
					best_conn_score = score;
					best_conn = conn;
				}
			}

			if (best_conn == nullptr) break;

			curr = best_conn->getTarget();
			visited_target_ids.insert(curr->getID());
			accumulated_target_dist += best_conn->getDistance();

			projected_nodes.push_back(curr);
			projected_times.push_back(accumulated_target_dist / v_target);
		}

		Node* chosen_intercept_node = projected_nodes.back();
		float chosen_time = projected_times.back();
		int chosen_path_id = INVALID_PATH_ID;
		std::vector<Node*> chosen_nodes;
		float chosen_length = 0.0f;

		for (size_t i = 1; i < projected_nodes.size(); ++i)
		{
			Node* candidate = projected_nodes[i];
			if (candidate == nullptr || candidate->isSetForDeletion()) continue;
			const float t_target = projected_times[i];
			const float approx_pursuer_dist = pursuer->getDistanceFromNode(candidate) * 1.25f;
			const float approx_pursuer_time = approx_pursuer_dist / v_pursuer;

			if (approx_pursuer_time <= t_target + 2.0f)
			{
				std::vector<Node*> cand_nodes;
				float cand_len = 0.0f;
				if (FindPathNodes(pursuer, candidate, cand_nodes, cand_len, type, ROUTE_PREF_HIGHWAYS))
				{
					const float actual_time = cand_len / v_pursuer;
					if (actual_time <= t_target + 3.0f || i == projected_nodes.size() - 1)
					{
						chosen_intercept_node = candidate;
						chosen_time = actual_time;
						chosen_nodes = std::move(cand_nodes);
						chosen_length = cand_len;
						break;
					}
				}
			}
		}

		if (chosen_nodes.empty())
		{
			std::vector<Node*> cand_nodes;
			float cand_len = 0.0f;
			if (FindPathNodes(pursuer, target, cand_nodes, cand_len, type, ROUTE_PREF_HIGHWAYS))
			{
				chosen_time = cand_len / v_pursuer;
				chosen_intercept_node = target;
				chosen_nodes = std::move(cand_nodes);
				chosen_length = cand_len;
			}
		}

		if (!chosen_nodes.empty())
		{
			auto path = Container::Paths::Register(chosen_nodes, chosen_length, amx);
			chosen_path_id = path->getID();
		}

		out_intercept_node_id = (chosen_intercept_node != nullptr) ? chosen_intercept_node->getID() : target->getID();
		out_time_to_intercept = chosen_time;
		return chosen_path_id;
	}


	std::vector<int> GetReachableNodes(Node* source, const float limit_value, const bool by_time, const float speed_kmh, const PathType type)
	{
		std::vector<int> reachable_ids;
		if (source == nullptr || source->isBlocked() || source->isSetForDeletion() || limit_value <= 0.0f)
		{
			return reachable_ids;
		}

		const float speed_mps = std::max(1.0f, (speed_kmh * 1000.0f) / 3600.0f);

		struct QNode
		{
			Node* node;
			float cost;
		};

		auto comp = [](const QNode& a, const QNode& b) { return a.cost > b.cost; };
		std::priority_queue<QNode, std::vector<QNode>, decltype(comp)> queue(comp);
		std::vector<float> min_cost(Container::Nodes::GetHighestID() + 1, infinity);

		min_cost[source->getID()] = 0.0f;
		queue.push({ source, 0.0f });

		while (!queue.empty())
		{
			auto current = queue.top();
			queue.pop();

			if (current.cost > min_cost[current.node->getID()]) continue;

			reachable_ids.push_back(current.node->getID());

			for (auto conn : current.node->getConnections())
			{
				if (conn->isBlocked()) continue;
				auto target = conn->getTarget();
				if (target == nullptr || target->isBlocked() || target->isSetForDeletion() || (type != PATH_TYPE_ANY && target->getType() != type)) continue;

				float edge_cost = conn->getDistance();
				if (by_time)
				{
					edge_cost /= speed_mps;
				}

				float new_cost = current.cost + edge_cost;
				if (new_cost <= limit_value && new_cost < min_cost[target->getID()])
				{
					min_cost[target->getID()] = new_cost;
					queue.push({ target, new_cost });
				}
			}
		}

		return reachable_ids;
	}


	bool IsMapNodeDeadEnd(Node* node)
	{
		if (node == nullptr) return true;
		if (node->getOutDegree() <= 1) return true;
		const auto& conns = node->getConnections();
		int active_out = 0;
		for (auto c : conns)
		{
			if (c != nullptr && !c->isBlocked())
			{
				auto tgt = c->getTarget();
				if (tgt != nullptr && tgt != node && !tgt->isBlocked())
				{
					if (++active_out > 1)
					{
						return false;
					}
				}
			}
		}
		return active_out <= 1;
	}


	int FindSafePath(Node* start, Node* target, float danger_penalty, const PathType type, const RoutePreference pref, AMX* amx)
	{
		try
		{
			std::vector<Node*> nodes;
			float length = 0.0f;
			{
				std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
				if (!FindPathNodes(start, target, nodes, length, type, pref, VEHICLE_CAT_STANDARD, danger_penalty, 0.0f, false, false))
				{
					return INVALID_PATH_ID;
				}
			}

			auto path = Container::Paths::Register(nodes, length, amx);
			return path->getID();
		}
		catch (...)
		{
			return INVALID_PATH_ID;
		}
	}


	int FindEcoPath(Node* start, Node* target, float max_slope_gradient, const PathType type, AMX* amx)
	{
		try
		{
			std::vector<Node*> nodes;
			float length = 0.0f;
			{
				std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
				if (!FindPathNodes(start, target, nodes, length, type, ROUTE_PREF_ECO, VEHICLE_CAT_STANDARD, 0.0f, max_slope_gradient, false, false))
				{
					return INVALID_PATH_ID;
				}
			}

			auto path = Container::Paths::Register(nodes, length, amx);
			return path->getID();
		}
		catch (...)
		{
			return INVALID_PATH_ID;
		}
	}


	int FindEvasivePath(Node* start, Node* target, const PathType type, AMX* amx)
	{
		try
		{
			std::vector<Node*> nodes;
			float length = 0.0f;
			{
				std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
				if (!FindPathNodes(start, target, nodes, length, type, ROUTE_PREF_EVASIVE, VEHICLE_CAT_STANDARD, 0.0f, 0.0f, false, true))
				{
					return INVALID_PATH_ID;
				}
			}

			auto path = Container::Paths::Register(nodes, length, amx);
			return path->getID();
		}
		catch (...)
		{
			return INVALID_PATH_ID;
		}
	}


	int FindPathAvoidTolls(Node* start, Node* target, const PathType type, const RoutePreference pref, AMX* amx)
	{
		try
		{
			std::vector<Node*> nodes;
			float length = 0.0f;
			{
				std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
				if (!FindPathNodes(start, target, nodes, length, type, pref, VEHICLE_CAT_STANDARD, 0.0f, 0.0f, true, false))
				{
					return INVALID_PATH_ID;
				}
			}

			auto path = Container::Paths::Register(nodes, length, amx);
			return path->getID();
		}
		catch (...)
		{
			return INVALID_PATH_ID;
		}
	}


	int FindNearestPointByRoad(float from_x, float from_y, float from_z, const std::vector<std::tuple<float, float, float>>& destinations, int& out_nearest_index, float& out_road_distance, int& out_path_id, PathType type, AMX* amx, bool create_path)
	{
		out_nearest_index = -1;
		out_road_distance = infinity;
		out_path_id = INVALID_PATH_ID;

		if (destinations.empty()) return -1;

		std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
		Node* start = Container::Nodes::FindClosest(from_x, from_y, from_z, type);
		if (start == nullptr || start->isBlocked())
		{
			return -1;
		}

		std::unordered_map<int, std::vector<int>> target_node_map;
		for (size_t i = 0; i < destinations.size(); ++i)
		{
			Node* dest_node = Container::Nodes::FindClosest(std::get<0>(destinations[i]), std::get<1>(destinations[i]), std::get<2>(destinations[i]), type);
			if (dest_node != nullptr)
			{
				target_node_map[dest_node->getID()].push_back(static_cast<int>(i));
			}
		}

		if (target_node_map.empty())
		{
			return -1;
		}

		const int highest_id = Container::Nodes::GetHighestID();
		const int start_id = start->getID();
		if (start_id < 0 || start_id > highest_id)
		{
			return -1;
		}

		if (tl_visited_nodes.size() <= static_cast<size_t>(highest_id))
		{
			tl_visited_nodes.resize(highest_id + 1);
		}

		++tl_visited_generation;
		if (tl_visited_generation == 0)
		{
			std::fill(tl_visited_nodes.begin(), tl_visited_nodes.end(), VisitedNode{});
			tl_visited_generation = 1;
		}

		auto get_node = [&](int id, Node* n) -> VisitedNode* {
			auto& vn = tl_visited_nodes[id];
			if (vn.generation != tl_visited_generation)
			{
				vn.generation = tl_visited_generation;
				vn.closed = false;
				vn.node = n;
				vn.previous = nullptr;
				vn.distance = infinity;
				vn.target_distance = 0.0f;
				vn.total_distance = infinity;
			}
			return &vn;
		};

		std::priority_queue<VisitedNode*, std::vector<VisitedNode*>, VisitedNodeComparator> queue;

		const auto first_node = get_node(start_id, start);
		first_node->distance = 0.0f;
		first_node->total_distance = 0.0f;
		queue.push(first_node);

		VisitedNode* solution = nullptr;
		int matched_dest_idx = -1;
		float best_total_dist = infinity;

		while (!queue.empty())
		{
			const auto current = queue.top();
			queue.pop();

			if (current->closed)
			{
				continue;
			}
			current->closed = true;

			if (current->distance > best_total_dist)
			{
				break;
			}

			auto it = target_node_map.find(current->node->getID());
			if (it != target_node_map.end())
			{
				for (int d_idx : it->second)
				{
					const auto& pt = destinations[d_idx];
					const float direct_to_dest = current->node->getDistanceFromPoint(std::get<0>(pt), std::get<1>(pt), std::get<2>(pt));
					const float full_road_dist = current->distance + start->getDistanceFromPoint(from_x, from_y, from_z) + direct_to_dest;

					if (full_road_dist < best_total_dist)
					{
						best_total_dist = full_road_dist;
						matched_dest_idx = d_idx;
						solution = current;
					}
				}
			}

			for (auto connection : current->node->getConnections())
			{
				if (connection->isBlocked()) continue;
				auto target = connection->getTarget();
				if (target == nullptr || target == current->node || target->isSetForDeletion() || target->isBlocked() || (type != PATH_TYPE_ANY && target->getType() != type)) continue;

				const int target_id = target->getID();
				if (target_id < 0 || target_id > highest_id) continue;

				auto next_node = get_node(target_id, target);
				if (next_node->closed) continue;

				const float cost = connection->getDistance() * connection->getTrafficMultiplier();
				const float distance = current->distance + cost;

				if (next_node->distance > distance)
				{
					next_node->distance = distance;
					next_node->previous = current;
					next_node->total_distance = distance;
					queue.push(next_node);
				}
			}
		}

		if (solution == nullptr || matched_dest_idx == -1)
		{
			return -1;
		}

		out_nearest_index = matched_dest_idx;
		out_road_distance = best_total_dist;

		if (create_path && amx != nullptr)
		{
			std::vector<Node*> rev_nodes;
			auto curr = solution;
			while (curr != nullptr)
			{
				rev_nodes.push_back(curr->node);
				curr = curr->previous;
			}

			std::vector<Node*> forward_nodes(rev_nodes.rbegin(), rev_nodes.rend());
			float actual_length = 0.0f;
			for (size_t i = 1; i < forward_nodes.size(); ++i)
			{
				actual_length += forward_nodes[i - 1]->getDistanceFromNode(forward_nodes[i]);
			}

			auto result_path = Container::Paths::Register(forward_nodes, actual_length, amx);
			out_path_id = (result_path != nullptr) ? result_path->getID() : INVALID_PATH_ID;
		}
		else
		{
			out_path_id = INVALID_PATH_ID;
		}

		return 0;
	}


	bool GetPathElevationStats(Path* path, float& out_min_z, float& out_max_z, float& out_total_climb, float& out_max_incline_percent)
	{
		out_min_z = 0.0f;
		out_max_z = 0.0f;
		out_total_climb = 0.0f;
		out_max_incline_percent = 0.0f;

		if (path == nullptr) return false;

		const auto& nodes = path->getNodes();
		if (nodes.empty()) return false;

		out_min_z = nodes[0]->getZ();
		out_max_z = nodes[0]->getZ();

		for (size_t i = 1; i < nodes.size(); ++i)
		{
			const float prev_z = nodes[i - 1]->getZ();
			const float curr_z = nodes[i]->getZ();

			out_min_z = std::min(out_min_z, curr_z);
			out_max_z = std::max(out_max_z, curr_z);

			const float dz = curr_z - prev_z;
			if (dz > 0.0f)
			{
				out_total_climb += dz;
				const float dx = nodes[i]->getX() - nodes[i - 1]->getX();
				const float dy = nodes[i]->getY() - nodes[i - 1]->getY();
				const float horiz = std::sqrt(dx * dx + dy * dy);
				if (horiz > 0.1f)
				{
					const float incline = (dz / horiz) * 100.0f;
					out_max_incline_percent = std::max(out_max_incline_percent, incline);
				}
			}
		}

		return true;
	}


	bool GetPathOffsetPosition(Path* path, float distance_along_path, float lateral_offset, float& out_x, float& out_y, float& out_z, float& out_angle)
	{
		out_x = 0.0f;
		out_y = 0.0f;
		out_z = 0.0f;
		out_angle = 0.0f;

		if (path == nullptr) return false;

		const auto& nodes = path->getNodes();
		if (nodes.empty()) return false;

		if (nodes.size() == 1)
		{
			out_x = nodes[0]->getX();
			out_y = nodes[0]->getY();
			out_z = nodes[0]->getZ();
			out_angle = 0.0f;
			return true;
		}

		float accum_dist = 0.0f;
		for (size_t i = 0; i < nodes.size() - 1; ++i)
		{
			const float seg_len = nodes[i]->getDistanceFromNode(nodes[i + 1]);
			if (distance_along_path <= accum_dist + seg_len || i == nodes.size() - 2)
			{
				float t = (seg_len > 0.0001f) ? ((distance_along_path - accum_dist) / seg_len) : 0.0f;
				t = std::max(0.0f, std::min(1.0f, t));

				const float ax = nodes[i]->getX(), ay = nodes[i]->getY(), az = nodes[i]->getZ();
				const float bx = nodes[i + 1]->getX(), by = nodes[i + 1]->getY(), bz = nodes[i + 1]->getZ();

				const float cx = ax + t * (bx - ax);
				const float cy = ay + t * (by - ay);
				const float cz = az + t * (bz - az);

				float dx = bx - ax;
				float dy = by - ay;
				const float dxy = std::sqrt(dx * dx + dy * dy);

				if (dxy > 0.0001f)
				{
					dx /= dxy;
					dy /= dxy;

					float bearing = std::atan2(-dx, dy) * 180.0f / 3.14159265358979323846f;
					if (bearing < 0.0f) bearing += 360.0f;
					if (bearing >= 360.0f) bearing -= 360.0f;

					const float nx = dy;
					const float ny = -dx;

					out_x = cx + nx * lateral_offset;
					out_y = cy + ny * lateral_offset;
					out_z = cz;
					out_angle = bearing;
				}
				else
				{
					out_x = cx;
					out_y = cy;
					out_z = cz;
					out_angle = 0.0f;
				}
				return true;
			}
			accum_dist += seg_len;
		}

		return true;
	}


	int FindPathPavedOnly(Node* start, Node* target, const PathType type, AMX* amx)
	{
		try
		{
			std::vector<Node*> nodes;
			float length = 0.0f;
			{
				std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
				if (!FindPathNodes(start, target, nodes, length, type, ROUTE_PREF_PAVED_ONLY, VEHICLE_CAT_STANDARD, 0.0f, 0.0f, false, false, 0.0f))
				{
					return INVALID_PATH_ID;
				}
			}

			auto path = Container::Paths::Register(nodes, length, amx);
			return path->getID();
		}
		catch (...)
		{
			return INVALID_PATH_ID;
		}
	}


	int FindPathWithHeightLimit(Node* start, Node* target, float vehicle_height, const PathType type, const RoutePreference pref, AMX* amx)
	{
		try
		{
			std::vector<Node*> nodes;
			float length = 0.0f;
			{
				std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
				if (!FindPathNodes(start, target, nodes, length, type, pref, VEHICLE_CAT_STANDARD, 0.0f, 0.0f, false, false, vehicle_height))
				{
					return INVALID_PATH_ID;
				}
			}

			auto path = Container::Paths::Register(nodes, length, amx);
			return path->getID();
		}
		catch (...)
		{
			return INVALID_PATH_ID;
		}
	}


	bool GetSmoothPathPoints(Path* path, float spacing_meters, std::vector<std::tuple<float, float, float>>& out_points, int max_points)
	{
		out_points.clear();
		if (path == nullptr || spacing_meters <= 0.0f || max_points <= 0) return false;

		const auto& nodes = path->getNodes();
		if (nodes.empty()) return false;

		if (nodes.size() == 1)
		{
			out_points.push_back({ nodes[0]->getX(), nodes[0]->getY(), nodes[0]->getZ() });
			return true;
		}

		const float total_len = path->getLength();
		if (total_len <= 0.001f)
		{
			out_points.push_back({ nodes[0]->getX(), nodes[0]->getY(), nodes[0]->getZ() });
			return true;
		}

		std::vector<float> dists;
		dists.reserve(nodes.size());
		float acc = 0.0f;
		dists.push_back(0.0f);
		for (size_t i = 1; i < nodes.size(); ++i)
		{
			acc += nodes[i - 1]->getDistanceFromNode(nodes[i]);
			dists.push_back(acc);
		}

		float d = 0.0f;
		size_t cur_seg = 0;

		while (d <= total_len + 0.01f && static_cast<int>(out_points.size()) < max_points)
		{
			while (cur_seg < dists.size() - 2 && dists[cur_seg + 1] < d)
			{
				cur_seg++;
			}

			size_t i0 = (cur_seg > 0) ? cur_seg - 1 : cur_seg;
			size_t i1 = cur_seg;
			size_t i2 = cur_seg + 1;
			size_t i3 = (cur_seg + 2 < nodes.size()) ? cur_seg + 2 : cur_seg + 1;

			float seg_start = dists[i1];
			float seg_end = dists[i2];
			float seg_len = seg_end - seg_start;
			float t = (seg_len > 0.0001f) ? ((d - seg_start) / seg_len) : 0.0f;
			t = std::max(0.0f, std::min(1.0f, t));

			float t2 = t * t;
			float t3 = t2 * t;

			auto interp = [t, t2, t3](float p0, float p1, float p2, float p3) {
				return 0.5f * ((2.0f * p1) +
					(-p0 + p2) * t +
					(2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
					(-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
			};

			float x = interp(nodes[i0]->getX(), nodes[i1]->getX(), nodes[i2]->getX(), nodes[i3]->getX());
			float y = interp(nodes[i0]->getY(), nodes[i1]->getY(), nodes[i2]->getY(), nodes[i3]->getY());
			float z = interp(nodes[i0]->getZ(), nodes[i1]->getZ(), nodes[i2]->getZ(), nodes[i3]->getZ());

			out_points.push_back({ x, y, z });

			d += spacing_meters;
		}

		return !out_points.empty();
	}


	bool GetPathRoundaboutInfo(Path* path, int index, bool& is_roundabout, int& exit_number, float& roundabout_radius)
	{
		is_roundabout = false;
		exit_number = 0;
		roundabout_radius = 0.0f;

		if (path == nullptr) return false;
		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());
		if (index < 0 || index >= count) return false;

		int start_idx = std::max(0, index - 2);
		int end_idx = std::min(count - 1, index + 4);
		if (end_idx - start_idx < 3) return true;

		float sum_x = 0.0f, sum_y = 0.0f;
		int pts = 0;
		for (int i = start_idx; i <= end_idx; ++i)
		{
			sum_x += nodes[i]->getX();
			sum_y += nodes[i]->getY();
			pts++;
		}
		float cx = sum_x / pts;
		float cy = sum_y / pts;

		float avg_r = 0.0f;
		for (int i = start_idx; i <= end_idx; ++i)
		{
			float dx = nodes[i]->getX() - cx;
			float dy = nodes[i]->getY() - cy;
			avg_r += std::sqrt(dx * dx + dy * dy);
		}
		avg_r /= pts;

		if (avg_r >= 8.0f && avg_r <= 55.0f)
		{
			int exits = 1;
			for (int i = start_idx; i < index && i < count - 1; ++i)
			{
				for (auto conn : nodes[i]->getConnections())
				{
					auto tgt = conn->getTarget();
					if (tgt != nullptr && tgt != nodes[i + 1])
					{
						float dx = tgt->getX() - cx;
						float dy = tgt->getY() - cy;
						float dist_to_c = std::sqrt(dx * dx + dy * dy);
						if (dist_to_c > avg_r * 1.3f)
						{
							exits++;
							break;
						}
					}
				}
			}

			is_roundabout = true;
			exit_number = exits;
			roundabout_radius = avg_r;
		}

		return true;
	}


	bool GetPathRejoinPoint(Path* old_path, Path* new_path, int& diverge_step, int& rejoin_old_step, int& rejoin_new_step)
	{
		diverge_step = -1;
		rejoin_old_step = -1;
		rejoin_new_step = -1;

		if (old_path == nullptr || new_path == nullptr) return false;

		const auto& old_nodes = old_path->getNodes();
		const auto& new_nodes = new_path->getNodes();

		if (old_nodes.empty() || new_nodes.empty()) return false;

		const int old_size = static_cast<int>(old_nodes.size());
		const int new_size = static_cast<int>(new_nodes.size());

		int min_len = std::min(old_size, new_size);
		int div = 0;
		while (div < min_len && old_nodes[div]->getID() == new_nodes[div]->getID())
		{
			div++;
		}
		diverge_step = div;

		std::unordered_map<int, int> old_node_indices;
		for (int i = diverge_step; i < old_size; ++i)
		{
			old_node_indices[old_nodes[i]->getID()] = i;
		}

		for (int j = diverge_step; j < new_size; ++j)
		{
			auto it = old_node_indices.find(new_nodes[j]->getID());
			if (it != old_node_indices.end())
			{
				rejoin_new_step = j;
				rejoin_old_step = it->second;
				return true;
			}
		}

		rejoin_old_step = old_size - 1;
		rejoin_new_step = new_size - 1;
		return true;
	}
};
