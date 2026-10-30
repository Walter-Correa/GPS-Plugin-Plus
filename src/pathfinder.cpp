#include <cmath>
#include <queue>
#include <exception>
#include <functional>

#include "pathfinder.h"
#include "connection.h"
#include "node.h"
#include "amx.h"


namespace Pathfinder
{
	float infinity = std::numeric_limits<float>::infinity();

	struct VisitedNode
	{
		Node* node = nullptr;
		VisitedNode* previous = nullptr;
		float distance = infinity;
		float target_distance = infinity;
		float total_distance = infinity;
	};


	bool NodeComparator(const VisitedNode* lhs, const VisitedNode* rhs)
	{
		return lhs->total_distance > rhs->total_distance;
	}


	struct PathNotFoundException : std::exception
	{
		const char* what() const noexcept override
		{
			return "Failed to find a path!";
		}
	};


	Path* FindPathInternal(Node* start, Node* target, const PathType type, const RoutePreference pref)
	{
		if (start == nullptr || target == nullptr || start->isBlocked() || target->isBlocked())
		{
			throw PathNotFoundException();
		}

		if (start == target || start->isSetForDeletion() || target->isSetForDeletion())
		{
			auto result = new Path(0.0f);
			result->addNodeToEnd(start);
			return result;
		}

		std::priority_queue<VisitedNode*, std::vector<VisitedNode*>, std::function<bool(VisitedNode*, VisitedNode*)>> queue(NodeComparator);
		std::vector<VisitedNode> existing_paths(Container::Nodes::GetHighestID() + 1);
		VisitedNode* solution = nullptr;

		const auto first_node = &existing_paths.at(start->getID());
		first_node->distance = 0.0f;
		first_node->total_distance = first_node->target_distance = start->getDistanceFromNode(target);
		first_node->node = start;
		queue.push(first_node);

		while (!queue.empty())
		{
			const auto current_node = queue.top();
			queue.pop();

			if (solution != nullptr && NodeComparator(current_node, solution))
			{
				break;
			}

			for (auto connection : current_node->node->getConnections())
			{
				if (connection->isBlocked())
				{
					continue;
				}

				const auto connection_target = connection->getTarget();

				if (connection_target == current_node->node || connection_target->isSetForDeletion() || connection_target->isBlocked())
				{
					continue;
				}

				if (connection_target->getType() != type)
				{
					continue;
				}

				auto next_node = &existing_paths.at(connection_target->getID());
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

				const auto travel_cost = edge_dist * weight_factor;
				const auto distance = current_node->distance + travel_cost;

				if (next_node->distance > distance)
				{
					if (next_node->distance == infinity)
					{
						next_node->node = connection_target;
						next_node->target_distance = target->getDistanceFromNode(connection_target);

						if (connection_target == target)
						{
							solution = next_node;
						}
					}

					next_node->distance = distance;
					next_node->previous = current_node;
					next_node->total_distance = next_node->target_distance + distance;

					queue.push(next_node);
				}
			}
		}

		if (solution == nullptr)
		{
			throw PathNotFoundException();
		}

		auto result = new Path(0.0f);

		while (solution != nullptr)
		{
			result->addNodeToStart(solution->node);
			solution = solution->previous;
		}

		float actual_length = 0.0f;
		const auto& path_nodes = result->getNodes();
		for (size_t i = 1; i < path_nodes.size(); ++i)
		{
			actual_length += path_nodes[i - 1]->getDistanceFromNode(path_nodes[i]);
		}
		result->setLength(actual_length);

		auto index = 0;

		for (auto node : result->getNodes())
		{
			node->addToPath(result, index++);
		}

		return result;
	}


	int FindPath(Node* start, Node* target, const PathType type, const RoutePreference pref)
	{
		try
		{
			Container::LockShared();
			const auto path = FindPathInternal(start, target, type, pref);
			Container::UnlockShared();

			Container::LockExclusive();
			const auto result = Container::Paths::Add(path);
			Container::UnlockExclusive();

			return result;
		}
		catch (const PathNotFoundException &e)
		{
			Container::UnlockShared();
			return INVALID_PATH_ID;
		}
	}


	void FindPathThreaded(Node* start, Node* target, Callback* callback, const PathType type, const RoutePreference pref)
	{
		try
		{
			Container::LockShared();
			const auto path = FindPathInternal(start, target, type, pref);
			Container::UnlockShared();

			Container::LockExclusive();
			const auto result = Container::Paths::Add(path);
			Container::UnlockExclusive();

			callback->setResult(result);
		}
		catch (const PathNotFoundException &e)
		{
			Container::UnlockShared();
			callback->setResult(INVALID_PATH_ID);
		}

		callback->getAmx()->queueCallback(callback);
	}


	Path* FindMultiStopPathInternal(const std::vector<Node*>& stops, const bool optimize_order, const PathType type, const RoutePreference pref)
	{
		if (stops.size() < 2)
		{
			throw PathNotFoundException();
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

		std::vector<Path*> legs;
		try
		{
			for (size_t i = 0; i < ordered_stops.size() - 1; ++i)
			{
				Path* leg = FindPathInternal(ordered_stops[i], ordered_stops[i + 1], type, pref);
				legs.push_back(leg);
			}
		}
		catch (...)
		{
			for (auto leg : legs)
			{
				delete leg;
			}
			throw PathNotFoundException();
		}

		auto combined = new Path(0.0f);
		float total_length = 0.0f;

		for (size_t leg_idx = 0; leg_idx < legs.size(); ++leg_idx)
		{
			const auto leg_nodes = legs[leg_idx]->getNodes();
			total_length += legs[leg_idx]->getLength();

			const size_t start_n = (leg_idx == 0) ? 0 : 1;
			for (size_t n = start_n; n < leg_nodes.size(); ++n)
			{
				combined->addNodeToEnd(leg_nodes[n]);
			}

			delete legs[leg_idx];
		}

		combined->setLength(total_length);

		int index = 0;
		for (auto node : combined->getNodes())
		{
			node->addToPath(combined, index++);
		}

		return combined;
	}


	int FindMultiStopPath(const std::vector<Node*>& stops, const bool optimize_order, const PathType type, const RoutePreference pref)
	{
		try
		{
			Container::LockShared();
			const auto path = FindMultiStopPathInternal(stops, optimize_order, type, pref);
			Container::UnlockShared();

			Container::LockExclusive();
			const auto result = Container::Paths::Add(path);
			Container::UnlockExclusive();

			return result;
		}
		catch (const PathNotFoundException &e)
		{
			Container::UnlockShared();
			return INVALID_PATH_ID;
		}
	}
};
