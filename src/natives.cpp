#include <map>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include <algorithm>

#include "natives.h"
#include "callback.h"
#include "connection.h"
#include "node.h"
#include "path.h"
#include "file.h"
#include "amx_handler.h"
#include "zones.h"
#include "tunnels.h"
#include "radars.h"


namespace Natives
{
	std::mt19937 mersenne{ std::random_device{}() };


	cell AMX_NATIVE_CALL CreateMapNode(AMX* amx, cell* params)
	{
		if (params[0] < (4 * 4))
		{
			std::cout << __func__ << ": Expecting at least 4 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		auto type = PATH_TYPE_VEHICLE;
		auto flags = NODE_FLAG_NONE;

		if (params[0] >= (5 * 4))
		{
			type = static_cast<PathType>(params[5]);
		}

		if (params[0] >= (6 * 4))
		{
			flags = static_cast<PathNodeFlags>(params[6]);
		}

		Container::LockExclusive();

		const auto id = Container::Nodes::AllocateID();

		Container::Nodes::Add(id, x, y, z, type, flags);
		Container::UnlockExclusive();

		cell* address = nullptr;
		amx_GetAddr(amx, params[4], &address);
		*address = id;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL DestroyMapNode(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		Container::LockExclusive();
		Container::Nodes::SetForDeletion(node);
		Container::UnlockExclusive();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsValidMapNode(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);

		return Container::Nodes::Find(id) != nullptr;
	}


	cell AMX_NATIVE_CALL GetMapNodePos(AMX* amx, cell* params)
	{
		CHECK_PARAMS(4);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		auto x = node->getX(), y = node->getY(), z = node->getZ();

		cell* address = nullptr;

		amx_GetAddr(amx, params[2], &address);
		*address = amx_ftoc(x);

		amx_GetAddr(amx, params[3], &address);
		*address = amx_ftoc(y);

		amx_GetAddr(amx, params[4], &address);
		*address = amx_ftoc(z);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeType(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		return static_cast<cell>(node->getType());
	}


	cell AMX_NATIVE_CALL GetMapNodeFlags(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[2], &address);
		*address = static_cast<cell>(node->getFlags());

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL CreateConnection(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << __func__ << ": Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto source_id = static_cast<int>(params[1]);
		const auto target_id = static_cast<int>(params[2]);

		auto road_width = 8.0f;
		auto left_lanes = 1;
		auto right_lanes = 1;
		auto one_way = false;

		if (params[0] >= (4 * 4))
		{
			road_width = amx_ctof(params[4]);
		}

		if (params[0] >= (5 * 4))
		{
			left_lanes = static_cast<int>(params[5]);
		}

		if (params[0] >= (6 * 4))
		{
			right_lanes = static_cast<int>(params[6]);
		}

		if (params[0] >= (7 * 4))
		{
			one_way = params[7] != 0;
		}

		Container::LockExclusive();
		const auto id = Container::Connections::Add(source_id, target_id, road_width, left_lanes, right_lanes, one_way);
		Container::UnlockExclusive();

		if (id == -1)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = id;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL DestroyConnection(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		Container::LockExclusive();
		Container::Connections::Delete(connection);
		Container::UnlockExclusive();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionSource(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[2], &address);
		*address = connection->getSource()->getID();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionTarget(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[2], &address);
		*address = connection->getTarget()->getID();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionLaneInfo(AMX* amx, cell* params)
	{
		CHECK_PARAMS(4);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		cell* address = nullptr;

		amx_GetAddr(amx, params[2], &address);
		auto road_width = connection->getRoadWidth();
		*address = amx_ftoc(road_width);

		amx_GetAddr(amx, params[3], &address);
		*address = connection->getLeftLanes();

		amx_GetAddr(amx, params[4], &address);
		*address = connection->getRightLanes();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionLanePos(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		const auto lane_index = static_cast<int>(params[2]);
		float x = 0.0f, y = 0.0f, z = 0.0f;

		if (!connection->getLanePosition(lane_index, x, y, z))
		{
			return GPS_ERROR_INTERNAL;
		}

		cell* address = nullptr;

		amx_GetAddr(amx, params[3], &address);
		*address = amx_ftoc(x);

		amx_GetAddr(amx, params[4], &address);
		*address = amx_ftoc(y);

		amx_GetAddr(amx, params[5], &address);
		*address = amx_ftoc(z);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsConnectionOneWay(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return false;
		}

		return connection->isOneWay() ? 1 : 0;
	}


	cell AMX_NATIVE_CALL GetHighestMapNodeID(AMX* amx, cell* params)
	{
		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (1 * 4))
		{
			type = static_cast<PathType>(params[1]);
		}

		return Container::Nodes::GetHighestID(type);
	}


	cell AMX_NATIVE_CALL GetRandomMapNode(AMX* amx, cell* params)
	{
		if (params[0] < (1 * 4))
		{
			std::cout << __func__ << ": Expecting at least 1 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (2 * 4))
		{
			type = static_cast<PathType>(params[2]);
		}

		float min_z = -99999.0f;
		float max_z = 99999.0f;

		if (params[0] >= (3 * 4))
		{
			min_z = amx_ctof(params[3]);
		}

		if (params[0] >= (4 * 4))
		{
			max_z = amx_ctof(params[4]);
		}

		if (min_z <= -90000.0f && max_z >= 90000.0f)
		{
			if (type == PATH_TYPE_VEHICLE || type == PATH_TYPE_PEDESTRIAN)
			{
				min_z = 1.5f;
				max_z = 500.0f;
			}
			else if (type == PATH_TYPE_BOAT)
			{
				min_z = -10.0f;
				max_z = 20.0f;
			}
		}

		std::vector<std::pair<int, Node*>> filtered_nodes;

		for (const auto& node : Container::Nodes::GetAll())
		{
			if (node.second->isSetForDeletion() || node.second->getType() != type)
			{
				continue;
			}

			const float z = node.second->getZ();
			if (z < min_z || z > max_z)
			{
				continue;
			}

			filtered_nodes.push_back(node);
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[1], &address);

		if (filtered_nodes.empty())
		{
			*address = INVALID_NODE_ID;
			return GPS_ERROR_INVALID_NODE;
		}

		std::vector<std::pair<int, Node*>> result;
		std::sample(filtered_nodes.begin(), filtered_nodes.end(), std::back_inserter(result), 1, mersenne);

		*address = result.at(0).second->getID();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetRandomMapNodeFromArea(AMX* amx, cell* params)
	{
		if (params[0] < (5 * 4))
		{
			std::cout << __func__ << ": Expecting at least 5 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const float min_x = std::min(amx_ctof(params[1]), amx_ctof(params[3]));
		const float max_x = std::max(amx_ctof(params[1]), amx_ctof(params[3]));
		const float min_y = std::min(amx_ctof(params[2]), amx_ctof(params[4]));
		const float max_y = std::max(amx_ctof(params[2]), amx_ctof(params[4]));

		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (6 * 4))
		{
			type = static_cast<PathType>(params[6]);
		}

		float min_z = -99999.0f;
		float max_z = 99999.0f;

		if (params[0] >= (7 * 4))
		{
			min_z = amx_ctof(params[7]);
		}

		if (params[0] >= (8 * 4))
		{
			max_z = amx_ctof(params[8]);
		}

		if (min_z <= -90000.0f && max_z >= 90000.0f)
		{
			if (type == PATH_TYPE_VEHICLE || type == PATH_TYPE_PEDESTRIAN)
			{
				min_z = 1.5f;
				max_z = 500.0f;
			}
			else if (type == PATH_TYPE_BOAT)
			{
				min_z = -10.0f;
				max_z = 20.0f;
			}
		}

		std::vector<int> filtered_ids;

		for (const auto& node : Container::Nodes::GetAll())
		{
			if (node.second->isSetForDeletion() || node.second->getType() != type)
			{
				continue;
			}

			const float x = node.second->getX();
			const float y = node.second->getY();
			const float z = node.second->getZ();

			if (x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z && z <= max_z)
			{
				filtered_ids.push_back(node.first);
			}
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[5], &address);

		if (filtered_ids.empty())
		{
			*address = INVALID_NODE_ID;
			return GPS_ERROR_INVALID_NODE;
		}

		std::uniform_int_distribution<size_t> dist(0, filtered_ids.size() - 1);
		*address = filtered_ids[dist(mersenne)];

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeConnectionCount(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[2], &address);
		*address = node->getConnections().size();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeConnection(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto index = static_cast<unsigned int>(params[2]);
		const auto connections = node->getConnections();

		if (connections.size() <= index)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = connections.at(index)->getId();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionBetweenMapNodes(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto source_id = static_cast<int>(params[1]);
		const auto source = Container::Nodes::Find(source_id);

		if (source == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}
		
		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);

		if (target == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		for (const auto connection : source->getConnections())
		{
			if (connection->getTarget() == target)
			{
				cell* address = nullptr;
				amx_GetAddr(amx, params[3], &address);
				*address = connection->getId();

				return GPS_ERROR_NONE;
			}
		}

		return GPS_ERROR_INVALID_NODE;
	}


	cell AMX_NATIVE_CALL GetDistanceBetweenMapNodes(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto first_id = static_cast<int>(params[1]);
		const auto first = Container::Nodes::Find(first_id);

		if (first == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto second_id = static_cast<int>(params[2]);
		const auto second = Container::Nodes::Find(second_id);

		if (second == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		auto distance = first->getDistanceFromNode(second);

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = amx_ftoc(distance);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetAngleBetweenMapNodes(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << __func__ << ": Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto first_id = static_cast<int>(params[1]);
		const auto first = Container::Nodes::Find(first_id);

		if (first == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto second_id = static_cast<int>(params[2]);
		const auto second = Container::Nodes::Find(second_id);

		if (second == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const bool facing_angle = (params[0] >= (4 * 4)) ? (params[4] != 0) : false;

		const auto angle = first->getAngleFromNode(second, facing_angle);

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = amx_ftoc(angle);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeDistanceFromPoint(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto x = amx_ctof(params[2]), y = amx_ctof(params[3]), z = amx_ctof(params[4]);

		auto distance = node->getDistanceFromPoint(x, y, z);

		cell* address = nullptr;
		amx_GetAddr(amx, params[5], &address);
		*address = amx_ftoc(distance);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeAngleFromPoint(AMX* amx, cell* params)
	{
		if (params[0] < (4 * 4))
		{
			std::cout << __func__ << ": Expecting at least 4 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto x = amx_ctof(params[2]), y = amx_ctof(params[3]);
		const bool facing_angle = (params[0] >= (5 * 4)) ? (params[5] != 0) : false;

		const auto angle = node->getAngleFromPoint(x, y, facing_angle);

		cell* address = nullptr;
		amx_GetAddr(amx, params[4], &address);
		*address = amx_ftoc(angle);

		return GPS_ERROR_NONE;
	}


	static cell FindClosestNodeInternal(AMX* amx, cell* params, PathType default_type)
	{
		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		auto ignored = INVALID_NODE_ID;
		auto type = default_type;

		if (params[0] >= (5 * 4))
		{
			ignored = static_cast<int>(params[5]);
		}

		if (params[0] >= (6 * 4))
		{
			type = static_cast<PathType>(params[6]);
		}

		Container::LockShared();
		const auto node = Container::Nodes::FindClosest(x, y, z, type, ignored);
		const auto result = (node != nullptr) ? node->getID() : INVALID_NODE_ID;
		Container::UnlockShared();

		cell* address = nullptr;
		amx_GetAddr(amx, params[4], &address);
		*address = result;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetClosestMapNodeToPoint(AMX* amx, cell* params)
	{
		if (params[0] < (4 * 4))
		{
			std::cout << __func__ << ": Expecting at least 4 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		return FindClosestNodeInternal(amx, params, PATH_TYPE_VEHICLE);
	}


	cell AMX_NATIVE_CALL SaveMapNodesToFile(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto filename = amx_GetCppString(amx, params[1]);

		if (File::SaveNodes(filename))
		{
			return GPS_ERROR_NONE;
		}

		return GPS_ERROR_INTERNAL;
	}


	static cell FindPathInternalWrapper(AMX* amx, cell* params, PathType default_type)
	{
		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);

		if (start == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);

		if (target == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		auto type = default_type;

		if (params[0] >= (4 * 4))
		{
			type = static_cast<PathType>(params[4]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (5 * 4))
		{
			pref = static_cast<RoutePreference>(params[5]);
		}

		const auto result = Pathfinder::FindPath(start, target, type, pref);

		if (result == -1)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = result;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindPath(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << __func__ << ": Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		return FindPathInternalWrapper(amx, params, PATH_TYPE_VEHICLE);
	}


	cell AMX_NATIVE_CALL FindMultiStopPath(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << __func__ << ": Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		cell* stops_addr = nullptr;
		amx_GetAddr(amx, params[1], &stops_addr);
		const auto count = static_cast<int>(params[2]);

		if (count < 2 || stops_addr == nullptr)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		bool optimize_order = false;
		if (params[0] >= (4 * 4))
		{
			optimize_order = params[4] != 0;
		}

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (5 * 4))
		{
			type = static_cast<PathType>(params[5]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (6 * 4))
		{
			pref = static_cast<RoutePreference>(params[6]);
		}

		std::vector<Node*> stop_nodes;
		stop_nodes.reserve(count);

		for (int i = 0; i < count; ++i)
		{
			const int node_id = static_cast<int>(stops_addr[i]);
			Node* node = Container::Nodes::Find(node_id);
			if (node == nullptr || node->isBlocked() || node->isSetForDeletion())
			{
				return GPS_ERROR_INVALID_NODE;
			}
			stop_nodes.push_back(node);
		}

		const int path_id = Pathfinder::FindMultiStopPath(stop_nodes, optimize_order, type, pref);
		if (path_id == INVALID_PATH_ID)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		cell* out_addr = nullptr;
		amx_GetAddr(amx, params[3], &out_addr);
		*out_addr = path_id;

		return GPS_ERROR_NONE;
	}


	static cell FindPathThreadedInternal(AMX* amx, cell* params, PathType type, RoutePreference pref, int format_param_index)
	{
		const auto start_id = static_cast<int>(params[1]);
		auto start = Container::Nodes::Find(start_id);

		if (start == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto target_id = static_cast<int>(params[2]);
		auto target = Container::Nodes::Find(target_id);

		if (target == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto callback_name = amx_GetCppString(amx, params[format_param_index - 1]);

		char* format = nullptr;
		if (params[0] >= (format_param_index * sizeof(cell)))
		{
			amx_StrParam(amx, params[format_param_index], format);
		}

		auto callback = new Callback(AmxHandler::GetAmx(amx), callback_name, format, params, format_param_index);

		try
		{
			std::thread t([=]() { Pathfinder::FindPathThreaded(start, target, callback, type, pref); });
			t.detach();
		}
		catch (std::exception &e)
		{
			std::cout << "Failed to dispatch thread: " << e.what() << "\n";
			return GPS_ERROR_INTERNAL;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindPathThreaded(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << "FindPathThreaded: Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		return FindPathThreadedInternal(amx, params, PATH_TYPE_VEHICLE, ROUTE_PREF_SHORTEST, 4);
	}


	cell AMX_NATIVE_CALL FindPathThreadedEx(AMX* amx, cell* params)
	{
		if (params[0] < (4 * 4))
		{
			std::cout << "FindPathThreadedEx: Expecting at least 4 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto type = static_cast<PathType>(params[3]);

		if (params[0] >= (5 * 4) && params[4] >= 0 && params[4] <= 2)
		{
			const auto pref = static_cast<RoutePreference>(params[4]);
			return FindPathThreadedInternal(amx, params, type, pref, 6);
		}

		return FindPathThreadedInternal(amx, params, type, ROUTE_PREF_SHORTEST, 5);
	}


	cell AMX_NATIVE_CALL IsValidPath(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);

		Container::LockShared();
		const auto path = Container::Paths::Find(id);
		Container::UnlockShared();

		return path != nullptr;
	}


	cell AMX_NATIVE_CALL GetPathSize(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);

		Container::LockShared();
		const auto path = Container::Paths::Find(id);
		Container::UnlockShared();

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[2], &address);
		*address = path->getNodes().size();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathLength(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);

		Container::LockShared();
		const auto path = Container::Paths::Find(id);
		Container::UnlockShared();

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		auto result = path->getLength();

		cell* address = nullptr;
		amx_GetAddr(amx, params[2], &address);
		*address = amx_ftoc(result);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathNode(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto id = static_cast<int>(params[1]);

		Container::LockShared();
		const auto path = Container::Paths::Find(id);
		Container::UnlockShared();

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto index = static_cast<unsigned int>(params[2]);
		auto nodes = path->getNodes();

		if (index >= nodes.size())
		{
			return GPS_ERROR_INVALID_NODE;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = nodes.at(index)->getID();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathNodeIndex(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto id = static_cast<int>(params[1]);

		Container::LockShared();
		const auto path = Container::Paths::Find(id);
		Container::UnlockShared();

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto node_id = static_cast<int>(params[2]);
		const auto node = Container::Nodes::Find(node_id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto index = node->getPositionInPath(path);

		if (index == -1)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = index;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathLaneNodePos(AMX* amx, cell* params)
	{
		CHECK_PARAMS(6);

		const auto id = static_cast<int>(params[1]);

		Container::LockShared();
		const auto path = Container::Paths::Find(id);
		Container::UnlockShared();

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto index = static_cast<unsigned int>(params[2]);
		const auto lane_index = static_cast<int>(params[3]);
		const auto nodes = path->getNodes();

		if (index >= nodes.size())
		{
			return GPS_ERROR_INVALID_NODE;
		}

		auto current_node = nodes.at(index);
		Connection* connection = nullptr;

		if (index + 1 < nodes.size())
		{
			auto next_node = nodes.at(index + 1);
			for (auto conn : current_node->getConnections())
			{
				if (conn->getTarget() == next_node)
				{
					connection = conn;
					break;
				}
			}
		}
		else if (index > 0)
		{
			auto prev_node = nodes.at(index - 1);
			for (auto conn : prev_node->getConnections())
			{
				if (conn->getTarget() == current_node)
				{
					connection = conn;
					break;
				}
			}
		}

		float x = current_node->getX(), y = current_node->getY(), z = current_node->getZ();

		if (connection != nullptr)
		{
			connection->getLanePosition(lane_index, x, y, z);
		}

		cell* address = nullptr;

		amx_GetAddr(amx, params[4], &address);
		*address = amx_ftoc(x);

		amx_GetAddr(amx, params[5], &address);
		*address = amx_ftoc(y);

		amx_GetAddr(amx, params[6], &address);
		*address = amx_ftoc(z);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL DestroyPath(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);

		Container::LockShared();
		const auto path = Container::Paths::Find(id);
		Container::UnlockShared();

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		Container::LockExclusive();
		Container::Paths::Delete(path);
		Container::UnlockExclusive();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetMapNodeBlocked(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto blocked = params[2] != 0;

		Container::LockShared();
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			Container::UnlockShared();
			return GPS_ERROR_INVALID_NODE;
		}

		node->setBlocked(blocked);
		Container::UnlockShared();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsMapNodeBlocked(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);

		Container::LockShared();
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			Container::UnlockShared();
			return 0;
		}

		const auto blocked = node->isBlocked();
		Container::UnlockShared();

		return blocked ? 1 : 0;
	}


	cell AMX_NATIVE_CALL SetConnectionBlocked(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto blocked = params[2] != 0;

		Container::LockShared();
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			Container::UnlockShared();
			return GPS_ERROR_INVALID_CONNECTION;
		}

		connection->setBlocked(blocked);
		Container::UnlockShared();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsConnectionBlocked(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);

		Container::LockShared();
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			Container::UnlockShared();
			return 0;
		}

		const auto blocked = connection->isBlocked();
		Container::UnlockShared();

		return blocked ? 1 : 0;
	}


	cell AMX_NATIVE_CALL GetClosestPointOnRoad(AMX* amx, cell* params)
	{
		if (params[0] < (7 * 4))
		{
			std::cout << __func__ << ": Expecting at least 7 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (9 * 4))
		{
			type = static_cast<PathType>(params[9]);
		}

		float out_x = 0.0f, out_y = 0.0f, out_z = 0.0f, out_angle = 0.0f, out_road_width = 8.0f;
		int out_conn_id = INVALID_CONNECTION_ID;

		Container::LockShared();
		const auto success = Container::Connections::FindClosestRoadPoint(x, y, z, out_x, out_y, out_z, out_angle, out_road_width, out_conn_id, type);
		Container::UnlockShared();

		if (!success)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[4], &address);
		*address = amx_ftoc(out_x);

		amx_GetAddr(amx, params[5], &address);
		*address = amx_ftoc(out_y);

		amx_GetAddr(amx, params[6], &address);
		*address = amx_ftoc(out_z);

		amx_GetAddr(amx, params[7], &address);
		*address = amx_ftoc(out_angle);

		if (params[0] >= (8 * 4))
		{
			amx_GetAddr(amx, params[8], &address);
			*address = out_conn_id;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetRoadPositionInfo(AMX* amx, cell* params)
	{
		if (params[0] < (8 * 4))
		{
			std::cout << __func__ << ": Expecting at least 8 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		int curb = 0;
		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (9 * 4))
		{
			curb = static_cast<int>(params[9]);
		}

		if (params[0] >= (10 * 4))
		{
			type = static_cast<PathType>(params[10]);
		}

		float out_x = 0.0f, out_y = 0.0f, out_z = 0.0f, out_angle = 0.0f, out_road_width = 8.0f;
		int out_conn_id = INVALID_CONNECTION_ID;

		Container::LockShared();
		const auto success = Container::Connections::FindClosestRoadPoint(x, y, z, out_x, out_y, out_z, out_angle, out_road_width, out_conn_id, type);
		Container::UnlockShared();

		if (!success)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		if (curb != 0)
		{
			const float lateral_angle = (out_angle - 90.0f) * 3.14159265358979323846f / 180.0f;
			const float offset = (curb == 2) ? (out_road_width / 2.0f) : ((out_road_width / 2.0f) + 1.3f);
			out_x += offset * std::sin(-lateral_angle);
			out_y += offset * std::cos(-lateral_angle);
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[4], &address);
		*address = amx_ftoc(out_x);

		amx_GetAddr(amx, params[5], &address);
		*address = amx_ftoc(out_y);

		amx_GetAddr(amx, params[6], &address);
		*address = amx_ftoc(out_z);

		amx_GetAddr(amx, params[7], &address);
		*address = amx_ftoc(out_angle);

		amx_GetAddr(amx, params[8], &address);
		*address = amx_ftoc(out_road_width);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsPointAgainstTraffic(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);

		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		const auto facing_angle = amx_ctof(params[4]);

		float out_x = 0.0f, out_y = 0.0f, out_z = 0.0f, out_angle = 0.0f, out_road_width = 8.0f;
		int out_conn_id = INVALID_CONNECTION_ID;

		Container::LockShared();
		const auto success = Container::Connections::FindClosestRoadPoint(x, y, z, out_x, out_y, out_z, out_angle, out_road_width, out_conn_id, PATH_TYPE_VEHICLE);

		if (!success)
		{
			Container::UnlockShared();
			return GPS_ERROR_INVALID_CONNECTION;
		}

		const auto conn = Container::Connections::Find(out_conn_id);
		if (conn == nullptr || conn->getSource() == nullptr || conn->getTarget() == nullptr)
		{
			Container::UnlockShared();
			return GPS_ERROR_INVALID_CONNECTION;
		}

		const auto src = conn->getSource();
		const auto tgt = conn->getTarget();

		const float forward_angle = src->getAngleFromNode(tgt, true);

		auto get_angle_delta = [](float a, float b)
		{
			float diff = std::abs(a - b);
			while (diff > 180.0f) diff = std::abs(diff - 360.0f);
			return diff;
		};

		bool wrong_way = false;

		if (conn->isOneWay())
		{
			wrong_way = (get_angle_delta(facing_angle, forward_angle) > 90.0f);
		}
		else
		{
			const float cross = (tgt->getX() - src->getX()) * (y - src->getY()) - (tgt->getY() - src->getY()) * (x - src->getX());
			if (cross <= 0.0f)
			{
				wrong_way = (get_angle_delta(facing_angle, forward_angle) > 90.0f);
			}
			else
			{
				float reverse_angle = forward_angle + 180.0f;
				if (reverse_angle >= 360.0f) reverse_angle -= 360.0f;
				wrong_way = (get_angle_delta(facing_angle, reverse_angle) > 90.0f);
			}
		}

		Container::UnlockShared();

		cell* address = nullptr;
		amx_GetAddr(amx, params[5], &address);
		*address = wrong_way ? 1 : 0;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathTurnDirection(AMX* amx, cell* params)
	{
		CHECK_PARAMS(4);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto index = static_cast<int>(params[2]);
		const auto nodes = path->getNodes();
		const auto count = static_cast<int>(nodes.size());

		TurnDirection turn = TURN_NONE;
		float turn_angle = 0.0f;

		if (index > 0 && index < (count - 1))
		{
			const auto prev = nodes[index - 1];
			const auto curr = nodes[index];
			const auto next = nodes[index + 1];

			if (prev != nullptr && curr != nullptr && next != nullptr)
			{
				const float angle1 = prev->getAngleFromNode(curr, true);
				const float angle2 = curr->getAngleFromNode(next, true);

				float diff = angle2 - angle1;
				while (diff > 180.0f) diff -= 360.0f;
				while (diff <= -180.0f) diff += 360.0f;

				// In SA-MP compass facing angle, diff < 0 is turning right, diff > 0 is turning left
				turn_angle = -diff;

				const float abs_angle = std::abs(turn_angle);
				if (abs_angle >= 160.0f)
				{
					turn = TURN_UTURN;
				}
				else if (turn_angle > 120.0f)
				{
					turn = TURN_SHARP_RIGHT;
				}
				else if (turn_angle > 20.0f)
				{
					turn = TURN_RIGHT;
				}
				else if (turn_angle < -120.0f)
				{
					turn = TURN_SHARP_LEFT;
				}
				else if (turn_angle < -20.0f)
				{
					turn = TURN_LEFT;
				}
				else
				{
					turn = TURN_STRAIGHT;
				}
			}
		}

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = static_cast<cell>(turn);

		amx_GetAddr(amx, params[4], &address);
		*address = amx_ftoc(turn_angle);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathEstimatedTravelTime(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto speed_kmh = amx_ctof(params[2]);
		if (speed_kmh <= 0.1f)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto speed_ms = speed_kmh / 3.6f;
		const auto seconds = path->getLength() / speed_ms;

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = amx_ftoc(seconds);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathProgress(AMX* amx, cell* params)
	{
		if (params[0] < (7 * 4))
		{
			std::cout << __func__ << ": Expecting at least 7 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const float px = amx_ctof(params[2]);
		const float py = amx_ctof(params[3]);
		const float pz = amx_ctof(params[4]);

		float threshold = 25.0f;
		if (params[0] >= (8 * 4))
		{
			threshold = amx_ctof(params[8]);
		}

		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());

		if (count == 0)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		int best_step = 0;
		float min_dist_sq = std::numeric_limits<float>::infinity();

		if (count == 1)
		{
			const float dx = px - nodes[0]->getX();
			const float dy = py - nodes[0]->getY();
			const float dz = pz - nodes[0]->getZ();
			min_dist_sq = dx * dx + dy * dy + dz * dz;
			best_step = 0;
		}
		else
		{
			for (int i = 0; i < count - 1; ++i)
			{
				const float ax = nodes[i]->getX();
				const float ay = nodes[i]->getY();
				const float az = nodes[i]->getZ();

				const float bx = nodes[i + 1]->getX();
				const float by = nodes[i + 1]->getY();
				const float bz = nodes[i + 1]->getZ();

				const float abx = bx - ax;
				const float aby = by - ay;
				const float abz = bz - az;

				const float apx = px - ax;
				const float apy = py - ay;
				const float apz = pz - az;

				const float ab_len_sq = abx * abx + aby * aby + abz * abz;
				float t = 0.0f;
				if (ab_len_sq > 0.0001f)
				{
					t = (apx * abx + apy * aby + apz * abz) / ab_len_sq;
					t = std::max(0.0f, std::min(1.0f, t));
				}

				const float qx = ax + t * abx;
				const float qy = ay + t * aby;
				const float qz = az + t * abz;

				const float d_sq = (px - qx) * (px - qx) + (py - qy) * (py - qy) + (pz - qz) * (pz - qz);
				if (d_sq < min_dist_sq)
				{
					min_dist_sq = d_sq;
					best_step = (t > 0.6f) ? (i + 1) : i;
				}
			}
		}

		const float distance_to_path = std::sqrt(min_dist_sq);
		const bool off_route = (distance_to_path > threshold);

		cell* addr = nullptr;
		amx_GetAddr(amx, params[5], &addr);
		*addr = best_step;

		amx_GetAddr(amx, params[6], &addr);
		*addr = amx_ftoc(distance_to_path);

		amx_GetAddr(amx, params[7], &addr);
		*addr = off_route ? 1 : 0;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathRemainingInfo(AMX* amx, cell* params)
	{
		if (params[0] < (4 * 4))
		{
			std::cout << __func__ << ": Expecting at least 4 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto current_step = static_cast<int>(params[2]);
		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());

		float speed_kmh = 60.0f;
		if (params[0] >= (5 * 4))
		{
			speed_kmh = amx_ctof(params[5]);
			if (speed_kmh <= 0.1f)
			{
				speed_kmh = 60.0f;
			}
		}

		float remaining_dist = 0.0f;
		if (current_step >= 0 && current_step < count - 1)
		{
			for (int i = current_step; i < count - 1; ++i)
			{
				remaining_dist += nodes[i]->getDistanceFromNode(nodes[i + 1]);
			}
		}

		const float speed_mps = (speed_kmh * 1000.0f) / 3600.0f;
		const float remaining_sec = remaining_dist / speed_mps;

		cell* addr = nullptr;
		amx_GetAddr(amx, params[3], &addr);
		*addr = amx_ftoc(remaining_dist);

		amx_GetAddr(amx, params[4], &addr);
		*addr = amx_ftoc(remaining_sec);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathNextManeuver(AMX* amx, cell* params)
	{
		CHECK_PARAMS(6);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const float px = amx_ctof(params[2]);
		const float py = amx_ctof(params[3]);
		const float pz = amx_ctof(params[4]);

		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());

		if (count < 3)
		{
			const float zero = 0.0f;
			cell* addr = nullptr;
			amx_GetAddr(amx, params[5], &addr);
			*addr = TURN_NONE;
			amx_GetAddr(amx, params[6], &addr);
			*addr = amx_ftoc(zero);
			return GPS_ERROR_NONE;
		}

		int current_step = 0;
		float min_dist_sq = std::numeric_limits<float>::infinity();

		for (int i = 0; i < count - 1; ++i)
		{
			const float ax = nodes[i]->getX();
			const float ay = nodes[i]->getY();
			const float az = nodes[i]->getZ();

			const float bx = nodes[i + 1]->getX();
			const float by = nodes[i + 1]->getY();
			const float bz = nodes[i + 1]->getZ();

			const float abx = bx - ax;
			const float aby = by - ay;
			const float abz = bz - az;

			const float apx = px - ax;
			const float apy = py - ay;
			const float apz = pz - az;

			const float ab_len_sq = abx * abx + aby * aby + abz * abz;
			float t = 0.0f;
			if (ab_len_sq > 0.0001f)
			{
				t = (apx * abx + apy * aby + apz * abz) / ab_len_sq;
				t = std::max(0.0f, std::min(1.0f, t));
			}

			const float qx = ax + t * abx;
			const float qy = ay + t * aby;
			const float qz = az + t * abz;

			const float d_sq = (px - qx) * (px - qx) + (py - qy) * (py - qy) + (pz - qz) * (pz - qz);
			if (d_sq < min_dist_sq)
			{
				min_dist_sq = d_sq;
				current_step = (t > 0.6f) ? (i + 1) : i;
			}
		}

		TurnDirection next_turn = TURN_NONE;
		int turn_node_idx = -1;

		for (int i = std::max(1, current_step + 1); i < count - 1; ++i)
		{
			const auto prev = nodes[i - 1];
			const auto curr = nodes[i];
			const auto next = nodes[i + 1];

			const float angle1 = prev->getAngleFromNode(curr, true);
			const float angle2 = curr->getAngleFromNode(next, true);

			float diff = angle2 - angle1;
			while (diff > 180.0f) diff -= 360.0f;
			while (diff <= -180.0f) diff += 360.0f;

			const float turn_angle = -diff;
			const float abs_angle = std::abs(turn_angle);

			if (abs_angle >= 160.0f)
			{
				next_turn = TURN_UTURN;
				turn_node_idx = i;
				break;
			}
			else if (turn_angle > 120.0f)
			{
				next_turn = TURN_SHARP_RIGHT;
				turn_node_idx = i;
				break;
			}
			else if (turn_angle > 20.0f)
			{
				next_turn = TURN_RIGHT;
				turn_node_idx = i;
				break;
			}
			else if (turn_angle < -120.0f)
			{
				next_turn = TURN_SHARP_LEFT;
				turn_node_idx = i;
				break;
			}
			else if (turn_angle < -20.0f)
			{
				next_turn = TURN_LEFT;
				turn_node_idx = i;
				break;
			}
		}

		float dist_to_turn = 0.0f;
		if (turn_node_idx != -1)
		{
			dist_to_turn = std::sqrt(min_dist_sq);
			for (int i = current_step; i < turn_node_idx; ++i)
			{
				dist_to_turn += nodes[i]->getDistanceFromNode(nodes[i + 1]);
			}
		}

		cell* addr = nullptr;
		amx_GetAddr(amx, params[5], &addr);
		*addr = next_turn;

		amx_GetAddr(amx, params[6], &addr);
		*addr = amx_ftoc(dist_to_turn);

		return GPS_ERROR_NONE;
	}


	static void DouglasPeuckerRecursive(const std::deque<Node*>& nodes, int first, int last, float epsilon, std::vector<bool>& keep)
	{
		if (last <= first + 1)
		{
			return;
		}

		float max_dist = 0.0f;
		int index = first;

		const float ax = nodes[first]->getX();
		const float ay = nodes[first]->getY();
		const float bx = nodes[last]->getX();
		const float by = nodes[last]->getY();

		const float dx = bx - ax;
		const float dy = by - ay;
		const float line_len_sq = dx * dx + dy * dy;

		for (int i = first + 1; i < last; ++i)
		{
			float dist = 0.0f;
			const float px = nodes[i]->getX();
			const float py = nodes[i]->getY();

			if (line_len_sq < 0.0001f)
			{
				dist = nodes[i]->getDistanceFromNode(nodes[first]);
			}
			else
			{
				const float cross = std::abs((px - ax) * dy - (py - ay) * dx);
				dist = cross / std::sqrt(line_len_sq);
			}

			if (dist > max_dist)
			{
				max_dist = dist;
				index = i;
			}
		}

		if (max_dist > epsilon)
		{
			keep[index] = true;
			DouglasPeuckerRecursive(nodes, first, index, epsilon, keep);
			DouglasPeuckerRecursive(nodes, index, last, epsilon, keep);
		}
	}


	cell AMX_NATIVE_CALL GetSimplifiedPathNodes(AMX* amx, cell* params)
	{
		if (params[0] < (5 * 4))
		{
			std::cout << __func__ << ": Expecting at least 5 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const float tolerance = amx_ctof(params[2]);
		cell* out_array = nullptr;
		amx_GetAddr(amx, params[3], &out_array);
		const auto max_nodes = static_cast<int>(params[4]);

		if (out_array == nullptr || max_nodes <= 0)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());

		if (count == 0)
		{
			cell* count_addr = nullptr;
			amx_GetAddr(amx, params[5], &count_addr);
			*count_addr = 0;
			return GPS_ERROR_NONE;
		}

		std::vector<int> kept_node_ids;
		if (count <= 2 || tolerance <= 0.0f)
		{
			for (int i = 0; i < std::min(count, max_nodes); ++i)
			{
				kept_node_ids.push_back(nodes[i]->getID());
			}
		}
		else
		{
			std::vector<bool> keep(count, false);
			keep[0] = true;
			keep[count - 1] = true;

			DouglasPeuckerRecursive(nodes, 0, count - 1, tolerance, keep);

			for (int i = 0; i < count; ++i)
			{
				if (keep[i])
				{
					kept_node_ids.push_back(nodes[i]->getID());
					if (static_cast<int>(kept_node_ids.size()) >= max_nodes)
					{
						break;
					}
				}
			}
		}

		for (size_t i = 0; i < kept_node_ids.size(); ++i)
		{
			out_array[i] = kept_node_ids[i];
		}

		cell* count_addr = nullptr;
		amx_GetAddr(amx, params[5], &count_addr);
		*count_addr = static_cast<cell>(kept_node_ids.size());

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsMapNodeIntersection(AMX* amx, cell* params)
	{
		if (params[0] < (1 * 4))
		{
			std::cout << __func__ << ": Expecting at least 1 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return false;
		}

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);
		if (node == nullptr)
		{
			return false;
		}

		const auto conn_count = static_cast<int>(node->getConnections().size());

		if (params[0] >= (2 * 4))
		{
			cell* addr = nullptr;
			amx_GetAddr(amx, params[2], &addr);
			*addr = conn_count;
		}

		return (conn_count >= 3) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL GetMapNodeConnectionTarget(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);
		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto index = static_cast<size_t>(params[2]);
		const auto& conns = node->getConnections();
		if (index >= conns.size())
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto target = conns[index]->getTarget();
		if (target == nullptr)
		{
			return GPS_ERROR_INTERNAL;
		}

		cell* addr = nullptr;
		amx_GetAddr(amx, params[3], &addr);
		*addr = target->getID();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetConnectionTrafficMultiplier(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);
		const auto id = static_cast<int>(params[1]);
		const auto mult = amx_ctof(params[2]);
		Container::Connections::SetTrafficMultiplier(id, mult);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionTrafficMultiplier(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const auto id = static_cast<int>(params[1]);
		float mult = Container::Connections::GetTrafficMultiplier(id);
		return amx_ftoc(mult);
	}


	cell AMX_NATIVE_CALL SetAreaTrafficMultiplier(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const auto minx = amx_ctof(params[1]);
		const auto miny = amx_ctof(params[2]);
		const auto maxx = amx_ctof(params[3]);
		const auto maxy = amx_ctof(params[4]);
		const auto mult = amx_ctof(params[5]);
		Container::Connections::SetAreaTrafficMultiplier(minx, miny, maxx, maxy, mult);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL ResetTrafficMultipliers(AMX* amx, cell* params)
	{
		Container::Connections::ResetTrafficMultipliers();
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindInterceptPath(AMX* amx, cell* params)
	{
		if (params[0] < (7 * 4))
		{
			std::cout << __func__ << ": Expecting at least 7 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto pursuer_id = static_cast<int>(params[1]);
		const auto pursuer = Container::Nodes::Find(pursuer_id);
		if (pursuer == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_speed = amx_ctof(params[3]);
		const auto pursuer_speed = amx_ctof(params[4]);

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (8 * 4))
		{
			type = static_cast<PathType>(params[8]);
		}

		int intercept_node_id = -1;
		float time_to_intercept = 0.0f;
		int path_id = Pathfinder::FindInterceptPath(pursuer, target, target_speed, pursuer_speed, intercept_node_id, time_to_intercept, type);

		if (path_id == INVALID_PATH_ID)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		cell* addr = nullptr;
		amx_GetAddr(amx, params[5], &addr);
		*addr = path_id;

		amx_GetAddr(amx, params[6], &addr);
		*addr = intercept_node_id;

		amx_GetAddr(amx, params[7], &addr);
		*addr = amx_ftoc(time_to_intercept);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPurePursuitSteering(AMX* amx, cell* params)
	{
		CHECK_PARAMS(7);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const float vx = amx_ctof(params[2]);
		const float vy = amx_ctof(params[3]);
		const float v_angle = amx_ctof(params[4]);
		float lookahead = amx_ctof(params[5]);
		if (lookahead < 2.0f) lookahead = 2.0f;

		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());
		if (count < 2)
		{
			cell* addr = nullptr;
			float zero = 0.0f;
			amx_GetAddr(amx, params[6], &addr);
			*addr = amx_ftoc(zero);
			amx_GetAddr(amx, params[7], &addr);
			*addr = amx_ftoc(zero);
			return GPS_ERROR_NONE;
		}

		int best_seg = 0;
		float best_t = 0.0f;
		float min_dist_sq = std::numeric_limits<float>::infinity();

		for (int i = 0; i < count - 1; ++i)
		{
			const float ax = nodes[i]->getX();
			const float ay = nodes[i]->getY();
			const float bx = nodes[i + 1]->getX();
			const float by = nodes[i + 1]->getY();

			const float abx = bx - ax;
			const float aby = by - ay;
			const float apx = vx - ax;
			const float apy = vy - ay;

			const float ab_len_sq = abx * abx + aby * aby;
			float t = 0.0f;
			if (ab_len_sq > 0.0001f)
			{
				t = (apx * abx + apy * aby) / ab_len_sq;
				t = std::max(0.0f, std::min(1.0f, t));
			}

			const float qx = ax + t * abx;
			const float qy = ay + t * aby;
			const float d_sq = (vx - qx) * (vx - qx) + (vy - qy) * (vy - qy);

			if (d_sq < min_dist_sq)
			{
				min_dist_sq = d_sq;
				best_seg = i;
				best_t = t;
			}
		}

		float remain = lookahead;
		float gx = nodes.back()->getX();
		float gy = nodes.back()->getY();

		for (int i = best_seg; i < count - 1; ++i)
		{
			float start_t = (i == best_seg) ? best_t : 0.0f;
			const float ax = nodes[i]->getX() + start_t * (nodes[i + 1]->getX() - nodes[i]->getX());
			const float ay = nodes[i]->getY() + start_t * (nodes[i + 1]->getY() - nodes[i]->getY());
			const float bx = nodes[i + 1]->getX();
			const float by = nodes[i + 1]->getY();

			const float seg_len = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
			if (remain <= seg_len)
			{
				const float frac = (seg_len > 0.0001f) ? (remain / seg_len) : 0.0f;
				gx = ax + frac * (bx - ax);
				gy = ay + frac * (by - ay);
				break;
			}
			remain -= seg_len;
		}

		float target_bearing = std::atan2(gx - vx, vy - gy) * 180.0f / 3.14159265358979323846f;
		if (target_bearing < 0.0f) target_bearing += 360.0f;
		if (target_bearing >= 360.0f) target_bearing -= 360.0f;

		float diff = target_bearing - v_angle;
		while (diff > 180.0f) diff -= 360.0f;
		while (diff <= -180.0f) diff += 360.0f;

		float steer_ratio = std::max(-1.0f, std::min(1.0f, -diff / 35.0f));

		const float abs_diff = std::abs(diff);
		float recommended_speed = 80.0f;
		if (abs_diff > 50.0f) recommended_speed = 25.0f;
		else if (abs_diff > 25.0f) recommended_speed = 45.0f;
		else if (abs_diff > 10.0f) recommended_speed = 65.0f;

		cell* addr = nullptr;
		amx_GetAddr(amx, params[6], &addr);
		*addr = amx_ftoc(steer_ratio);

		amx_GetAddr(amx, params[7], &addr);
		*addr = amx_ftoc(recommended_speed);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetReachableMapNodes(AMX* amx, cell* params)
	{
		if (params[0] < (6 * 4))
		{
			std::cout << __func__ << ": Expecting at least 6 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto source_id = static_cast<int>(params[1]);
		const auto source = Container::Nodes::Find(source_id);
		if (source == nullptr) return GPS_ERROR_INVALID_NODE;

		const float limit = amx_ctof(params[2]);
		const bool by_time = params[3] != 0;
		const float speed = amx_ctof(params[4]);

		cell* out_nodes = nullptr;
		amx_GetAddr(amx, params[5], &out_nodes);
		const auto max_nodes = static_cast<int>(params[6]);

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (8 * 4))
		{
			type = static_cast<PathType>(params[8]);
		}

		if (out_nodes == nullptr || max_nodes <= 0)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		std::vector<int> reached = Pathfinder::GetReachableNodes(source, limit, by_time, speed, type);
		const int written = std::min(static_cast<int>(reached.size()), max_nodes);

		for (int i = 0; i < written; ++i)
		{
			out_nodes[i] = reached[i];
		}

		cell* count_addr = nullptr;
		amx_GetAddr(amx, params[7], &count_addr);
		*count_addr = written;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetReachableBoundingBox(AMX* amx, cell* params)
	{
		if (params[0] < (6 * 4))
		{
			std::cout << __func__ << ": Expecting at least 6 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto source_id = static_cast<int>(params[1]);
		const auto source = Container::Nodes::Find(source_id);
		if (source == nullptr) return GPS_ERROR_INVALID_NODE;

		const float max_dist = amx_ctof(params[2]);
		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (7 * 4))
		{
			type = static_cast<PathType>(params[7]);
		}

		std::vector<int> reached = Pathfinder::GetReachableNodes(source, max_dist, false, 60.0f, type);
		if (reached.empty())
		{
			return GPS_ERROR_INVALID_NODE;
		}

		float minx = std::numeric_limits<float>::infinity();
		float miny = std::numeric_limits<float>::infinity();
		float maxx = -std::numeric_limits<float>::infinity();
		float maxy = -std::numeric_limits<float>::infinity();

		for (int id : reached)
		{
			auto node = Container::Nodes::Find(id);
			if (node != nullptr)
			{
				minx = std::min(minx, node->getX());
				miny = std::min(miny, node->getY());
				maxx = std::max(maxx, node->getX());
				maxy = std::max(maxy, node->getY());
			}
		}

		cell* addr = nullptr;
		amx_GetAddr(amx, params[3], &addr);
		*addr = amx_ftoc(minx);

		amx_GetAddr(amx, params[4], &addr);
		*addr = amx_ftoc(miny);

		amx_GetAddr(amx, params[5], &addr);
		*addr = amx_ftoc(maxx);

		amx_GetAddr(amx, params[6], &addr);
		*addr = amx_ftoc(maxy);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPointZoneName(AMX* amx, cell* params)
	{
		if (params[0] < (5 * 4))
		{
			std::cout << __func__ << ": Expecting at least 5 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const float x = amx_ctof(params[1]);
		const float y = amx_ctof(params[2]);
		const float z = amx_ctof(params[3]);
		const int max_len = static_cast<int>(params[5]);

		std::string zone = Zones::GetZoneNameAt(x, y, z);
		amx_SetCppString(amx, params[4], zone, max_len);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeZoneName(AMX* amx, cell* params)
	{
		if (params[0] < (2 * 4))
		{
			std::cout << __func__ << ": Expecting at least 2 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto node_id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(node_id);
		if (node == nullptr) return GPS_ERROR_INVALID_NODE;

		int max_len = 32;
		if (params[0] >= (3 * 4))
		{
			max_len = static_cast<int>(params[3]);
		}

		std::string zone = Zones::GetZoneNameAt(node->getX(), node->getY(), node->getZ());
		amx_SetCppString(amx, params[2], zone, max_len);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathZonesSummary(AMX* amx, cell* params)
	{
		if (params[0] < (4 * 4))
		{
			std::cout << __func__ << ": Expecting at least 4 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const int max_zones = static_cast<int>(params[3]);
		int max_zone_len = 32;
		if (params[0] >= (5 * 4))
		{
			max_zone_len = static_cast<int>(params[5]);
		}

		std::vector<Node*> node_vec(path->getNodes().begin(), path->getNodes().end());
		std::vector<std::string> zone_names = Zones::GetPathZones(node_vec);

		const int count = std::min(static_cast<int>(zone_names.size()), max_zones);
		cell* base_addr = nullptr;
		amx_GetAddr(amx, params[2], &base_addr);

		for (int i = 0; i < count; ++i)
		{
			amx_SetCppString(amx, params[2] + (i * max_zone_len * sizeof(cell)), zone_names[i], max_zone_len);
		}

		cell* count_addr = nullptr;
		amx_GetAddr(amx, params[4], &count_addr);
		*count_addr = count;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetPathAutoDestroy(AMX* amx, cell* params)
	{
		if (params[0] < (1 * 4))
		{
			std::cout << __func__ << ": Expecting at least 1 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		uint32_t ttl_ms = 60000;
		if (params[0] >= (2 * 4))
		{
			ttl_ms = static_cast<uint32_t>(params[2]);
		}

		path->setTTL(ttl_ms);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SavePathToFile(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const auto filename = amx_GetCppString(amx, params[2]);
		std::ofstream out("scriptfiles/" + filename);
		if (!out.good()) return GPS_ERROR_INTERNAL;

		const auto& nodes = path->getNodes();
		out << nodes.size() << "\n";
		for (auto node : nodes)
		{
			out << node->getID() << " ";
		}
		out << "\n";
		out.close();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL LoadPathFromFile(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto filename = amx_GetCppString(amx, params[1]);
		std::ifstream in("scriptfiles/" + filename);
		if (!in.good()) return GPS_ERROR_INTERNAL;

		size_t count = 0;
		if (!(in >> count) || count == 0) return GPS_ERROR_INVALID_PATH;

		auto path = new Path(0.0f);
		float total_len = 0.0f;
		Node* prev = nullptr;

		for (size_t i = 0; i < count; ++i)
		{
			int node_id = -1;
			if (!(in >> node_id)) break;

			Node* node = Container::Nodes::Find(node_id);
			if (node != nullptr)
			{
				if (prev != nullptr)
				{
					total_len += prev->getDistanceFromNode(node);
				}
				path->addNodeToEnd(node);
				prev = node;
			}
		}

		in.close();

		if (path->getNodes().empty())
		{
			delete path;
			return GPS_ERROR_INVALID_PATH;
		}

		path->setLength(total_len);
		int idx = 0;
		for (auto n : path->getNodes())
		{
			n->addToPath(path, idx++);
		}

		const int id = Container::Paths::Add(path);

		cell* addr = nullptr;
		amx_GetAddr(amx, params[2], &addr);
		*addr = id;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL ExportPathToString(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const int max_len = static_cast<int>(params[3]);
		std::string str;
		const auto& nodes = path->getNodes();
		for (size_t i = 0; i < nodes.size(); ++i)
		{
			if (i > 0) str += ",";
			str += std::to_string(nodes[i]->getID());
		}

		amx_SetCppString(amx, params[2], str, max_len);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL ImportPathFromString(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto str = amx_GetCppString(amx, params[1]);
		if (str.empty()) return GPS_ERROR_INVALID_PARAMS;

		std::stringstream ss(str);
		std::string item;
		auto path = new Path(0.0f);
		float total_len = 0.0f;
		Node* prev = nullptr;

		while (std::getline(ss, item, ','))
		{
			if (item.empty()) continue;
			int id = std::stoi(item);
			Node* node = Container::Nodes::Find(id);
			if (node != nullptr)
			{
				if (prev != nullptr)
				{
					total_len += prev->getDistanceFromNode(node);
				}
				path->addNodeToEnd(node);
				prev = node;
			}
		}

		if (path->getNodes().empty())
		{
			delete path;
			return GPS_ERROR_INVALID_PATH;
		}

		path->setLength(total_len);
		int idx = 0;
		for (auto n : path->getNodes())
		{
			n->addToPath(path, idx++);
		}

		const int id = Container::Paths::Add(path);

		cell* addr = nullptr;
		amx_GetAddr(amx, params[2], &addr);
		*addr = id;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindPathForVehicleCat(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << __func__ << ": Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		auto cat = VEHICLE_CAT_STANDARD;
		if (params[0] >= (4 * 4))
		{
			cat = static_cast<VehicleCategory>(params[4]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (5 * 4))
		{
			pref = static_cast<RoutePreference>(params[5]);
		}

		const auto result = Pathfinder::FindPath(start, target, PATH_TYPE_VEHICLE, pref, cat);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = result;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetMapNodeDanger(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);
		const auto id = static_cast<int>(params[1]);
		const auto weight = amx_ctof(params[2]);
		Container::Nodes::SetDangerWeight(id, weight);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeDanger(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const auto id = static_cast<int>(params[1]);
		float danger = Container::Nodes::GetDangerWeight(id);
		return amx_ftoc(danger);
	}


	cell AMX_NATIVE_CALL SetAreaDangerWeight(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const auto minx = amx_ctof(params[1]);
		const auto miny = amx_ctof(params[2]);
		const auto maxx = amx_ctof(params[3]);
		const auto maxy = amx_ctof(params[4]);
		const auto weight = amx_ctof(params[5]);
		Container::Nodes::SetAreaDangerWeight(minx, miny, maxx, maxy, weight);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL ResetDangerWeights(AMX* amx, cell* params)
	{
		Container::Nodes::ResetDangerWeights();
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindSafePath(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << __func__ << ": Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		float danger_penalty = 5.0f;
		if (params[0] >= (4 * 4))
		{
			danger_penalty = amx_ctof(params[4]);
		}

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (5 * 4))
		{
			type = static_cast<PathType>(params[5]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (6 * 4))
		{
			pref = static_cast<RoutePreference>(params[6]);
		}

		const auto result = Pathfinder::FindSafePath(start, target, danger_penalty, type, pref);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = result;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindEcoPath(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << __func__ << ": Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		float max_slope = 0.12f;
		if (params[0] >= (4 * 4))
		{
			max_slope = amx_ctof(params[4]);
		}

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (5 * 4))
		{
			type = static_cast<PathType>(params[5]);
		}

		const auto result = Pathfinder::FindEcoPath(start, target, max_slope, type);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = result;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathElevationStats(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		float min_z = 0.0f, max_z = 0.0f, climb = 0.0f, max_incline = 0.0f;
		if (!Pathfinder::GetPathElevationStats(path, min_z, max_z, climb, max_incline))
		{
			return GPS_ERROR_INVALID_PATH;
		}

		cell* addr = nullptr;
		amx_GetAddr(amx, params[2], &addr);
		*addr = amx_ftoc(min_z);

		amx_GetAddr(amx, params[3], &addr);
		*addr = amx_ftoc(max_z);

		amx_GetAddr(amx, params[4], &addr);
		*addr = amx_ftoc(climb);

		amx_GetAddr(amx, params[5], &addr);
		*addr = amx_ftoc(max_incline);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL RegisterTunnelArea(AMX* amx, cell* params)
	{
		CHECK_PARAMS(7);
		const float minx = amx_ctof(params[1]);
		const float miny = amx_ctof(params[2]);
		const float minz = amx_ctof(params[3]);
		const float maxx = amx_ctof(params[4]);
		const float maxy = amx_ctof(params[5]);
		const float maxz = amx_ctof(params[6]);

		int id = Tunnels::RegisterTunnelArea(minx, miny, minz, maxx, maxy, maxz);

		cell* addr = nullptr;
		amx_GetAddr(amx, params[7], &addr);
		*addr = id;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL UnregisterTunnelArea(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const int id = static_cast<int>(params[1]);
		return Tunnels::UnregisterTunnelArea(id) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL ClearCustomTunnelAreas(AMX* amx, cell* params)
	{
		Tunnels::ClearCustomTunnelAreas();
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsPointInTunnel(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);
		const float x = amx_ctof(params[1]);
		const float y = amx_ctof(params[2]);
		const float z = amx_ctof(params[3]);
		return Tunnels::IsPointInTunnel(x, y, z) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL IsMapNodeUnderground(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const auto id = static_cast<int>(params[1]);
		auto node = Container::Nodes::Find(id);
		return Tunnels::IsMapNodeUnderground(node) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL GetPathTunnelSegments(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);
		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		float entry = 0.0f, exit = 0.0f;
		bool has_tunnel = Tunnels::GetPathTunnelSegments(path, entry, exit);

		cell* addr = nullptr;
		amx_GetAddr(amx, params[2], &addr);
		*addr = amx_ftoc(entry);

		amx_GetAddr(amx, params[3], &addr);
		*addr = amx_ftoc(exit);

		return has_tunnel ? 1 : 0;
	}


	cell AMX_NATIVE_CALL FindNearestPointByRoad(AMX* amx, cell* params)
	{
		if (params[0] < (7 * 4))
		{
			std::cout << __func__ << ": Expecting at least 7 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const float from_x = amx_ctof(params[1]);
		const float from_y = amx_ctof(params[2]);
		const float from_z = amx_ctof(params[3]);

		cell* points_addr = nullptr;
		amx_GetAddr(amx, params[4], &points_addr);
		const int total_points = static_cast<int>(params[5]);

		if (points_addr == nullptr || total_points <= 0)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		std::vector<std::tuple<float, float, float>> destinations;
		destinations.reserve(total_points);

		for (int i = 0; i < total_points; ++i)
		{
			cell* sub_addr = nullptr;
			amx_GetAddr(amx, points_addr[i], &sub_addr);
			if (sub_addr != nullptr)
			{
				destinations.push_back({ amx_ctof(sub_addr[0]), amx_ctof(sub_addr[1]), amx_ctof(sub_addr[2]) });
			}
			else
			{
				destinations.push_back({ 0.0f, 0.0f, 0.0f });
			}
		}

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (9 * 4))
		{
			type = static_cast<PathType>(params[9]);
		}

		int nearest_idx = -1;
		float road_dist = 0.0f;
		int path_id = INVALID_PATH_ID;

		int status = Pathfinder::FindNearestPointByRoad(from_x, from_y, from_z, destinations, nearest_idx, road_dist, path_id, type);
		if (status != 0)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		cell* out_idx_addr = nullptr;
		amx_GetAddr(amx, params[6], &out_idx_addr);
		*out_idx_addr = nearest_idx;

		cell* out_dist_addr = nullptr;
		amx_GetAddr(amx, params[7], &out_dist_addr);
		*out_dist_addr = amx_ftoc(road_dist);

		if (params[0] >= (8 * 4))
		{
			cell* out_path_addr = nullptr;
			amx_GetAddr(amx, params[8], &out_path_addr);
			if (out_path_addr != nullptr)
			{
				*out_path_addr = path_id;
			}
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL RegisterRoadRadar(AMX* amx, cell* params)
	{
		if (params[0] < (5 * 4))
		{
			std::cout << __func__ << ": Expecting at least 5 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const int id = static_cast<int>(params[1]);
		const float x = amx_ctof(params[2]);
		const float y = amx_ctof(params[3]);
		const float z = amx_ctof(params[4]);
		const float speed_limit = amx_ctof(params[5]);
		float alert_radius = 300.0f;
		if (params[0] >= (6 * 4))
		{
			alert_radius = amx_ctof(params[6]);
		}

		Radars::RegisterRadar(id, x, y, z, speed_limit, alert_radius);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL UnregisterRoadRadar(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const int id = static_cast<int>(params[1]);
		return Radars::UnregisterRadar(id) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL ClearRoadRadars(AMX* amx, cell* params)
	{
		Radars::ClearRadars();
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetNextPathRadar(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const float current_dist = amx_ctof(params[2]);
		int radar_id = -1;
		float dist_to_radar = 0.0f;
		float speed_limit = 0.0f;

		bool found = Radars::GetNextPathRadar(path, current_dist, radar_id, dist_to_radar, speed_limit);

		cell* addr = nullptr;
		amx_GetAddr(amx, params[3], &addr);
		*addr = radar_id;

		amx_GetAddr(amx, params[4], &addr);
		*addr = amx_ftoc(dist_to_radar);

		amx_GetAddr(amx, params[5], &addr);
		*addr = amx_ftoc(speed_limit);

		return found ? 1 : 0;
	}


	cell AMX_NATIVE_CALL GetPathOffsetPosition(AMX* amx, cell* params)
	{
		CHECK_PARAMS(7);
		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const float dist = amx_ctof(params[2]);
		const float offset = amx_ctof(params[3]);

		float ox = 0.0f, oy = 0.0f, oz = 0.0f, oang = 0.0f;
		if (!Pathfinder::GetPathOffsetPosition(path, dist, offset, ox, oy, oz, oang))
		{
			return GPS_ERROR_INVALID_PATH;
		}

		cell* addr = nullptr;
		amx_GetAddr(amx, params[4], &addr);
		*addr = amx_ftoc(ox);

		amx_GetAddr(amx, params[5], &addr);
		*addr = amx_ftoc(oy);

		amx_GetAddr(amx, params[6], &addr);
		*addr = amx_ftoc(oz);

		amx_GetAddr(amx, params[7], &addr);
		*addr = amx_ftoc(oang);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsMapNodeDeadEnd(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const auto id = static_cast<int>(params[1]);
		auto node = Container::Nodes::Find(id);
		return Pathfinder::IsMapNodeDeadEnd(node) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL FindEvasivePath(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << __func__ << ": Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (4 * 4))
		{
			type = static_cast<PathType>(params[4]);
		}

		const auto result = Pathfinder::FindEvasivePath(start, target, type);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = result;

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetConnectionToll(AMX* amx, cell* params)
	{
		if (params[0] < (2 * 4))
		{
			std::cout << __func__ << ": Expecting at least 2 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto id = static_cast<int>(params[1]);
		const bool has_toll = params[2] != 0;
		int cost = 0;
		if (params[0] >= (3 * 4))
		{
			cost = static_cast<int>(params[3]);
		}

		Container::Connections::SetToll(id, has_toll, cost);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionToll(AMX* amx, cell* params)
	{
		if (params[0] < (1 * 4))
		{
			std::cout << __func__ << ": Expecting at least 1 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto id = static_cast<int>(params[1]);
		int cost = 0;
		bool has_toll = Container::Connections::GetToll(id, cost);

		if (params[0] >= (2 * 4))
		{
			cell* addr = nullptr;
			amx_GetAddr(amx, params[2], &addr);
			if (addr != nullptr)
			{
				*addr = cost;
			}
		}

		return has_toll ? 1 : 0;
	}


	cell AMX_NATIVE_CALL FindPathAvoidTolls(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << __func__ << ": Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (4 * 4))
		{
			type = static_cast<PathType>(params[4]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (5 * 4))
		{
			pref = static_cast<RoutePreference>(params[5]);
		}

		const auto result = Pathfinder::FindPathAvoidTolls(start, target, type, pref);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		cell* address = nullptr;
		amx_GetAddr(amx, params[3], &address);
		*address = result;

		return GPS_ERROR_NONE;
	}
}

AMX_NATIVE_INFO PluginNatives[] =
{
	{"CreateMapNode",					Natives::CreateMapNode},
	{"DestroyMapNode",					Natives::DestroyMapNode},
	{"IsValidMapNode", 					Natives::IsValidMapNode},
	{"GetMapNodePos",					Natives::GetMapNodePos},

	{"GetMapNodeType",					Natives::GetMapNodeType},
	{"GetMapNodeFlags",					Natives::GetMapNodeFlags},

	{"CreateConnection",				Natives::CreateConnection},
	{"DestroyConnection",				Natives::DestroyConnection},
	{"GetConnectionSource", 			Natives::GetConnectionSource},
	{"GetConnectionTarget",				Natives::GetConnectionTarget},

	{"GetMapNodeConnectionCount",		Natives::GetMapNodeConnectionCount},
	{"GetMapNodeConnection",			Natives::GetMapNodeConnection},
	{"GetConnectionBetweenMapNodes",	Natives::GetConnectionBetweenMapNodes},

	{"GetConnectionLaneInfo",			Natives::GetConnectionLaneInfo},
	{"GetConnectionLanePos",			Natives::GetConnectionLanePos},
	{"IsConnectionOneWay",				Natives::IsConnectionOneWay},

	{"GetDistanceBetweenMapNodes",		Natives::GetDistanceBetweenMapNodes},
	{"GetAngleBetweenMapNodes",			Natives::GetAngleBetweenMapNodes},
	{"GetMapNodeDistanceFromPoint",		Natives::GetMapNodeDistanceFromPoint},
	{"GetMapNodeAngleFromPoint",		Natives::GetMapNodeAngleFromPoint},
	{"GetClosestMapNodeToPoint",		Natives::GetClosestMapNodeToPoint},

	{"GetHighestMapNodeID",				Natives::GetHighestMapNodeID},
	{"GetRandomMapNode",				Natives::GetRandomMapNode},
	{"GetRandomMapNodeFromArea",		Natives::GetRandomMapNodeFromArea},
	{"GetRandomMapNodeInArea",			Natives::GetRandomMapNodeFromArea},

	{"FindPath",						Natives::FindPath},
	{"FindPathThreaded",				Natives::FindPathThreaded},
	{"FindPathThreadedEx",				Natives::FindPathThreadedEx},
	{"FindMultiStopPath",				Natives::FindMultiStopPath},

	{"SaveMapNodesToFile",				Natives::SaveMapNodesToFile},

	{"IsValidPath", 					Natives::IsValidPath},
	{"GetPathSize",						Natives::GetPathSize},
	{"GetPathLength",					Natives::GetPathLength},
	{"GetPathNode",						Natives::GetPathNode},
	{"GetPathNodeIndex",				Natives::GetPathNodeIndex},
	{"GetPathLaneNodePos",				Natives::GetPathLaneNodePos},
	{"DestroyPath",						Natives::DestroyPath},

	{"SetMapNodeBlocked",				Natives::SetMapNodeBlocked},
	{"IsMapNodeBlocked",				Natives::IsMapNodeBlocked},
	{"SetConnectionBlocked",			Natives::SetConnectionBlocked},
	{"IsConnectionBlocked",				Natives::IsConnectionBlocked},

	{"GetClosestPointOnRoad",			Natives::GetClosestPointOnRoad},
	{"GetRoadPositionInfo",				Natives::GetRoadPositionInfo},
	{"IsPointAgainstTraffic",			Natives::IsPointAgainstTraffic},

	{"GetPathTurnDirection",			Natives::GetPathTurnDirection},
	{"GetPathEstimatedTravelTime",		Natives::GetPathEstimatedTravelTime},

	{"GetPathProgress",					Natives::GetPathProgress},
	{"GetPathRemainingInfo",			Natives::GetPathRemainingInfo},
	{"GetPathNextManeuver",				Natives::GetPathNextManeuver},
	{"GetSimplifiedPathNodes",			Natives::GetSimplifiedPathNodes},
	{"IsMapNodeIntersection",			Natives::IsMapNodeIntersection},
	{"GetMapNodeConnectionTarget",		Natives::GetMapNodeConnectionTarget},

	{"SetConnectionTrafficMultiplier",	Natives::SetConnectionTrafficMultiplier},
	{"GetConnectionTrafficMultiplier",	Natives::GetConnectionTrafficMultiplier},
	{"SetAreaTrafficMultiplier",		Natives::SetAreaTrafficMultiplier},
	{"ResetTrafficMultipliers",			Natives::ResetTrafficMultipliers},

	{"FindInterceptPath",				Natives::FindInterceptPath},
	{"GetPurePursuitSteering",			Natives::GetPurePursuitSteering},

	{"GetReachableMapNodes",			Natives::GetReachableMapNodes},
	{"GetReachableBoundingBox",			Natives::GetReachableBoundingBox},

	{"GetPointZoneName",				Natives::GetPointZoneName},
	{"GetMapNodeZoneName",				Natives::GetMapNodeZoneName},
	{"GetPathZonesSummary",				Natives::GetPathZonesSummary},

	{"SetPathAutoDestroy",				Natives::SetPathAutoDestroy},

	{"SavePathToFile",					Natives::SavePathToFile},
	{"LoadPathFromFile",				Natives::LoadPathFromFile},
	{"ExportPathToString",				Natives::ExportPathToString},
	{"ImportPathFromString",			Natives::ImportPathFromString},

	{"FindPathForVehicleCat",			Natives::FindPathForVehicleCat},

	{"SetMapNodeDanger",				Natives::SetMapNodeDanger},
	{"GetMapNodeDanger",				Natives::GetMapNodeDanger},
	{"SetAreaDangerWeight",				Natives::SetAreaDangerWeight},
	{"ResetDangerWeights",				Natives::ResetDangerWeights},
	{"FindSafePath",					Natives::FindSafePath},

	{"FindEcoPath",						Natives::FindEcoPath},
	{"GetPathElevationStats",			Natives::GetPathElevationStats},

	{"RegisterTunnelArea",				Natives::RegisterTunnelArea},
	{"UnregisterTunnelArea",			Natives::UnregisterTunnelArea},
	{"ClearCustomTunnelAreas",			Natives::ClearCustomTunnelAreas},
	{"IsPointInTunnel",					Natives::IsPointInTunnel},
	{"IsMapNodeUnderground",			Natives::IsMapNodeUnderground},
	{"GetPathTunnelSegments",			Natives::GetPathTunnelSegments},

	{"FindNearestPointByRoad",			Natives::FindNearestPointByRoad},

	{"RegisterRoadRadar",				Natives::RegisterRoadRadar},
	{"UnregisterRoadRadar",				Natives::UnregisterRoadRadar},
	{"ClearRoadRadars",					Natives::ClearRoadRadars},
	{"GetNextPathRadar",				Natives::GetNextPathRadar},

	{"GetPathOffsetPosition",			Natives::GetPathOffsetPosition},

	{"IsMapNodeDeadEnd",				Natives::IsMapNodeDeadEnd},
	{"FindEvasivePath",					Natives::FindEvasivePath},

	{"SetConnectionToll",				Natives::SetConnectionToll},
	{"GetConnectionToll",				Natives::GetConnectionToll},
	{"FindPathAvoidTolls",				Natives::FindPathAvoidTolls},

	{0, 0}
};
