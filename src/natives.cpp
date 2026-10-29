#include <map>
#include <thread>
#include <iostream>
#include <random>
#include <algorithm>

#include "natives.h"
#include "callback.h"
#include "connection.h"
#include "node.h"
#include "path.h"
#include "file.h"
#include "amx_handler.h"


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
		CHECK_PARAMS(0);

		return Container::Nodes::GetHighestID();
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


	cell AMX_NATIVE_CALL GetClosestPedMapNodeToPoint(AMX* amx, cell* params)
	{
		if (params[0] < (4 * 4))
		{
			std::cout << __func__ << ": Expecting at least 4 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		return FindClosestNodeInternal(amx, params, PATH_TYPE_PEDESTRIAN);
	}


	cell AMX_NATIVE_CALL GetClosestBoatMapNodeToPoint(AMX* amx, cell* params)
	{
		if (params[0] < (4 * 4))
		{
			std::cout << __func__ << ": Expecting at least 4 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		return FindClosestNodeInternal(amx, params, PATH_TYPE_BOAT);
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

		const auto result = Pathfinder::FindPath(start, target, type);

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


	cell AMX_NATIVE_CALL FindPedPath(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		return FindPathInternalWrapper(amx, params, PATH_TYPE_PEDESTRIAN);
	}


	cell AMX_NATIVE_CALL FindBoatPath(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		return FindPathInternalWrapper(amx, params, PATH_TYPE_BOAT);
	}


	static cell FindPathThreadedInternal(AMX* amx, cell* params, PathType type, int format_param_index)
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
			std::thread t(Pathfinder::FindPathThreaded, start, target, callback, type);
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

		return FindPathThreadedInternal(amx, params, PATH_TYPE_VEHICLE, 4);
	}


	cell AMX_NATIVE_CALL FindPathThreadedEx(AMX* amx, cell* params)
	{
		if (params[0] < (4 * 4))
		{
			std::cout << "FindPathThreadedEx: Expecting at least 4 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto type = static_cast<PathType>(params[3]);

		return FindPathThreadedInternal(amx, params, type, 5);
	}


	cell AMX_NATIVE_CALL FindPedPathThreaded(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << "FindPedPathThreaded: Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		return FindPathThreadedInternal(amx, params, PATH_TYPE_PEDESTRIAN, 4);
	}


	cell AMX_NATIVE_CALL FindBoatPathThreaded(AMX* amx, cell* params)
	{
		if (params[0] < (3 * 4))
		{
			std::cout << "FindBoatPathThreaded: Expecting at least 3 parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n";
			return GPS_ERROR_INVALID_PARAMS;
		}

		return FindPathThreadedInternal(amx, params, PATH_TYPE_BOAT, 4);
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

	{"GetClosestPedMapNodeToPoint",		Natives::GetClosestPedMapNodeToPoint},
	{"GetClosestBoatMapNodeToPoint",	Natives::GetClosestBoatMapNodeToPoint},

	{"GetHighestMapNodeID",				Natives::GetHighestMapNodeID},
	{"GetRandomMapNode",				Natives::GetRandomMapNode},
	{"GetRandomMapNodeFromArea",		Natives::GetRandomMapNodeFromArea},
	{"GetRandomMapNodeInArea",			Natives::GetRandomMapNodeFromArea},

	{"FindPath",						Natives::FindPath},
	{"FindPathThreaded",				Natives::FindPathThreaded},
	{"FindPathThreadedEx",				Natives::FindPathThreadedEx},

	{"FindPedPath",						Natives::FindPedPath},
	{"FindPedPathThreaded",				Natives::FindPedPathThreaded},

	{"FindBoatPath",					Natives::FindBoatPath},
	{"FindBoatPathThreaded",			Natives::FindBoatPathThreaded},

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

	{0, 0}
};
