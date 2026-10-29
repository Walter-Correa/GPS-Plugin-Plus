#pragma once

#include "pathfinder.h"


enum
{
	GPS_ERROR_NONE = 0,
	GPS_ERROR_INVALID_PARAMS = -1,
	GPS_ERROR_INVALID_PATH = -2,
	GPS_ERROR_INVALID_NODE = -3,
	GPS_ERROR_INVALID_CONNECTION = -4,
	GPS_ERROR_INTERNAL = -5
};


#define CHECK_PARAMS(n) \
	if (params[0] != (n * 4)) \
	{ \
		std::cout << __func__ << ": Expecting " << n << " parameter(s), but found " << (params[0] / sizeof(cell)) << ".\n"; \
		return GPS_ERROR_INVALID_PARAMS; \
	}


namespace Natives 
{
	cell AMX_NATIVE_CALL CreateMapNode(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL DestroyMapNode(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL IsValidMapNode(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetMapNodePos(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL GetMapNodeType(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetMapNodeFlags(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL CreateConnection(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL DestroyConnection(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetConnectionSource(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetConnectionTarget(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL GetMapNodeConnectionCount(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetMapNodeConnection(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetConnectionBetweenMapNodes(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL GetConnectionLaneInfo(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetConnectionLanePos(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL IsConnectionOneWay(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL GetDistanceBetweenMapNodes(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetAngleBetweenMapNodes(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetMapNodeDistanceFromPoint(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetMapNodeAngleFromPoint(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetClosestMapNodeToPoint(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL GetClosestPedMapNodeToPoint(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetClosestBoatMapNodeToPoint(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL GetHighestMapNodeID(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetRandomMapNode(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetRandomMapNodeFromArea(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL SaveMapNodesToFile(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL FindPath(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL FindPathThreaded(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL FindPathThreadedEx(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL FindPedPath(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL FindPedPathThreaded(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL FindBoatPath(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL FindBoatPathThreaded(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL IsValidPath(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetPathSize(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetPathLength(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetPathNode(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetPathNodeIndex(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetPathLaneNodePos(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL DestroyPath(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL SetMapNodeBlocked(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL IsMapNodeBlocked(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL SetConnectionBlocked(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL IsConnectionBlocked(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL GetClosestPointOnRoad(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetRoadPositionInfo(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL IsPointAgainstTraffic(AMX* amx, cell* params);

	cell AMX_NATIVE_CALL GetPathTurnDirection(AMX* amx, cell* params);
	cell AMX_NATIVE_CALL GetPathEstimatedTravelTime(AMX* amx, cell* params);
}
