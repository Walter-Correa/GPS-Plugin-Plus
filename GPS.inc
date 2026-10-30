#if defined _inc_gps
    #undef _inc_gps
#endif

#if defined _gps_included
    #endinput
#endif
#define _gps_included


const MapNode:GPS_INVALID_MAP_NODE_ID = MapNode:-1;
const Path:GPS_INVALID_PATH_ID = Path:-1;
const Connection:GPS_INVALID_CONNECTION_ID = Connection:-1;

#define INVALID_MAP_NODE_ID             GPS_INVALID_MAP_NODE_ID
#define INVALID_GPS_PATH_ID             GPS_INVALID_PATH_ID
#define INVALID_CONNECTION_ID           GPS_INVALID_CONNECTION_ID

enum _:GPS_ERROR 
{
    GPS_ERROR_NONE                  =  0,
    GPS_ERROR_INVALID_PARAMS        = -1,
    GPS_ERROR_INVALID_PATH          = -2,
    GPS_ERROR_INVALID_NODE          = -3,
    GPS_ERROR_INVALID_CONNECTION    = -4,
    GPS_ERROR_INTERNAL              = -5
}

enum PathType
{
    PATH_TYPE_ANY                   = -1,
    PATH_TYPE_VEHICLE               = 0,
    PATH_TYPE_PEDESTRIAN            = 1,
    PATH_TYPE_BOAT                  = 2
}

enum PathNodeFlags (<<= 1)
{
    NODE_FLAG_NONE                  = 0,
    NODE_FLAG_HIGHWAY               = 1,
    NODE_FLAG_EMERGENCY_ONLY,
    NODE_FLAG_ROADBLOCK,
    NODE_FLAG_CROSSWALK,
    NODE_FLAG_ONEWAY
}

enum TurnDirection
{
    TURN_NONE                       = 0,
    TURN_STRAIGHT                   = 1,
    TURN_RIGHT                      = 2,
    TURN_LEFT                       = 3,
    TURN_SHARP_RIGHT                = 4,
    TURN_SHARP_LEFT                 = 5,
    TURN_UTURN                      = 6
}

enum RoadPosition
{
    ROAD_POS_CENTER                 = 0,
    ROAD_POS_CURB                   = 1,
    ROAD_POS_EDGE                   = 2
}


native CreateMapNode(Float:x, Float:y, Float:z, &MapNode:nodeid, PathType:type = PATH_TYPE_VEHICLE, PathNodeFlags:flags = NODE_FLAG_NONE);
native DestroyMapNode(MapNode:nodeid);
native bool:IsValidMapNode(MapNode:nodeid);
native GetMapNodePos(MapNode:nodeid, &Float:x, &Float:y, &Float:z);

native PathType:GetMapNodeType(MapNode:nodeid);
native GetMapNodeFlags(MapNode:nodeid, &PathNodeFlags:flags);

native CreateConnection(MapNode:source, MapNode:target, &Connection:connectionid, Float:road_width = 8.0, left_lanes = 1, right_lanes = 1, bool:one_way = false);
native DestroyConnection(Connection:connectionid);
native GetConnectionSource(Connection:connectionid, &MapNode:nodeid);
native GetConnectionTarget(Connection:connectionid, &MapNode:nodeid);

native GetMapNodeConnectionCount(MapNode:nodeid, &count);
native GetMapNodeConnection(MapNode:nodeid, index, &Connection:connectionid);
native GetConnectionBetweenMapNodes(MapNode:source, MapNode:target, &Connection:connectionid);

native GetConnectionLaneInfo(Connection:connectionid, &Float:road_width, &left_lanes, &right_lanes);
native GetConnectionLanePos(Connection:connectionid, lane_index, &Float:x, &Float:y, &Float:z);
native bool:IsConnectionOneWay(Connection:connectionid);

native GetDistanceBetweenMapNodes(MapNode:first, MapNode:second, &Float:distance);
native GetAngleBetweenMapNodes(MapNode:first, MapNode:second, &Float:angle, bool:facing_angle = false);
native GetMapNodeDistanceFromPoint(MapNode:nodeid, Float:x, Float:y, Float:z, &Float:distance);
native GetMapNodeAngleFromPoint(MapNode:nodeid, Float:x, Float:y, &Float:angle, bool:facing_angle = false);
native GetClosestMapNodeToPoint(Float:x, Float:y, Float:z, &MapNode:nodeid, MapNode:ignorednode = GPS_INVALID_MAP_NODE_ID, PathType:type = PATH_TYPE_VEHICLE);

native GetClosestPedMapNodeToPoint(Float:x, Float:y, Float:z, &MapNode:nodeid, MapNode:ignorednode = GPS_INVALID_MAP_NODE_ID);
native GetClosestBoatMapNodeToPoint(Float:x, Float:y, Float:z, &MapNode:nodeid, MapNode:ignorednode = GPS_INVALID_MAP_NODE_ID);

native GetHighestMapNodeID(PathType:type = PATH_TYPE_VEHICLE);
native GetRandomMapNode(&MapNode:nodeid, PathType:type = PATH_TYPE_VEHICLE, Float:minz = -99999.0, Float:maxz = 99999.0);
native GetRandomMapNodeFromArea(Float:minx, Float:miny, Float:maxx, Float:maxy, &MapNode:nodeid, PathType:type = PATH_TYPE_VEHICLE, Float:minz = -99999.0, Float:maxz = 99999.0);

native SaveMapNodesToFile(const filename[]);

native FindPath(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE);
native FindPathThreaded(MapNode:source, MapNode:target, const callback[], const format[] = "", {Float, _}:...);
native FindPathThreadedEx(MapNode:source, MapNode:target, PathType:type, const callback[], const format[] = "", {Float, _}:...);

native FindPedPath(MapNode:source, MapNode:target, &Path:pathid);
native FindPedPathThreaded(MapNode:source, MapNode:target, const callback[], const format[] = "", {Float, _}:...);

native FindBoatPath(MapNode:source, MapNode:target, &Path:pathid);
native FindBoatPathThreaded(MapNode:source, MapNode:target, const callback[], const format[] = "", {Float, _}:...);

native bool:IsValidPath(Path:pathid);
native GetPathSize(Path:pathid, &size);
native GetPathLength(Path:pathid, &Float:length);
native GetPathNode(Path:pathid, index, &MapNode:nodeid);
native GetPathNodeIndex(Path:pathid, MapNode:nodeid, &index);
native GetPathLaneNodePos(Path:pathid, index, lane_index, &Float:x, &Float:y, &Float:z);
native DestroyPath(Path:pathid);

native SetMapNodeBlocked(MapNode:nodeid, bool:blocked);
native bool:IsMapNodeBlocked(MapNode:nodeid);

native SetConnectionBlocked(Connection:connectionid, bool:blocked);
native bool:IsConnectionBlocked(Connection:connectionid);

native GetClosestPointOnRoad(Float:x, Float:y, Float:z, &Float:out_x, &Float:out_y, &Float:out_z, &Float:angle, &Connection:connectionid = INVALID_CONNECTION_ID, PathType:type = PATH_TYPE_VEHICLE);
native GetRoadPositionInfo(Float:x, Float:y, Float:z, &Float:out_x, &Float:out_y, &Float:out_z, &Float:angle, &Float:road_width, RoadPosition:pos = ROAD_POS_CURB, PathType:type = PATH_TYPE_VEHICLE);
native IsPointAgainstTraffic(Float:x, Float:y, Float:z, Float:facing_angle, &bool:against_traffic);

native GetPathTurnDirection(Path:pathid, index, &TurnDirection:turn, &Float:turn_angle);
native GetPathEstimatedTravelTime(Path:pathid, Float:average_speed_kmh, &Float:seconds);


#if defined _PawnPlus_included
    stock Task:FindPathAsync(MapNode:source, MapNode:target) 
    {
        new Task:task = task_new();

        if (FindPathThreaded(source, target, "FindPathAsyncResponse", "i", _:task) != GPS_ERROR_NONE) 
        {
            task_set_error_ticks(task, amx_err_exit, 0);
        }

        return task;
    }

    forward public FindPathAsyncResponse(Path:path, Task:task);
    public FindPathAsyncResponse(Path:path, Task:task) 
    {
        task_set_result(task, _:path);
    }
#endif
