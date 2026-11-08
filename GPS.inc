#if defined _inc_gps
    #undef _inc_gps
#endif

#if defined _gps_included
    #endinput
#endif
#define _gps_included

#define GPS_PLUGIN_VERSION_MAJOR        2
#define GPS_PLUGIN_VERSION_MINOR        0
#define GPS_PLUGIN_VERSION_PATCH        0
#define GPS_PLUGIN_VERSION_STRING       "2.0.0"


const MapNode:GPS_INVALID_MAP_NODE_ID = MapNode:-1;
const Path:GPS_INVALID_PATH_ID = Path:-1;
const Connection:GPS_INVALID_CONNECTION_ID = Connection:-1;

#define INVALID_MAP_NODE_ID             GPS_INVALID_MAP_NODE_ID
#define INVALID_GPS_PATH_ID             GPS_INVALID_PATH_ID
#if !defined INVALID_PATH_ID
    #define INVALID_PATH_ID             GPS_INVALID_PATH_ID
#endif
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

// PathNodeFlags represents bitmask attributes of a map node.
// Note: In original GPS.dat data, flags 2 (EMERGENCY_ONLY) and 4 (ROADBLOCK) are present on
// thousands of normal road nodes as dataset classification and do NOT automatically block
// pathfinding. Use SetMapNodeBlocked() if you wish to block routing through specific nodes.
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

enum RoutePreference
{
    ROUTE_PREF_SHORTEST             = 0,
    ROUTE_PREF_HIGHWAYS             = 1,
    ROUTE_PREF_AVOID_HIGHWAYS       = 2,
    ROUTE_PREF_SAFEST               = 3,
    ROUTE_PREF_ECO                  = 4,
    ROUTE_PREF_EVASIVE              = 5,
    ROUTE_PREF_AVOID_TOLLS          = 6,
    ROUTE_PREF_PAVED_ONLY           = 7
}

enum RoadClass
{
    ROAD_CLASS_ALLEY                = 0,
    ROAD_CLASS_RESIDENTIAL          = 1,
    ROAD_CLASS_AVENUE               = 2,
    ROAD_CLASS_HIGHWAY              = 3
}

enum VehicleCategory
{
    VEHICLE_CAT_STANDARD            = 0,
    VEHICLE_CAT_HEAVY_TRUCK         = 1,
    VEHICLE_CAT_MOTORCYCLE          = 2
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

native GetHighestMapNodeID(PathType:type = PATH_TYPE_VEHICLE);
native GetRandomMapNode(&MapNode:nodeid, PathType:type = PATH_TYPE_VEHICLE, Float:minz = -99999.0, Float:maxz = 99999.0);
native GetRandomMapNodeFromArea(Float:minx, Float:miny, Float:maxx, Float:maxy, &MapNode:nodeid, PathType:type = PATH_TYPE_VEHICLE, Float:minz = -99999.0, Float:maxz = 99999.0);

native SaveMapNodesToFile(const filename[]);

native FindPath(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);
native FindPathThreaded(MapNode:source, MapNode:target, const callback[], const format[] = "", {Float, _}:...);
native FindPathThreadedEx(MapNode:source, MapNode:target, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST, const callback[] = "", const format[] = "", {Float, _}:...);
native FindMultiStopPath(const MapNode:stops[], stops_count, &Path:pathid, bool:optimize_order = false, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);

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

native GetPathProgress(Path:pathid, Float:x, Float:y, Float:z, &current_step, &Float:distance_to_path, &bool:off_route, Float:max_distance_threshold = 25.0);
native GetPathRemainingInfo(Path:pathid, current_step, &Float:remaining_distance, &Float:remaining_seconds, Float:speed_kmh = 60.0);
native GetPathNextManeuver(Path:pathid, Float:x, Float:y, Float:z, &TurnDirection:turn, &Float:distance_to_turn);
native GetSimplifiedPathNodes(Path:pathid, Float:tolerance_meters, MapNode:output_nodes[], max_nodes = sizeof(output_nodes), &nodes_count);

native bool:IsMapNodeIntersection(MapNode:nodeid, &connection_count = 0);
native GetMapNodeConnectionTarget(MapNode:nodeid, index, &MapNode:target_nodeid);

native SetConnectionTrafficMultiplier(Connection:connectionid, Float:multiplier);
native Float:GetConnectionTrafficMultiplier(Connection:connectionid);
native SetAreaTrafficMultiplier(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:multiplier);
native ResetTrafficMultipliers();

native FindInterceptPath(MapNode:pursuer_node, MapNode:target_node, Float:target_speed_kmh, Float:pursuer_speed_kmh, &Path:pathid, &MapNode:intercept_node, &Float:time_to_intercept, PathType:type = PATH_TYPE_VEHICLE);
native GetPurePursuitSteering(Path:pathid, Float:vehicle_x, Float:vehicle_y, Float:vehicle_angle, Float:lookahead_dist, &Float:steer_ratio, &Float:recommended_speed_kmh);

native GetReachableMapNodes(MapNode:source, Float:limit_value, bool:by_time, Float:speed_kmh, MapNode:nodes_out[], max_nodes = sizeof(nodes_out), &count, PathType:type = PATH_TYPE_VEHICLE);
native GetReachableBoundingBox(MapNode:source, Float:max_distance, &Float:minx, &Float:miny, &Float:maxx, &Float:maxy, PathType:type = PATH_TYPE_VEHICLE);

native GetPointZoneName(Float:x, Float:y, Float:z, zone_name[], max_len = sizeof(zone_name));
native GetMapNodeZoneName(MapNode:nodeid, zone_name[], max_len = sizeof(zone_name));
native GetPathZonesSummary(Path:pathid, zones[][], max_zones, &zones_count, max_zone_len = sizeof(zones[]));

native SetPathAutoDestroy(Path:pathid, milliseconds = 60000);

native SavePathToFile(Path:pathid, const filename[]);
native LoadPathFromFile(const filename[], &Path:pathid);
native ExportPathToString(Path:pathid, buffer[], max_len = sizeof(buffer));
native ImportPathFromString(const buffer[], &Path:pathid);

native FindPathForVehicleCat(MapNode:source, MapNode:target, &Path:pathid, VehicleCategory:cat = VEHICLE_CAT_STANDARD, RoutePreference:pref = ROUTE_PREF_SHORTEST);

native SetMapNodeDanger(MapNode:nodeid, Float:danger_weight);
native Float:GetMapNodeDanger(MapNode:nodeid);
native SetAreaDangerWeight(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:danger_weight);
native ResetDangerWeights();
native FindSafePath(MapNode:source, MapNode:target, &Path:pathid, Float:danger_penalty = 5.0, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);

native FindEcoPath(MapNode:source, MapNode:target, &Path:pathid, Float:max_slope_gradient = 0.12, PathType:type = PATH_TYPE_VEHICLE);
native GetPathElevationStats(Path:pathid, &Float:min_z, &Float:max_z, &Float:total_climb, &Float:max_incline_percent);

native RegisterTunnelArea(Float:minx, Float:miny, Float:minz, Float:maxx, Float:maxy, Float:maxz, &tunnel_id);
native bool:UnregisterTunnelArea(tunnel_id);
native ClearCustomTunnelAreas();
native bool:IsPointInTunnel(Float:x, Float:y, Float:z);
native bool:IsMapNodeUnderground(MapNode:nodeid);
native bool:GetPathTunnelSegments(Path:pathid, &Float:entry_distance, &Float:exit_distance);

native FindNearestPointByRoad(Float:from_x, Float:from_y, Float:from_z, const Float:points[][3], total_points, &nearest_index, &Float:road_distance, &Path:pathid = GPS_INVALID_PATH_ID, PathType:type = PATH_TYPE_VEHICLE);

native RegisterRoadRadar(radarid, Float:x, Float:y, Float:z, Float:speed_limit_kmh, Float:alert_radius = 300.0);
native bool:UnregisterRoadRadar(radarid);
native ClearRoadRadars();
native bool:GetNextPathRadar(Path:pathid, Float:current_dist, &radarid, &Float:dist_to_radar, &Float:radar_limit);

native GetPathOffsetPosition(Path:pathid, Float:distance_along_path, Float:lateral_offset, &Float:x, &Float:y, &Float:z, &Float:angle);

native bool:IsMapNodeDeadEnd(MapNode:nodeid);
native FindEvasivePath(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE);

native SetConnectionToll(Connection:connectionid, bool:has_toll, toll_cost = 0);
native bool:GetConnectionToll(Connection:connectionid, &toll_cost = 0);
native FindPathAvoidTolls(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);

native GetPathRoundaboutInfo(Path:pathid, index, &bool:is_roundabout, &exit_number, &Float:roundabout_radius);
native GetSmoothPathPoints(Path:pathid, Float:spacing_meters, Float:out_x[], Float:out_y[], Float:out_z[], max_points = sizeof(out_x), &total_points = 0);

native bool:IsConnectionPaved(Connection:connectionid);
native SetConnectionPaved(Connection:connectionid, bool:paved);
native FindPathPavedOnly(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE);

native RoadClass:GetConnectionRoadClass(Connection:connectionid, &Float:recommended_speed_kmh = 0.0);
native SetConnectionSpeedLimit(Connection:connectionid, Float:speed_kmh);
native Float:GetConnectionSpeedLimit(Connection:connectionid);

native GetPathRejoinPoint(Path:old_path, Path:new_path, &diverge_step, &rejoin_old_step, &rejoin_new_step);

native SetConnectionMaxHeight(Connection:connectionid, Float:max_height_meters);
native Float:GetConnectionMaxHeight(Connection:connectionid);
native SetAreaMaxHeight(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:max_height_meters);
native FindPathWithHeightLimit(MapNode:source, MapNode:target, Float:vehicle_height, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);


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

    stock Task:FindPathAsyncEx(MapNode:source, MapNode:target, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST) 
    {
        new Task:task = task_new();

        if (FindPathThreadedEx(source, target, type, pref, "FindPathAsyncResponse", "i", _:task) != GPS_ERROR_NONE) 
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
