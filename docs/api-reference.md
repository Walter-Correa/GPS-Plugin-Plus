# Complete API Reference

Exhaustive technical dictionary covering all functions, callbacks, enumerations, error codes, and PawnPlus asynchronous stocks provided by **GPS Plugin Plus**.

---

## Quick Navigation

* [1. Map Node Management](#1-map-node-management)
* [2. Road Connections & Lanes](#2-road-connections--lanes)
* [3. Spatial Queries & Geometric Snapping](#3-spatial-queries--geometric-snapping)
* [4. Pathfinding & Routing](#4-pathfinding--routing)
* [5. Dynamic Traffic & Roadblocks](#5-dynamic-traffic--roadblocks)
* [6. Turn-by-Turn Navigation & Tracking](#6-turn-by-turn-navigation--tracking)
* [7. Route Geometry & Splines](#7-route-geometry--splines)
* [8. Autonomous AI, Interception & Evasion](#8-autonomous-ai-interception--evasion)
* [9. Radars, Isochrones & Nearest Service](#9-radars-isochrones--nearest-service)
* [10. Tunnels, Danger & Elevation](#10-tunnels-danger--elevation)
* [11. Road Characteristics & Speed Limits](#11-road-characteristics--speed-limits)
* [12. Height Clearances & Vehicle Categories](#12-height-clearances--vehicle-categories)
* [13. Path Serialization & Persistence](#13-path-serialization--persistence)
* [14. San Andreas Geographic Zones](#14-san-andreas-geographic-zones)
* [15. PawnPlus Asynchronous Tasks](#15-pawnplus-asynchronous-tasks)
* [16. Constants, Enums & Error Codes](#16-constants-enums--error-codes)

---

## 1. Map Node Management

### `CreateMapNode`
```pawn
native CreateMapNode(
    Float:x, Float:y, Float:z, 
    &MapNode:nodeid, 
    PathType:type = PATH_TYPE_VEHICLE, 
    PathNodeFlags:flags = NODE_FLAG_NONE
);
```
Allocates a new custom map node at the specified 3D coordinate.
* **`x, y, z`:** World coordinates.
* **`&nodeid`:** Output parameter receiving the newly allocated node ID.
* **`type`:** Transportation layer (`PATH_TYPE_VEHICLE`, `PATH_TYPE_PEDESTRIAN`, `PATH_TYPE_BOAT`).
* **`flags`:** Bitwise node properties (`NODE_FLAG_HIGHWAY`, `NODE_FLAG_CROSSWALK`, etc.). Note that `NODE_FLAG_EMERGENCY_ONLY` (2) and `NODE_FLAG_ROADBLOCK` (4) are dataset metadata flags present in `GPS.dat`; they describe road attributes but do not automatically prevent routing. Use `SetMapNodeBlocked` to block path traversal dynamically.
* **Returns:** `GPS_ERROR_NONE` (0) on success, or a negative `GPS_ERROR` code.

---

### `DestroyMapNode`
```pawn
native DestroyMapNode(MapNode:nodeid);
```
Destroys a dynamically created map node and breaks all attached connections.
* **Returns:** `1` on success, `0` if node is invalid.

---

### `IsValidMapNode`
```pawn
native bool:IsValidMapNode(MapNode:nodeid);
```
Checks if the specified `MapNode` ID exists in the spatial graph.
* **Returns:** `true` if valid, `false` otherwise.

---

### `GetMapNodePos`
```pawn
native GetMapNodePos(MapNode:nodeid, &Float:x, &Float:y, &Float:z);
```
Retrieves the 3D world coordinates of a map node.
* **Returns:** `1` on success, `0` on invalid node.

---

### `GetMapNodeType`
```pawn
native PathType:GetMapNodeType(MapNode:nodeid);
```
Queries whether a node belongs to vehicles, pedestrians, or boats.
* **Returns:** `PathType` enumeration value.

---

### `GetMapNodeFlags`
```pawn
native GetMapNodeFlags(MapNode:nodeid, &PathNodeFlags:flags);
```
Queries bitwise flags attached to a node.
* **Returns:** `1` on success, `0` on failure.

---

### `GetHighestMapNodeID`
```pawn
native GetHighestMapNodeID(PathType:type = PATH_TYPE_VEHICLE);
```
Returns the maximum active node ID in the graph for the given transport type (defaults to `PATH_TYPE_VEHICLE`). Pass `PATH_TYPE_ANY` to query the highest ID across all modalities (vehicles, pedestrians, and boats), matching legacy v1.x behavior.

---

### `GetRandomMapNode`
```pawn
native GetRandomMapNode(
    &MapNode:nodeid, 
    PathType:type = PATH_TYPE_VEHICLE, 
    Float:minz = -99999.0, Float:maxz = 99999.0
);
```
Picks a random map node across the entire map within optional height bounds.

---

### `GetRandomMapNodeFromArea`
```pawn
native GetRandomMapNodeFromArea(
    Float:minx, Float:miny, Float:maxx, Float:maxy, 
    &MapNode:nodeid, 
    PathType:type = PATH_TYPE_VEHICLE, 
    Float:minz = -99999.0, Float:maxz = 99999.0
);
```
Samples a random map node bounded within a 2D/3D axis-aligned bounding box.

---

### `SaveMapNodesToFile`
```pawn
native SaveMapNodesToFile(const filename[]);
```
Serializes the current active in-memory node and connection graph into a `.dat` text file.

---

## 2. Road Connections & Lanes

### `CreateConnection`
```pawn
native CreateConnection(
    MapNode:source, MapNode:target, 
    &Connection:connectionid, 
    Float:road_width = 8.0, 
    left_lanes = 1, 
    right_lanes = 1, 
    bool:one_way = false
);
```
Constructs a navigable road connection linking `source` to `target`.
* **`road_width`:** Roadbed width in meters.
* **`left_lanes`, `right_lanes`:** Lane configuration.
* **`one_way`:** If `true`, permits traversal only from `source` $\to$ `target`.
* **Returns:** `GPS_ERROR_NONE` on success.

---

### `DestroyConnection`
```pawn
native DestroyConnection(Connection:connectionid);
```
Removes a connection between two nodes.

---

### `GetConnectionSource` / `GetConnectionTarget`
```pawn
native GetConnectionSource(Connection:connectionid, &MapNode:nodeid);
native GetConnectionTarget(Connection:connectionid, &MapNode:nodeid);
```
Queries the start (`source`) or destination (`target`) map node of a connection.

---

### `GetMapNodeConnectionCount` / `GetMapNodeConnection`
```pawn
native GetMapNodeConnectionCount(MapNode:nodeid, &count);
native GetMapNodeConnection(MapNode:nodeid, index, &Connection:connectionid);
```
Iterates through all outward connections leaving a given map node.

---

### `GetMapNodeConnectionTarget`
```pawn
native GetMapNodeConnectionTarget(MapNode:nodeid, index, &MapNode:target_nodeid);
```
Retrieves the target destination map node ID for the given outward connection index from a map node.

---

### `IsMapNodeIntersection`
```pawn
native bool:IsMapNodeIntersection(MapNode:nodeid, &connection_count = 0);
```
Determines if a map node is an intersection (3 or more outward connections) and optionally writes the total connection count.

---

### `GetConnectionBetweenMapNodes`
```pawn
native GetConnectionBetweenMapNodes(MapNode:source, MapNode:target, &Connection:connectionid);
```
Finds the active connection linking two specific map nodes directly.

---

### `GetConnectionLaneInfo`
```pawn
native GetConnectionLaneInfo(Connection:connectionid, &Float:road_width, &left_lanes, &right_lanes);
```
Retrieves physical road width and lane counts for both directions.

---

### `GetConnectionLanePos`
```pawn
native GetConnectionLanePos(Connection:connectionid, lane_index, &Float:x, &Float:y, &Float:z);
```
Calculates the exact 3D midpoint coordinate of a specific lane along a connection.

---

### `IsConnectionOneWay`
```pawn
native bool:IsConnectionOneWay(Connection:connectionid);
```
Returns `true` if traffic is restricted to a single direction.

---

## 3. Spatial Queries & Geometric Snapping

### `GetDistanceBetweenMapNodes`
```pawn
native GetDistanceBetweenMapNodes(MapNode:first, MapNode:second, &Float:distance);
```
Calculates 3D Euclidean distance in meters between two map nodes.

---

### `GetAngleBetweenMapNodes`
```pawn
native GetAngleBetweenMapNodes(MapNode:first, MapNode:second, &Float:angle, bool:facing_angle = false);
```
Computes the compass bearing ($0.0^\circ$ to $360.0^\circ$) from `first` to `second`.
* If `facing_angle = true`, converts bearing to GTA SA vehicle heading convention.

---

### `GetMapNodeDistanceFromPoint` / `GetMapNodeAngleFromPoint`
```pawn
native GetMapNodeDistanceFromPoint(MapNode:nodeid, Float:x, Float:y, Float:z, &Float:distance);
native GetMapNodeAngleFromPoint(MapNode:nodeid, Float:x, Float:y, &Float:angle, bool:facing_angle = false);
```
Computes distance and relative angle from an arbitrary world coordinate to a map node.

---

### `GetClosestMapNodeToPoint`
```pawn
native GetClosestMapNodeToPoint(
    Float:x, Float:y, Float:z, 
    &MapNode:nodeid, 
    MapNode:ignorednode = GPS_INVALID_MAP_NODE_ID, 
    PathType:type = PATH_TYPE_VEHICLE
);
```
Searches the spatial index for the closest node to the specified coordinate.

---

### `GetClosestPointOnRoad`
```pawn
native GetClosestPointOnRoad(
    Float:x, Float:y, Float:z, 
    &Float:out_x, &Float:out_y, &Float:out_z, 
    &Float:angle, 
    &Connection:connectionid = INVALID_CONNECTION_ID, 
    PathType:type = PATH_TYPE_VEHICLE
);
```
Snaps an off-road point onto the nearest drivable road centerline segment.

---

### `GetRoadPositionInfo`
```pawn
native GetRoadPositionInfo(
    Float:x, Float:y, Float:z, 
    &Float:out_x, &Float:out_y, &Float:out_z, 
    &Float:angle, 
    &Float:road_width, 
    RoadPosition:pos = ROAD_POS_CURB, 
    PathType:type = PATH_TYPE_VEHICLE
);
```
Snaps a coordinate to the road centerline (`ROAD_POS_CENTER`), sidewalk curb (`ROAD_POS_CURB`), or outer edge shoulder (`ROAD_POS_EDGE`), returning the exact heading of traffic.

---

## 4. Pathfinding & Routing

> [!IMPORTANT]
> **Script Ownership & Memory Management**: Paths created by a script (gamemode or filterscript) are tracked per AMX instance. When a script is unloaded (e.g. during `gmx` or filterscript unload), all paths belonging to that script are automatically destroyed and freed, preventing orphan path leaks.
> 
> **Input Validation**: All pathfinding routines validate `PathType` and `RoutePreference` parameters against known bounds. If an invalid type or preference is passed, the function aborts immediately and returns `GPS_ERROR_INVALID_PATH_TYPE` (-2) or `GPS_ERROR_INVALID_ROUTE_PREF` (-3).

### `FindPath`
```pawn
native FindPath(
    MapNode:source, MapNode:target, 
    &Path:pathid, 
    PathType:type = PATH_TYPE_VEHICLE, 
    RoutePreference:pref = ROUTE_PREF_SHORTEST
);
```
Calculates a route synchronously on the server main thread.

---

### `FindPathThreaded` / `FindPathThreadedEx`
```pawn
native FindPathThreaded(
    MapNode:source, MapNode:target, 
    const callback[], 
    const format[] = "", 
    {Float, _}:...
);

native FindPathThreadedEx(
    MapNode:source, MapNode:target, 
    PathType:type = PATH_TYPE_VEHICLE, 
    RoutePreference:pref = ROUTE_PREF_SHORTEST, 
    const callback[] = "", 
    const format[] = "", 
    {Float, _}:...
);
```
Offloads path calculation to a native background C++ worker thread, invoking `callback` when complete.

---

### `FindMultiStopPath`
```pawn
native FindMultiStopPath(
    const MapNode:stops[], stops_count, 
    &Path:pathid, 
    bool:optimize_order = false, 
    PathType:type = PATH_TYPE_VEHICLE, 
    RoutePreference:pref = ROUTE_PREF_SHORTEST
);
```
Computes a continuous path visiting multiple sequential waypoints, with optional Travelling Salesperson (TSP) route optimization.

---

### `IsValidPath` / `DestroyPath`
```pawn
native bool:IsValidPath(Path:pathid);
native DestroyPath(Path:pathid);
```
Validates or releases a calculated path from memory.

---

### `GetPathSize` / `GetPathLength`
```pawn
native GetPathSize(Path:pathid, &size);
native GetPathLength(Path:pathid, &Float:length);
```
Queries the total node count and total metric length (meters) of a path.

---

### `GetPathNode` / `GetPathNodeIndex`
```pawn
native GetPathNode(Path:pathid, index, &MapNode:nodeid);
native GetPathNodeIndex(Path:pathid, MapNode:nodeid, &index);
```
Queries the node ID at step `index`, or retrieves the index of a node in the path.

---

### `GetPathLaneNodePos`
```pawn
native GetPathLaneNodePos(Path:pathid, index, lane_index, &Float:x, &Float:y, &Float:z);
```
Retrieves the 3D position of a path step offset into a specific lane.

---

## 5. Dynamic Traffic & Roadblocks

### `SetMapNodeBlocked` / `IsMapNodeBlocked`
```pawn
native SetMapNodeBlocked(MapNode:nodeid, bool:blocked);
native bool:IsMapNodeBlocked(MapNode:nodeid);
```
Closes or re-opens a map node. Blocked nodes are excluded from all pathfinding.

---

### `SetConnectionBlocked` / `IsConnectionBlocked`
```pawn
native SetConnectionBlocked(Connection:connectionid, bool:blocked);
native bool:IsConnectionBlocked(Connection:connectionid);
```
Closes or re-opens an individual road connection.

---

### `SetConnectionTrafficMultiplier` / `GetConnectionTrafficMultiplier`
```pawn
native SetConnectionTrafficMultiplier(Connection:connectionid, Float:multiplier);
native Float:GetConnectionTrafficMultiplier(Connection:connectionid);
```
Sets a congestion cost penalty on a road segment ($1.0 = \text{normal}$, $5.0 = \text{heavy}$).

---

### `SetAreaTrafficMultiplier` / `ResetTrafficMultipliers`
```pawn
native SetAreaTrafficMultiplier(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:multiplier);
native ResetTrafficMultipliers();
```
Applies traffic multipliers across a regional area or resets all connections to default.

---

## 6. Turn-by-Turn Navigation & Tracking

### `GetPathProgress`
```pawn
native GetPathProgress(
    Path:pathid, 
    Float:x, Float:y, Float:z, 
    &current_step, 
    &Float:distance_to_path, 
    &bool:off_route, 
    Float:max_distance_threshold = 25.0
);
```
Matches current vehicle coordinates onto the path, outputting current step index and off-route state.

> [!TIP]
> `GetPathProgress` and `GetPathNextManeuver` project onto path segments. Calling them inside a 250ms–500ms timer provides an optimal balance between responsiveness and peak server tick rates.

---

### `GetPathRemainingInfo`
```pawn
native GetPathRemainingInfo(
    Path:pathid, 
    current_step, 
    &Float:remaining_distance, 
    &Float:remaining_seconds, 
    Float:speed_kmh = 60.0
);
```
Calculates remaining meters and ETA from `current_step` to final destination.

---

### `GetPathEstimatedTravelTime`
```pawn
native GetPathEstimatedTravelTime(Path:pathid, Float:average_speed_kmh, &Float:seconds);
```
Calculates expected journey duration across the full path at a given average speed.

---

### `GetPathTurnDirection`
```pawn
native GetPathTurnDirection(Path:pathid, index, &TurnDirection:turn, &Float:turn_angle);
```
Determines the turn classification (`TURN_LEFT`, `TURN_RIGHT`, `TURN_UTURN`, etc.) at a given step.

---

### `GetPathNextManeuver`
```pawn
native GetPathNextManeuver(
    Path:pathid, 
    Float:x, Float:y, Float:z, 
    &TurnDirection:turn, 
    &Float:distance_to_turn
);
```
Finds the immediate upcoming turn maneuver and distance from the driver's current position.

---

### `GetPathRoundaboutInfo`
```pawn
native GetPathRoundaboutInfo(
    Path:pathid, 
    index, 
    &bool:is_roundabout, 
    &exit_number, 
    &Float:roundabout_radius
);
```
Detects if step `index` traverses a rotary junction, returning radius and exit number.

---

## 7. Route Geometry & Splines

### `GetSimplifiedPathNodes`
```pawn
native GetSimplifiedPathNodes(
    Path:pathid, 
    Float:tolerance_meters, 
    MapNode:output_nodes[], 
    max_nodes = sizeof(output_nodes), 
    &nodes_count
);
```
Decimates collinear path nodes using the Douglas-Peucker algorithm within a distance tolerance.

---

### `GetSmoothPathPoints`
```pawn
native GetSmoothPathPoints(
    Path:pathid, 
    Float:spacing_meters, 
    Float:out_x[], Float:out_y[], Float:out_z[], 
    max_points = sizeof(out_x), 
    &total_points = 0
);
```
Generates curvature-continuous Catmull-Rom spline points evenly spaced along the route.

---

### `GetPathRejoinPoint`
```pawn
native GetPathRejoinPoint(
    Path:old_path, Path:new_path, 
    &diverge_step, 
    &rejoin_old_step, 
    &rejoin_new_step
);
```
Finds where a detour route diverges from and rejoins an original route.

---

### `GetPathOffsetPosition`
```pawn
native GetPathOffsetPosition(
    Path:pathid, 
    Float:distance_along_path, 
    Float:lateral_offset, 
    &Float:x, &Float:y, &Float:z, &Float:angle
);
```
Calculates parallel left/right offset positions for escort motorcades and multi-car convoys.

---

## 8. Autonomous AI, Interception & Evasion

### `GetPurePursuitSteering`
```pawn
native GetPurePursuitSteering(
    Path:pathid, 
    Float:vehicle_x, Float:vehicle_y, Float:vehicle_angle, 
    Float:lookahead_dist, 
    &Float:steer_ratio, 
    &Float:recommended_speed_kmh
);
```
Pure Pursuit controller outputting dynamic steering wheel ratio ($-1.0$ to $+1.0$) and cornering speed for autonomous bot drivers.

---

### `FindInterceptPath`
```pawn
native FindInterceptPath(
    MapNode:pursuer_node, 
    MapNode:target_node, 
    Float:target_speed_kmh, 
    Float:pursuer_speed_kmh, 
    &Path:pathid, 
    &MapNode:intercept_node, 
    &Float:time_to_intercept, 
    PathType:type = PATH_TYPE_VEHICLE
);
```
Calculates a police pursuit intercept route cutting off a suspect ahead of time.

---

### `IsMapNodeDeadEnd` / `FindEvasivePath`
```pawn
native bool:IsMapNodeDeadEnd(MapNode:nodeid);
native FindEvasivePath(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE);
```
Identifies dead-end cul-de-sacs and calculates tactical escape routes avoiding traps.

---

## 9. Radars, Isochrones & Nearest Service

### `FindNearestPointByRoad`
```pawn
native FindNearestPointByRoad(
    Float:from_x, Float:from_y, Float:from_z, 
    const Float:points[][3], 
    total_points, 
    &nearest_index, 
    &Float:road_distance, 
    &Path:pathid = GPS_INVALID_PATH_ID, 
    PathType:type = PATH_TYPE_VEHICLE
);
```
Searches a list of 3D destination candidates, returning the index with the shortest **actual road driving distance**.

> [!NOTE]
> Parameter `pathid` is optional (default: `GPS_INVALID_PATH_ID`). If omitted or passed as `GPS_INVALID_PATH_ID`, no path object is registered in memory, preventing path leaks on pure distance/nearest-index lookups.

---

### `RegisterRoadRadar` / `UnregisterRoadRadar` / `ClearRoadRadars`
```pawn
native RegisterRoadRadar(radarid, Float:x, Float:y, Float:z, Float:speed_limit_kmh, Float:alert_radius = 300.0);
native bool:UnregisterRoadRadar(radarid);
native ClearRoadRadars();
```
Manages fixed or mobile speed radar cameras in the world.

> [!NOTE]
> Radars are stored globally in the server process. `GetNextPathRadar` evaluates cameras within `alert_radius` (default: 300m) of any upcoming segment of the path, including adjacent parallel streets within that radius.

---

### `GetNextPathRadar`
```pawn
native bool:GetNextPathRadar(
    Path:pathid, 
    Float:current_dist, 
    &radarid, 
    &Float:dist_to_radar, 
    &Float:radar_limit
);
```
Detects if an upcoming path node falls inside an active speed radar enforcement zone.

---

### `GetReachableMapNodes` / `GetReachableBoundingBox`
```pawn
native GetReachableMapNodes(
    MapNode:source, 
    Float:limit_value, 
    bool:by_time, 
    Float:speed_kmh, 
    MapNode:nodes_out[], 
    max_nodes = sizeof(nodes_out), 
    &count, 
    PathType:type = PATH_TYPE_VEHICLE
);

native GetReachableBoundingBox(
    MapNode:source, 
    Float:max_distance, 
    &Float:minx, &Float:miny, &Float:maxx, &Float:maxy, 
    PathType:type = PATH_TYPE_VEHICLE
);
```
Computes driving range isochrones (by seconds or meters) and geographic bounding boxes.

---

## 10. Tunnels, Danger & Elevation

### `RegisterTunnelArea` / `UnregisterTunnelArea` / `ClearCustomTunnelAreas`
```pawn
native RegisterTunnelArea(Float:minx, Float:miny, Float:minz, Float:maxx, Float:maxy, Float:maxz, &tunnel_id);
native bool:UnregisterTunnelArea(tunnel_id);
native ClearCustomTunnelAreas();
```
Registers 3D subterranean volumes for GPS signal loss simulation.

---

### `IsPointInTunnel` / `IsMapNodeUnderground` / `GetPathTunnelSegments`
```pawn
native bool:IsPointInTunnel(Float:x, Float:y, Float:z);
native bool:IsMapNodeUnderground(MapNode:nodeid);
native bool:GetPathTunnelSegments(Path:pathid, &Float:entry_distance, &Float:exit_distance);
```
Queries whether a point/node is underground, or locates tunnel entry/exit distances along a path.

---

### `SetMapNodeDanger` / `SetAreaDangerWeight` / `ResetDangerWeights` / `FindSafePath`
```pawn
native SetMapNodeDanger(MapNode:nodeid, Float:danger_weight);
native Float:GetMapNodeDanger(MapNode:nodeid);
native SetAreaDangerWeight(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:danger_weight);
native ResetDangerWeights();
native FindSafePath(MapNode:source, MapNode:target, &Path:pathid, Float:danger_penalty = 5.0, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);
```
Applies crime/danger penalties to nodes or areas, allowing `FindSafePath` to calculate safe detours.

---

### `FindEcoPath` / `GetPathElevationStats`
```pawn
native FindEcoPath(MapNode:source, MapNode:target, &Path:pathid, Float:max_slope_gradient = 0.12, PathType:type = PATH_TYPE_VEHICLE);
native GetPathElevationStats(Path:pathid, &Float:min_z, &Float:max_z, &Float:total_climb, &Float:max_incline_percent);
```
Calculates fuel-efficient routes avoiding steep grades and computes elevation telemetry.

---

## 11. Road Characteristics & Speed Limits

### `IsPointAgainstTraffic`
```pawn
native IsPointAgainstTraffic(Float:x, Float:y, Float:z, Float:facing_angle, &bool:against_traffic);
```
Detects if a vehicle is facing the opposite direction of legal traffic flow.

---

### `IsConnectionPaved` / `SetConnectionPaved` / `FindPathPavedOnly`
```pawn
native bool:IsConnectionPaved(Connection:connectionid);
native SetConnectionPaved(Connection:connectionid, bool:paved);
native FindPathPavedOnly(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE);
```
Manages paved asphalt vs dirt status and restricts paths strictly to asphalt.

---

### `GetConnectionRoadClass` / `SetConnectionSpeedLimit` / `GetConnectionSpeedLimit`
```pawn
native RoadClass:GetConnectionRoadClass(Connection:connectionid, &Float:recommended_speed_kmh = 0.0);
native SetConnectionSpeedLimit(Connection:connectionid, Float:speed_kmh);
native Float:GetConnectionSpeedLimit(Connection:connectionid);
```
Queries road hierarchy (`ROAD_CLASS_ALLEY`, `ROAD_CLASS_RESIDENTIAL`, `ROAD_CLASS_AVENUE`, `ROAD_CLASS_HIGHWAY`) and manages legal speed limits.

---

### `SetConnectionToll` / `GetConnectionToll` / `FindPathAvoidTolls`
```pawn
native SetConnectionToll(Connection:connectionid, bool:has_toll, toll_cost = 0);
native bool:GetConnectionToll(Connection:connectionid, &toll_cost = 0);
native FindPathAvoidTolls(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);
```
Configures toll bridges/turnpikes and routes around pay gates.

---

## 12. Height Clearances & Vehicle Categories

### `SetConnectionMaxHeight` / `GetConnectionMaxHeight` / `SetAreaMaxHeight`
```pawn
native SetConnectionMaxHeight(Connection:connectionid, Float:max_height_meters);
native Float:GetConnectionMaxHeight(Connection:connectionid);
native SetAreaMaxHeight(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:max_height_meters);
```
Defines vertical clearance limits for low bridges, tunnels, and overpasses.

---

### `FindPathWithHeightLimit`
```pawn
native FindPathWithHeightLimit(
    MapNode:source, MapNode:target, 
    Float:vehicle_height, 
    &Path:pathid, 
    PathType:type = PATH_TYPE_VEHICLE, 
    RoutePreference:pref = ROUTE_PREF_SHORTEST
);
```
Calculates a route that avoids any bridge or tunnel lower than `vehicle_height`.

---

### `FindPathForVehicleCat`
```pawn
native FindPathForVehicleCat(
    MapNode:source, MapNode:target, 
    &Path:pathid, 
    VehicleCategory:cat = VEHICLE_CAT_STANDARD, 
    RoutePreference:pref = ROUTE_PREF_SHORTEST
);
```
Routes with vehicle constraints (`VEHICLE_CAT_STANDARD`, `VEHICLE_CAT_HEAVY_TRUCK`, `VEHICLE_CAT_MOTORCYCLE`).

---

## 13. Path Serialization & Persistence

### `SetPathAutoDestroy`
```pawn
native SetPathAutoDestroy(Path:pathid, milliseconds = 60000);
```
Arms a path with an automatic TTL garbage collection timer, freeing C++ memory after expiration.

---

### `SavePathToFile` / `LoadPathFromFile`
```pawn
native SavePathToFile(Path:pathid, const filename[]);
native LoadPathFromFile(const filename[], &Path:pathid);
```
Serializes and restores paths to/from text files on disk.

> [!IMPORTANT]
> **Path Traversal Protection**: File operations are strictly sandboxed inside the server `scriptfiles/` directory. Path traversal sequences (`..`), absolute paths (`C:\`, `/`), drive colons, and leading slashes are rejected with `GPS_ERROR_INVALID_FILE` (-9).

---

### `ExportPathToString` / `ImportPathFromString`
```pawn
native ExportPathToString(Path:pathid, buffer[], max_len = sizeof(buffer));
native ImportPathFromString(const buffer[], &Path:pathid);
```
Encodes and decodes paths to/from portable string buffers (ideal for SQL/NoSQL databases).

---

## 14. San Andreas Geographic Zones

### `GetPointZoneName` / `GetMapNodeZoneName`
```pawn
native GetPointZoneName(Float:x, Float:y, Float:z, zone_name[], max_len = sizeof(zone_name));
native GetMapNodeZoneName(MapNode:nodeid, zone_name[], max_len = sizeof(zone_name));
```
Resolves 3D coordinates or map nodes to in-game neighborhood names (e.g. "Ganton", "Rodeo", "Prickle Pine").

---

### `GetPathZonesSummary`
```pawn
native GetPathZonesSummary(Path:pathid, zones[][], max_zones, &zones_count, max_zone_len = sizeof(zones[]));
```
Outputs an ordered list of unique district names traversed along a path.

---

## 15. PawnPlus Asynchronous Tasks

When `<PawnPlus>` is included before `<GPS>`, modern Task coroutine wrappers are automatically available:

### `FindPathAsync`
```pawn
stock Task:FindPathAsync(MapNode:source, MapNode:target);
```
Calculates a path asynchronously, resolving a `Task` promise with the allocated `Path:pathid`.

---

### `FindPathAsyncEx`
```pawn
stock Task:FindPathAsyncEx(
    MapNode:source, MapNode:target, 
    PathType:type = PATH_TYPE_VEHICLE, 
    RoutePreference:pref = ROUTE_PREF_SHORTEST
);
```
Asynchronous Task pathfinding with explicit `PathType` and `RoutePreference`.

---

## 16. Constants, Enums & Error Codes

### Error Codes (`GPS_ERROR`)
```pawn
enum _:GPS_ERROR 
{
    GPS_ERROR_NONE               =  0, // Success
    GPS_ERROR_INVALID_PARAMS     = -1, // Invalid parameter types or values
    GPS_ERROR_INVALID_PATH       = -2, // Path handle does not exist
    GPS_ERROR_INVALID_NODE       = -3, // MapNode handle does not exist
    GPS_ERROR_INVALID_CONNECTION = -4, // Connection handle does not exist
    GPS_ERROR_INTERNAL           = -5  // Internal memory or threading error
}
```

### Transport Types (`PathType`)
```pawn
enum PathType
{
    PATH_TYPE_ANY        = -1,
    PATH_TYPE_VEHICLE    = 0,
    PATH_TYPE_PEDESTRIAN = 1,
    PATH_TYPE_BOAT       = 2
}
```

### Routing Heuristics (`RoutePreference`)
```pawn
enum RoutePreference
{
    ROUTE_PREF_SHORTEST       = 0,
    ROUTE_PREF_HIGHWAYS       = 1,
    ROUTE_PREF_AVOID_HIGHWAYS = 2,
    ROUTE_PREF_SAFEST         = 3,
    ROUTE_PREF_ECO            = 4,
    ROUTE_PREF_EVASIVE        = 5,
    ROUTE_PREF_AVOID_TOLLS    = 6,
    ROUTE_PREF_PAVED_ONLY     = 7
}
```

### Turn Classifications (`TurnDirection`)
```pawn
enum TurnDirection
{
    TURN_NONE        = 0,
    TURN_STRAIGHT    = 1,
    TURN_RIGHT       = 2,
    TURN_LEFT        = 3,
    TURN_SHARP_RIGHT = 4,
    TURN_SHARP_LEFT  = 5,
    TURN_UTURN       = 6
}
```

### Road Alignments (`RoadPosition`)
```pawn
enum RoadPosition
{
    ROAD_POS_CENTER = 0,
    ROAD_POS_CURB   = 1,
    ROAD_POS_EDGE   = 2
}
```

### Road Classifications (`RoadClass`)
```pawn
enum RoadClass
{
    ROAD_CLASS_ALLEY       = 0,
    ROAD_CLASS_RESIDENTIAL = 1,
    ROAD_CLASS_AVENUE      = 2,
    ROAD_CLASS_HIGHWAY     = 3
}
```

### Vehicle Categories (`VehicleCategory`)
```pawn
enum VehicleCategory
{
    VEHICLE_CAT_STANDARD    = 0,
    VEHICLE_CAT_HEAVY_TRUCK = 1,
    VEHICLE_CAT_MOTORCYCLE  = 2
}
```

### Node Flags (`PathNodeFlags`)
```pawn
enum PathNodeFlags (<<= 1)
{
    NODE_FLAG_NONE = 0,
    NODE_FLAG_HIGHWAY = 1,
    NODE_FLAG_EMERGENCY_ONLY,
    NODE_FLAG_ROADBLOCK,
    NODE_FLAG_CROSSWALK,
    NODE_FLAG_ONEWAY
}
```

> [!NOTE]
> `NODE_FLAG_EMERGENCY_ONLY` (2) and `NODE_FLAG_ROADBLOCK` (4) originate from San Andreas map metadata in `GPS.dat` (tagged on over 7,600 nodes). They serve as descriptive road attributes and do **not** automatically exclude nodes during A* pathfinding. To dynamically block roads or intersections, use `SetMapNodeBlocked(nodeid, true)` or `SetConnectionBlocked(connectionid, true)`.
