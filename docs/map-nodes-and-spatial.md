# Map Nodes & Spatial Analytics

This document details the spatial architecture of **GPS Plugin Plus**, covering map node topologies, multi-lane road connection geometry, curb-side vehicle snapping, neighborhood zone analytics, and memory serialization pipelines.

---

## 1. Graph Node Architecture

The road network of San Andreas is represented as an optimized directed spatial graph:
* **Nodes (`MapNode`):** 3D geographic coordinates representing intersections, waypoints, road bends, footpath vertices, and maritime anchors.
* **Connections (`Connection`):** Directed or bidirectional road edges linking nodes, holding lane widths, one-way constraints, speed limits, and pavement properties.

```pawn
const MapNode:GPS_INVALID_MAP_NODE_ID       = MapNode:-1;
const Connection:GPS_INVALID_CONNECTION_ID = Connection:-1;
```

### Dynamic Node Lifecycle
While `GPS.dat` preloads over 30,000 nodes across the state, developers can dynamically allocate and destroy custom nodes at runtime (e.g. for custom mapping, interior roads, or islands):

```pawn
native CreateMapNode(
    Float:x, Float:y, Float:z, 
    &MapNode:nodeid, 
    PathType:type = PATH_TYPE_VEHICLE, 
    PathNodeFlags:flags = NODE_FLAG_NONE
);
native DestroyMapNode(MapNode:nodeid);
native bool:IsValidMapNode(MapNode:nodeid);
native GetMapNodePos(MapNode:nodeid, &Float:x, &Float:y, &Float:z);
native PathType:GetMapNodeType(MapNode:nodeid);
native GetMapNodeFlags(MapNode:nodeid, &PathNodeFlags:flags);
native GetHighestMapNodeID(PathType:type = PATH_TYPE_VEHICLE);
```

---

## 2. Road Connections & Multi-Lane Geometry

Connections define drivable paths between nodes with explicit road widths and lane counts:

```pawn
native CreateConnection(
    MapNode:source, MapNode:target, 
    &Connection:connectionid, 
    Float:road_width = 8.0, 
    left_lanes = 1, 
    right_lanes = 1, 
    bool:one_way = false
);
native DestroyConnection(Connection:connectionid);

native GetConnectionSource(Connection:connectionid, &MapNode:nodeid);
native GetConnectionTarget(Connection:connectionid, &MapNode:nodeid);
native GetMapNodeConnectionCount(MapNode:nodeid, &count);
native GetMapNodeConnection(MapNode:nodeid, index, &Connection:connectionid);
native GetConnectionBetweenMapNodes(MapNode:source, MapNode:target, &Connection:connectionid);

native GetConnectionLaneInfo(Connection:connectionid, &Float:road_width, &left_lanes, &right_lanes);
native GetConnectionLanePos(Connection:connectionid, lane_index, &Float:x, &Float:y, &Float:z);
native bool:IsConnectionOneWay(Connection:connectionid);
```

### Precise Lane Coordinate Query
To place an AI vehicle into a specific lane (e.g. the far-right slow lane of a 4-lane freeway), use `GetConnectionLanePos`:

```pawn
new Float:lx, Float:ly, Float:lz;
// Lane indices: 0 = leftmost, right_lanes - 1 = rightmost
GetConnectionLanePos(highway_conn, 2, lx, ly, lz);
SetVehiclePos(patrol_car, lx, ly, lz);
```

---

## 3. Spatial Queries & Geometric Snapping

Locate road nodes, compute angles, and sample random positions across districts:

```pawn
native GetDistanceBetweenMapNodes(MapNode:first, MapNode:second, &Float:distance);
native GetAngleBetweenMapNodes(MapNode:first, MapNode:second, &Float:angle, bool:facing_angle = false);
native GetMapNodeDistanceFromPoint(MapNode:nodeid, Float:x, Float:y, Float:z, &Float:distance);
native GetMapNodeAngleFromPoint(MapNode:nodeid, Float:x, Float:y, &Float:angle, bool:facing_angle = false);

native GetClosestMapNodeToPoint(
    Float:x, Float:y, Float:z, 
    &MapNode:nodeid, 
    MapNode:ignorednode = GPS_INVALID_MAP_NODE_ID, 
    PathType:type = PATH_TYPE_VEHICLE
);

native GetClosestPointOnRoad(
    Float:x, Float:y, Float:z, 
    &Float:out_x, &Float:out_y, &Float:out_z, 
    &Float:angle, 
    &Connection:connectionid = INVALID_CONNECTION_ID, 
    PathType:type = PATH_TYPE_VEHICLE
);
```

### Random Map Node Sampling
Select random road nodes across the entire world or strictly inside a bounded regional area:

```pawn
native GetRandomMapNode(
    &MapNode:nodeid, 
    PathType:type = PATH_TYPE_VEHICLE, 
    Float:minz = -99999.0, Float:maxz = 99999.0
);

native GetRandomMapNodeFromArea(
    Float:minx, Float:miny, Float:maxx, Float:maxy, 
    &MapNode:nodeid, 
    PathType:type = PATH_TYPE_VEHICLE, 
    Float:minz = -99999.0, Float:maxz = 99999.0
);
```

```pawn
// Pick a random pedestrian sidewalk node within Los Santos
new MapNode:ped_node;
GetRandomMapNodeFromArea(1000.0, -2000.0, 2800.0, -1000.0, ped_node, PATH_TYPE_PEDESTRIAN);
```

---

## 4. Road Position & Curb-Side Alignment

Spawning vehicles in the dead center of a road obstructs active traffic. When creating roadside assistance missions (mechanic calls), bus stops, or parked police speed traps, vehicles should be positioned **aligned against the curb or road edge**:

```pawn
enum RoadPosition
{
    ROAD_POS_CENTER = 0, // Road centerline
    ROAD_POS_CURB   = 1, // Outer edge of lane adjacent to sidewalk curb
    ROAD_POS_EDGE   = 2  // Off-road shoulder
}

native GetRoadPositionInfo(
    Float:x, Float:y, Float:z, 
    &Float:out_x, &Float:out_y, &Float:out_z, 
    &Float:angle, 
    &Float:road_width, 
    RoadPosition:pos = ROAD_POS_CURB, 
    PathType:type = PATH_TYPE_VEHICLE
);
```

### Mechanic Broken-Down Vehicle Spawner
```pawn
SpawnRoadsideMechanicMission(playerid)
{
    new MapNode:random_road;
    GetRandomMapNodeFromArea(1300.0, -1800.0, 2200.0, -1100.0, random_road);

    new Float:nx, Float:ny, Float:nz;
    GetMapNodePos(random_road, nx, ny, nz);

    new Float:curb_x, Float:curb_y, Float:curb_z, Float:curb_heading, Float:width;
    GetRoadPositionInfo(nx, ny, nz, curb_x, curb_y, curb_z, curb_heading, width, ROAD_POS_CURB);

    // Spawn vehicle perfectly parallel to curb facing flow of traffic
    new veh = CreateVehicle(400, curb_x, curb_y, curb_z, curb_heading, 1, 1, -1);
    SetVehicleHealth(veh, 350.0);
}
```

---

## 5. San Andreas Neighborhood Zones

Automatically identify in-game district names without relying on external `#include <zones>` lookups:

```pawn
native GetPointZoneName(Float:x, Float:y, Float:z, zone_name[], max_len = sizeof(zone_name));
native GetMapNodeZoneName(MapNode:nodeid, zone_name[], max_len = sizeof(zone_name));
native GetPathZonesSummary(Path:pathid, zones[][], max_zones, &zones_count, max_zone_len = sizeof(zones[]));
```

```pawn
new zone[32];
GetPointZoneName(px, py, pz, zone);
printf("Player is currently located in: %s", zone); // e.g. "Rodeo", "Ganton", "Bone County"
```

### Route Neighborhood Summary
Display a list of all neighborhoods traversed by a planned delivery route:
```pawn
new route_zones[8][32], zone_count;
GetPathZonesSummary(delivery_path, route_zones, sizeof(route_zones), zone_count);

for (new i = 0; i < zone_count; i++)
{
    printf("Waypoint Sector: %s", route_zones[i]);
}
```

---

## 6. Path Serialization & Memory Management

Paths allocated in GPS Plugin Plus reside in high-speed C++ native memory. To maintain peak server performance, active paths should be destroyed when no longer needed, or configured with an **Auto-Destroy TTL**:

```pawn
native SetPathAutoDestroy(Path:pathid, milliseconds = 60000);
native DestroyPath(Path:pathid);
```

```pawn
new Path:p;
FindPath(src, dst, p);
// Configure path to self-destruct after 45 seconds (absolute TTL)
SetPathAutoDestroy(p, 45000);
```

### Persistent Path Storage & Networking
Save, load, or serialize paths into text files or string representations:

```pawn
native SavePathToFile(Path:pathid, const filename[]);
native LoadPathFromFile(const filename[], &Path:pathid);

native ExportPathToString(Path:pathid, buffer[], max_len = sizeof(buffer));
native ImportPathFromString(const buffer[], &Path:pathid);

native SaveMapNodesToFile(const filename[]);
```

```pawn
// Export a calculated route as a base64/hex string for database storage
new serialized[2048];
ExportPathToString(tour_path, serialized);

// Later / across server restarts:
new Path:restored_path;
ImportPathFromString(serialized, restored_path);
```
