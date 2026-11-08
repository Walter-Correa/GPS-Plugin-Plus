# Pathfinding & Routing Engine

**GPS Plugin Plus** provides an industrial-grade A* pathfinding and routing engine engineered specifically for San Andreas. With support for multi-threaded worker pools, multi-modal transport graphs, and 8 intelligent routing heuristics, it enables servers to compute complex routes across thousands of road segments via background worker threads (with zero server tick lag) or fast on-demand synchronous heuristics running in milliseconds.

---

## 1. Synchronous vs. Asynchronous Pathfinding

Depending on your use case, the plugin offers three execution models:

| Mode | Function | Execution Context | Recommended Use Case |
| :--- | :--- | :--- | :--- |
| **Synchronous** | `FindPath` | Server Main Thread (Blocks tick) | Short distances (< 300m), initialization, admin tools. |
| **Threaded (Callback)** | `FindPathThreaded` / `FindPathThreadedEx` | C++ Worker Thread Pool | Long-distance routing across islands, real-time player GPS. |
| **Async Tasks (PawnPlus)** | `FindPathAsync` / `FindPathAsyncEx` | Threaded Worker + Task Promise | Modern linear coroutines using `task_await` without callbacks. |

### Threaded Pathfinding with Parameter Passing

When calling `FindPathThreadedEx`, you can forward custom integer, float, or string arguments into your response callback:

```pawn
FindPathThreadedEx(
    start_node, target_node,
    PATH_TYPE_VEHICLE, ROUTE_PREF_HIGHWAYS,
    "OnPlayerPathFound", "iiif",
    playerid, destination_id, is_priority, custom_speed
);

forward OnPlayerPathFound(Path:path, playerid, destination_id, is_priority, Float:custom_speed);
public OnPlayerPathFound(Path:path, playerid, destination_id, is_priority, Float:custom_speed)
{
    if (!IsValidPath(path)) return 0;
    // Process route...
    return 1;
}
```

---

## 2. Multi-Modal Transport Graphs (`PathType`)

The San Andreas map database (`GPS.dat`) contains distinct graph layers tailored to different entity movement types:

```pawn
enum PathType
{
    PATH_TYPE_ANY        = -1, // Traverse all connected nodes regardless of type
    PATH_TYPE_VEHICLE    = 0,  // Standard roads, freeways, lanes, dirt tracks (Default)
    PATH_TYPE_PEDESTRIAN = 1,  // Sidewalks, pedestrian alleys, stairs, footbridges
    PATH_TYPE_BOAT       = 2   // Oceans, bay channels, rivers, canals
}
```

### Pedestrian Footpaths
Generate walking routes that utilize pedestrian paths, crosswalks, and sidewalks while avoiding highway expressways:
```pawn
new Path:ped_path;
FindPath(ped_start, ped_dest, ped_path, PATH_TYPE_PEDESTRIAN);
```

### Maritime & Boat Navigation
Calculate navigable river channels and ocean routes around San Andreas for boats, ferries, and Coast Guard patrols:
```pawn
new Path:boat_path;
FindPath(dock_node, harbor_node, boat_path, PATH_TYPE_BOAT);
```

---

## 3. Route Preferences (`RoutePreference`)

Unlike basic distance-only algorithms, GPS Plugin Plus incorporates cost-function weighting heuristics:

```pawn
enum RoutePreference
{
    ROUTE_PREF_SHORTEST       = 0, // Minimizes absolute metric distance (Euclidean road length)
    ROUTE_PREF_HIGHWAYS       = 1, // Prioritizes freeways & avenues for fastest travel time
    ROUTE_PREF_AVOID_HIGHWAYS = 2, // Avoids freeways; routes through local streets & scenic roads
    ROUTE_PREF_SAFEST         = 3, // Penalizes high-crime zones and danger-weighted areas
    ROUTE_PREF_ECO            = 4, // Minimizes elevation climbs, steep hills, and mountain inclines
    ROUTE_PREF_EVASIVE        = 5, // Maximizes egress options; avoids dead-ends and chokepoints
    ROUTE_PREF_AVOID_TOLLS    = 6, // Avoids toll bridges, turnpikes, and pay-gates
    ROUTE_PREF_PAVED_ONLY     = 7  // Restricts routing strictly to asphalt/concrete paved roads
}
```

---

## 4. Multi-Stop & Waypoint Routing (`FindMultiStopPath`)

Deliveries, bus transit schedules, garbage routes, and patrol patterns frequently require visiting multiple waypoints:

```pawn
native FindMultiStopPath(
    const MapNode:stops[], stops_count, &Path:pathid, 
    bool:optimize_order = false, 
    PathType:type = PATH_TYPE_VEHICLE, 
    RoutePreference:pref = ROUTE_PREF_SHORTEST
);
```

### Sequential vs. TSP Optimization
* **`optimize_order = false`:** Visits waypoints in the exact order specified in `stops[]`.
* **`optimize_order = true`:** Solves the **Travelling Salesperson Problem (TSP)** using an optimized heuristic, reordering stops to produce the overall shortest total driving time.

```pawn
new MapNode:dropoffs[4];
dropoffs[0] = warehouse_node;
dropoffs[1] = customer_a;
dropoffs[2] = customer_b;
dropoffs[3] = customer_c;

new Path:delivery_route;
// Calculates fastest route reordering stops intelligently
FindMultiStopPath(dropoffs, sizeof(dropoffs), delivery_route, true, PATH_TYPE_VEHICLE, ROUTE_PREF_HIGHWAYS);
```

---

## 5. Dynamic Roadblocks & Detours

> [!NOTE]
> Graph modifications—including roadblocks (`SetConnectionBlocked`), traffic multipliers (`SetConnectionTrafficMultiplier`), danger zones, and tolls—operate globally across the server process and are shared between gamemodes and filterscripts.

Simulate police roadblocks, construction closures, fallen trees, or gang territory lockdowns in real time.

```pawn
native SetMapNodeBlocked(MapNode:nodeid, bool:blocked);
native bool:IsMapNodeBlocked(MapNode:nodeid);

native SetConnectionBlocked(Connection:connectionid, bool:blocked);
native bool:IsConnectionBlocked(Connection:connectionid);
```

When an edge or node is marked as blocked, the pathfinding engine treats its traversal cost as infinity ($\infty$), instantly routing all subsequent calculations around the blockage:

```pawn
// Block a specific bridge connection due to an accident
SetConnectionBlocked(vinewood_bridge_conn, true);

// Any player requesting a route through the bridge will automatically take a detour
FindPathThreaded(player_node, target_node, "OnDetourFound");
```

---

## 6. Dynamic Traffic Congestion

Simulate rush hour traffic, construction delays, or holiday slowdowns by modifying traversal cost multipliers ($1.0 = \text{normal}$, $3.0 = \text{heavy traffic}$, $10.0 = \text{gridlock}$):

```pawn
native SetConnectionTrafficMultiplier(Connection:connectionid, Float:multiplier);
native Float:GetConnectionTrafficMultiplier(Connection:connectionid);
native SetAreaTrafficMultiplier(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:multiplier);
native ResetTrafficMultipliers();
```

### Area-Wide Rush Hour Example
```pawn
// Apply 3.5x congestion penalty across Downtown Los Santos during 18:00 rush hour
SetAreaTrafficMultiplier(1350.0, -1850.0, 1900.0, -1150.0, 3.5);

// Reset all traffic multipliers to standard speed
ResetTrafficMultipliers();
```

---

## 7. Safe-Route & Anti-Crime Routing

Protect armored bank trucks, VIP convoys, or taxi passengers from dangerous gang zones or high-crime neighborhoods:

```pawn
native SetMapNodeDanger(MapNode:nodeid, Float:danger_weight);
native Float:GetMapNodeDanger(MapNode:nodeid);
native SetAreaDangerWeight(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:danger_weight);
native ResetDangerWeights();

native FindSafePath(
    MapNode:source, MapNode:target, &Path:pathid, 
    Float:danger_penalty = 5.0, 
    PathType:type = PATH_TYPE_VEHICLE, 
    RoutePreference:pref = ROUTE_PREF_SHORTEST
);
```

```pawn
// Flag Idlewood / Ganton territory as high danger (weight: 10.0)
SetAreaDangerWeight(1900.0, -1800.0, 2400.0, -1500.0, 10.0);

// Compute a safe detour route avoiding the danger zone
new Path:safe_path;
FindSafePath(src, dst, safe_path, 8.0);
```

---

## 8. Eco-Routing & Elevation Analysis

Steep mountain roads (e.g. Mount Chiliad or Missionary Hill) consume excessive fuel and slow down heavy trucks:

```pawn
native FindEcoPath(
    MapNode:source, MapNode:target, &Path:pathid, 
    Float:max_slope_gradient = 0.12, 
    PathType:type = PATH_TYPE_VEHICLE
);

native GetPathElevationStats(
    Path:pathid, 
    &Float:min_z, &Float:max_z, 
    &Float:total_climb, &Float:max_incline_percent
);
```

```pawn
new Path:eco_path;
// Avoid any road segments with slope grade steeper than 10%
FindEcoPath(angel_pine_node, bayside_node, eco_path, 0.10);

new Float:minz, Float:maxz, Float:climb, Float:steepest;
GetPathElevationStats(eco_path, minz, maxz, climb, steepest);
printf("Total Climb: %.1fm | Max Incline: %.1f%%", climb, steepest * 100.0);
```

---

## 9. Toll Roads & Bridge Passes

Implement automated toll booths and turnpike fares:

```pawn
native SetConnectionToll(Connection:connectionid, bool:has_toll, toll_cost = 0);
native bool:GetConnectionToll(Connection:connectionid, &toll_cost = 0);
native FindPathAvoidTolls(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);
```

---

## 10. Paved vs. Offroad Restrictions

Prevent sports cars and luxury supercars from taking dirt tracks, sand paths, and gravel backroads:

```pawn
native bool:IsConnectionPaved(Connection:connectionid);
native SetConnectionPaved(Connection:connectionid, bool:paved);
native FindPathPavedOnly(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE);
```

---

## 11. Vehicle Height Clearances & Low Bridges

Ensure double-decker buses, semi-trailers (e.g., Linerunner, Roadtrain), and tall cargo do not crash into low overpasses or tunnel ceilings:

```pawn
native SetConnectionMaxHeight(Connection:connectionid, Float:max_height_meters);
native Float:GetConnectionMaxHeight(Connection:connectionid);
native SetAreaMaxHeight(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:max_height_meters);

native FindPathWithHeightLimit(
    MapNode:source, MapNode:target, 
    Float:vehicle_height, 
    &Path:pathid, 
    PathType:type = PATH_TYPE_VEHICLE, 
    RoutePreference:pref = ROUTE_PREF_SHORTEST
);
```

```pawn
// Semi-truck with tall container: 4.20 meters
new Path:truck_path;
FindPathWithHeightLimit(dock_node, factory_node, 4.20, truck_path);
```

---

## 12. Vehicle Category Constraints (`VehicleCategory`)

```pawn
enum VehicleCategory
{
    VEHICLE_CAT_STANDARD    = 0, // Sedans, SUVs, Coupes
    VEHICLE_CAT_HEAVY_TRUCK = 1, // Tankers, Semi-trailers, Heavy Flatbeds
    VEHICLE_CAT_MOTORCYCLE  = 2  // Motorcycles, Scooters, Bicycles
}

native FindPathForVehicleCat(
    MapNode:source, MapNode:target, &Path:pathid, 
    VehicleCategory:cat = VEHICLE_CAT_STANDARD, 
    RoutePreference:pref = ROUTE_PREF_SHORTEST
);
```
* **Heavy Trucks:** Automatically avoids narrow single-lane alleys, hairpin turns, and low-clearance bridges.
* **Motorcycles:** Can filter through narrow pedestrian pathways where vehicles are otherwise restricted.
