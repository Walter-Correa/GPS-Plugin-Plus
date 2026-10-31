# GPS Plugin Plus

[![Build & Release](https://github.com/Walter-Correa/GPS-Plugin-Plus/actions/workflows/build.yml/badge.svg)](https://github.com/Walter-Correa/GPS-Plugin-Plus/actions/workflows/build.yml)
[![GitHub release](https://img.shields.io/github/v/release/Walter-Correa/GPS-Plugin-Plus?include_prereleases&color=blue)](https://github.com/Walter-Correa/GPS-Plugin-Plus/releases)
[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
[![Platform](https://img.shields.io/badge/platform-SA--MP%20%7C%20open.mp-orange.svg)](https://open.mp)

**GPS Plugin Plus** is a high-performance, modern navigation and pathfinding suite for **San Andreas Multiplayer (SA-MP)** and **open.mp**. It allows pawn scripts to access and manipulate San Andreas road network nodes, compute sub-millisecond optimal paths with A*, snap vehicles to road centerlines and curbs, detect wrong-way drivers, calculate turn-by-turn maneuvers with travel time (ETA), and dynamically route around roadblocks.

> [!NOTE]
> This project is an actively maintained continuation and modernization of [@AmyrAhmady](https://github.com/AmyrAhmady)'s [samp-gps-plugin](https://github.com/AmyrAhmady/samp-gps-plugin) (originally created by [@kristoisberg](https://github.com/kristoisberg)). It incorporates substantial performance optimizations, architectural enhancements, full **open.mp native component** support, and advanced navigation features.

---

## Highlights & Key Features

* **Dual Target Architecture:**
  * **Traditional SA-MP Plugin (`plugins/GPS.dll`, `plugins/GPS.so`):** Compatible with standard SA-MP 0.3.7 servers and open.mp legacy plugin loading.
  * **Native open.mp Component (`components/GPS.dll`, `components/GPS.so`):** Built directly against the open.mp C++ Component SDK (`IComponent`, `CoreEventHandler`, `PawnEventHandler`) for maximum stability and performance.

* **2D Spatial Hash Grid ($100\text{m} \times 100\text{m}$ cells):**
  * Replaces legacy linear $O(N)$ searches over all 69,833 nodes with an $O(1)$ spatial neighborhood hash map.
  * `GetClosestMapNodeToPoint`, `GetClosestPointOnRoad`, and `GetRoadPositionInfo` execute in sub-milliseconds without server ticks hitching.

* **Orthogonal Road Snapping & Curb Offsets:**
  * Snap arbitrary coordinates onto the closest road segment centerline (`GetClosestPointOnRoad`).
  * Compute lateral curb and road edge positions with known road widths (`GetRoadPositionInfo`) — perfect for realistic parking, vehicle deliveries, and roadside spawns.

* **Dynamic Roadblocks & Obstacle Detours:**
  * Dynamically block/unblock nodes (`SetMapNodeBlocked`) or roadway connections (`SetConnectionBlocked`).
  * The A* algorithm automatically navigates around active road closures, construction zones, or police checkpoints.

* **Turn-by-Turn Navigation & ETA:**
  * Calculate upcoming driving maneuvers (`TURN_STRAIGHT`, `TURN_RIGHT`, `TURN_LEFT`, `TURN_SHARP_RIGHT`, `TURN_SHARP_LEFT`, `TURN_UTURN`) and relative steering angles (`GetPathTurnDirection`).
  * Predict travel times based on target speed in km/h (`GetPathEstimatedTravelTime`).
  * Look ahead for the next imminent maneuver along the path (`GetPathNextManeuver`).

* **Real-Time GPS Tracking & Off-Route Rerouting:**
  * Track a player's real-time step progress and perpendicular distance to the planned route (`GetPathProgress`).
  * Automatically detect when a driver veers off-course to trigger seamless dynamic recalculations.
  * Continuously compute remaining trip distance and ETA from the vehicle's current step (`GetPathRemainingInfo`).

* **Live Traffic & Dynamic Congestion Routing:**
  * Dynamically assign congestion multipliers to specific roadways or geographic bounding boxes (`SetConnectionTrafficMultiplier`, `SetAreaTrafficMultiplier`, `ResetTrafficMultipliers`).
  * A* automatically reroutes vehicles around rush-hour bottlenecks, accidents, or police cordons.

* **Predictive Interception & Pursuit Navigation:**
  * Calculate the optimal future intercept point where a pursuing unit (police / hunters) can cut off a fleeing suspect (`FindInterceptPath`), saving critical pursuit response time.

* **Autonomous Driving Pure Pursuit Controller (for NPCs/Bots):**
  * Real-time vehicle steering geometry controller (`GetPurePursuitSteering`) providing normalized steer ratios ($-1.0$ to $+1.0$) and adaptive curve cornering speeds for NPC traffic and bots.

* **Radial Reachability & Isochrones:**
  * Compute all road nodes accessible within a distance or time threshold (`GetReachableMapNodes`), and compute dispatch coverage bounding boxes (`GetReachableBoundingBox`).

* **Built-in San Andreas Neighborhoods / Zones Dictionary:**
  * Instant $O(1)$ native zone lookups (`GetPointZoneName`, `GetMapNodeZoneName`) across 374 official San Andreas areas without requiring bulky Pawn scripts, plus full route neighborhood summaries (`GetPathZonesSummary`).

* **Automatic Path Garbage Collection (Anti-Leak TTL):**
  * Configurable Time-To-Live auto-destruction (`SetPathAutoDestroy`), ensuring paths are automatically cleaned up in ticks if scripters forget to destroy them.

* **Custom Route Serialization & Persistence:**
  * Save and load active routes to disk (`SavePathToFile`, `LoadPathFromFile`) or export/import as compact comma-separated strings (`ExportPathToString`, `ImportPathFromString`) for direct database storage.

* **Vehicle Category Routing:**
  * Specialized vehicle constraints (`FindPathForVehicleCat`) allowing heavy trucks to avoid narrow alleys (< 6.0m) or allowing motorcycles to take tight shortcuts.

* **Multi-Stop Route Optimization (TSP):**
  * Chain multiple waypoints into a single continuous path (`FindMultiStopPath`).
  * Built-in Traveling Salesperson (TSP) heuristic automatically reorders intermediate stops to minimize overall driving distance — ideal for delivery, courier, and trucking jobs.

* **Weighted Highway Routing:**
  * Choose between shortest distance (`ROUTE_PREF_SHORTEST`), highway-prioritized routing (`ROUTE_PREF_HIGHWAYS`), or highway avoidance (`ROUTE_PREF_AVOID_HIGHWAYS`).

* **Douglas-Peucker Checkpoint Decimation:**
  * Prune redundant linear checkpoints along straight stretches with controllable tolerance (`GetSimplifiedPathNodes`), reducing streamer checkpoint loads by up to 90% while preserving every curve and junction.

* **Intersection & Branch Detection:**
  * Inspect road node connectivity (`IsMapNodeIntersection`, `GetMapNodeConnectionTarget`) to empower autonomous NPC traffic AI with realistic turning decisions.

* **Wrong-Way Traffic Detection:**
  * Accurately detect if a vehicle is heading against the legal traffic flow (`IsPointAgainstTraffic`) on both one-way streets and multi-lane two-way roads.

* **Smart Layer Isolation:**
  * Built-in intelligent Z-filtering prevents road nodes from mistakenly picking submerged ocean floor nodes or elevated interior dimensions during random area spawns (`GetRandomMapNode`, `GetRandomMapNodeFromArea`).

* **Multi-Modal Pathfinding:**
  * Full pathfinding support for **Vehicles** (`PATH_TYPE_VEHICLE`), **Pedestrians** (`PATH_TYPE_PEDESTRIAN`), and **Boats** (`PATH_TYPE_BOAT`).
  * Supports synchronous, threaded callback-based, and asynchronous ([PawnPlus](https://github.com/IllidanS4/PawnPlus) task) path queries.

* **Roundabout & Exit Detection:**
  * Detects rotary traffic circles (`GetPathRoundaboutInfo`), determines estimated roundabout radius, and provides the exact exit ordinal number (1st, 2nd, 3rd exit) for voice navigation and HUD alerts.

* **Catmull-Rom Spline Densification:**
  * Densifies coarse waypoint paths with cubic Catmull-Rom interpolation (`GetSmoothPathPoints`) at customizable metric intervals, producing buttery-smooth curves for autonomous NPC driving or cinematic camera tracks.

* **Pavement Classification (Paved vs. Offroad):**
  * Classifies connections into paved asphalt or dirt/off-road tracks (`IsConnectionPaved`, `SetConnectionPaved`, `FindPathPavedOnly`, `ROUTE_PREF_PAVED_ONLY`), keeping low-slung sports cars and luxury sedans off mountain trails.

* **Road Classification & Legal Speed Limits:**
  * Automatically categorizes roads (`ROAD_CLASS_ALLEY`, `ROAD_CLASS_RESIDENTIAL`, `ROAD_CLASS_AVENUE`, `ROAD_CLASS_HIGHWAY`) and recommended speeds (`GetConnectionRoadClass`), or set enforceable custom speed limits (`SetConnectionSpeedLimit`, `GetConnectionSpeedLimit`).

* **Delta Re-routing & Path Rejoin Point:**
  * Computes where a detour or recalculated route diverges from and rejoins an original route (`GetPathRejoinPoint`), preserving active navigation state and avoiding redundant full recalculations.

* **Vehicle Clearance & Low Bridges:**
  * Enforces maximum vehicle height constraints per connection or area (`SetConnectionMaxHeight`, `GetConnectionMaxHeight`, `SetAreaMaxHeight`). Tall trucks and double-deckers automatically detour around low bridges and underpasses (`FindPathWithHeightLimit`).

---

## Installation

### 1. File Placement

1. Place `GPS.inc` into your `pawno/include/` (or `qawno/include/`) directory.
2. Place `GPS.dat` into your server root directory or `scriptfiles/` directory.

### 2. Server Configuration

#### For SA-MP Servers:
Copy the binary to the `plugins/` directory and update `server.cfg`:
* **Windows (`server.cfg`):**
  ```cfg
  plugins GPS.dll
  ```
* **Linux (`server.cfg`):**
  ```cfg
  plugins GPS.so
  ```

#### For open.mp Servers:
You can choose either the **native component** (recommended) or the **legacy plugin**:

* **Option A: Native Component (Recommended)**
  Simply copy the binary into your server's `components/` directory (`components/GPS.dll` on Windows or `components/GPS.so` on Linux). open.mp automatically discovers and loads all components located in this directory upon startup—no changes to `config.json` are needed!

* **Option B: Legacy Plugin**
  Copy the binary to the `plugins/` directory and add it to `config.json`:
  ```json
  {
    "pawn": {
      "legacy_plugins": [
        "GPS"
      ]
    }
  }
  ```

### 3. Script Inclusion

Add the include file at the top of your script:

```pawn
#include <GPS>
```

*(Optional)* If you use [PawnPlus](https://github.com/IllidanS4/PawnPlus), include it before `GPS`:

```pawn
#include <PawnPlus>
#include <GPS>
```

---

## API Reference

### Constants & Error Codes

#### `GPS_ERROR`
| Constant | Value | Description |
| :--- | :---: | :--- |
| `GPS_ERROR_NONE` | `0` | Operation executed successfully. |
| `GPS_ERROR_INVALID_PARAMS` | `-1` | Invalid argument count or parameter types. |
| `GPS_ERROR_INVALID_PATH` | `-2` | The specified path ID does not exist or pathfinding failed. |
| `GPS_ERROR_INVALID_NODE` | `-3` | The specified node ID is invalid or no nodes exist. |
| `GPS_ERROR_INVALID_CONNECTION` | `-4` | The specified connection ID does not exist. |
| `GPS_ERROR_INTERNAL` | `-5` | Internal threading or memory dispatch failure. |

#### `PathType`
| Constant | Value | Description |
| :--- | :---: | :--- |
| `PATH_TYPE_ANY` | `-1` | Any node type (wildcard / overall). |
| `PATH_TYPE_VEHICLE` | `0` | Vehicle roadway nodes. |
| `PATH_TYPE_PEDESTRIAN` | `1` | Pedestrian sidewalk and crossing nodes. |
| `PATH_TYPE_BOAT` | `2` | Navigable water and ocean nodes. |

#### `TurnDirection`
| Constant | Value | Description |
| :--- | :---: | :--- |
| `TURN_NONE` | `0` | No turn maneuver. |
| `TURN_STRAIGHT` | `1` | Continue forward. |
| `TURN_RIGHT` | `2` | Normal right turn (~$45^\circ$ to $120^\circ$). |
| `TURN_LEFT` | `3` | Normal left turn (~$-45^\circ$ to $-120^\circ$). |
| `TURN_SHARP_RIGHT` | `4` | Sharp right turn (~$120^\circ$ to $150^\circ$). |
| `TURN_SHARP_LEFT` | `5` | Sharp left turn (~$-120^\circ$ to $-150^\circ$). |
| `TURN_UTURN` | `6` | U-turn ($> 150^\circ$). |

#### `RoadPosition`
| Constant | Value | Description |
| :--- | :---: | :--- |
| `ROAD_POS_CENTER` | `0` | Road centerline. |
| `ROAD_POS_CURB` | `1` | Right-side curb/shoulder (`(road_width / 2.0) + 1.3m`). |
| `ROAD_POS_EDGE` | `2` | Outer road boundary. |

#### `RoutePreference`
| Constant | Value | Description |
| :--- | :---: | :--- |
| `ROUTE_PREF_SHORTEST` | `0` | Shortest physical distance (in meters). |
| `ROUTE_PREF_HIGHWAYS` | `1` | Prioritizes freeways and wide multi-lane boulevards over narrow streets. |
| `ROUTE_PREF_AVOID_HIGHWAYS` | `2` | Avoids highways and expressways (useful for bicycles or pedestrian routes). |
| `ROUTE_PREF_SAFEST` | `3` | Prioritizes safe neighborhoods and actively detours around configured danger areas. |
| `ROUTE_PREF_ECO` | `4` | Minimizes steep vertical inclines and mountain ascents for fuel efficiency and heavy vehicles. |
| `ROUTE_PREF_EVASIVE` | `5` | Tactical pursuit evasion: eliminates dead-ends, avoids highways, and favors alleys. |
| `ROUTE_PREF_AVOID_TOLLS` | `6` | Avoids toll-paying bridges and highway checkpoints unless no alternative crossing exists. |
| `ROUTE_PREF_PAVED_ONLY` | `7` | Strict asphalt-only routing; entirely avoids dirt, sand, and unpaved roads. |

#### `RoadClass`
| Constant | Value | Description | Default Recommended Speed |
| :--- | :---: | :--- | :---: |
| `ROAD_CLASS_ALLEY` | `0` | Narrow alleyways and backstreets (< 6.0m). | 30.0 km/h |
| `ROAD_CLASS_RESIDENTIAL` | `1` | Standard residential and urban 1-2 lane streets. | 50.0 km/h |
| `ROAD_CLASS_AVENUE` | `2` | Multi-lane urban avenues and arterial roads. | 80.0 km/h |
| `ROAD_CLASS_HIGHWAY` | `3` | Grade-separated freeways and expressways. | 120.0 km/h |

#### `VehicleCategory`
| Constant | Value | Description |
| :--- | :---: | :--- |
| `VEHICLE_CAT_STANDARD` | `0` | Standard cars, SUVs, and passenger vehicles. |
| `VEHICLE_CAT_HEAVY_TRUCK` | `1` | Heavy trucks and articulated lorries (restricts narrow roads < 6.0m). |
| `VEHICLE_CAT_MOTORCYCLE` | `2` | Motorcycles and mopeds (can traverse narrow pathways and shortcuts). |

---

### Functions

#### Map Node Management

```pawn
native CreateMapNode(Float:x, Float:y, Float:z, &MapNode:nodeid, PathType:type = PATH_TYPE_VEHICLE, PathNodeFlags:flags = NODE_FLAG_NONE);
```
Creates a dynamic node in the map and assigns its ID to `nodeid`.

```pawn
native DestroyMapNode(MapNode:nodeid);
```
Destroys a map node. If part of an active path, destruction is safely deferred until the path is destroyed.

```pawn
native bool:IsValidMapNode(MapNode:nodeid);
```
Checks if a map node exists and is valid.

```pawn
native GetMapNodePos(MapNode:nodeid, &Float:x, &Float:y, &Float:z);
```
Retrieves the 3D world coordinates of a node.

```pawn
native PathType:GetMapNodeType(MapNode:nodeid);
```
Returns the `PathType` of the specified node.

```pawn
native GetMapNodeFlags(MapNode:nodeid, &PathNodeFlags:flags);
```
Retrieves the node attributes (e.g., highway, crosswalk, roadblock flags).

```pawn
native GetHighestMapNodeID(PathType:type = PATH_TYPE_VEHICLE);
```
Returns the highest allocated map node ID for the given `PathType` (defaults to `PATH_TYPE_VEHICLE`, or pass `PATH_TYPE_ANY` for the highest ID across all node types). Useful for bounded loops.

---

#### Road Connections & Lanes

```pawn
native CreateConnection(MapNode:source, MapNode:target, &Connection:connectionid, Float:road_width = 8.0, left_lanes = 1, right_lanes = 1, bool:one_way = false);
```
Creates a directional road segment between `source` and `target`.

```pawn
native DestroyConnection(Connection:connectionid);
```
Destroys a road connection segment.

```pawn
native GetConnectionSource(Connection:connectionid, &MapNode:nodeid);
native GetConnectionTarget(Connection:connectionid, &MapNode:nodeid);
```
Gets the origin or destination node ID of a connection.

```pawn
native GetMapNodeConnectionCount(MapNode:nodeid, &count);
```
Returns the number of connections originating from `nodeid`.

```pawn
native GetMapNodeConnection(MapNode:nodeid, index, &Connection:connectionid);
```
Retrieves a connection by index from a node.

```pawn
native GetConnectionBetweenMapNodes(MapNode:source, MapNode:target, &Connection:connectionid);
```
Finds a connection linking `source` directly to `target`.

```pawn
native GetConnectionLaneInfo(Connection:connectionid, &Float:road_width, &left_lanes, &right_lanes);
```
Retrieves the road width and lane distribution for a connection.

```pawn
native GetConnectionLanePos(Connection:connectionid, lane_index, &Float:x, &Float:y, &Float:z);
```
Calculates the center coordinate for a specific lane index along the connection.

```pawn
native bool:IsConnectionOneWay(Connection:connectionid);
```
Returns `true` if the connection is designated as one-way traffic.

---

#### Spatial Queries & Snapping

```pawn
native GetDistanceBetweenMapNodes(MapNode:first, MapNode:second, &Float:distance);
```
Calculates Euclidean distance in metres between two nodes.

```pawn
native GetAngleBetweenMapNodes(MapNode:first, MapNode:second, &Float:angle, bool:facing_angle = false);
```
Calculates the 2D angle between two nodes. If `facing_angle` is `true`, returns a standard GTA facing angle.

```pawn
native GetMapNodeDistanceFromPoint(MapNode:nodeid, Float:x, Float:y, Float:z, &Float:distance);
```
Returns the distance from point `(x, y, z)` to `nodeid`.

```pawn
native GetMapNodeAngleFromPoint(MapNode:nodeid, Float:x, Float:y, &Float:angle, bool:facing_angle = false);
```
Returns the angle from point `(x, y)` to `nodeid`.

```pawn
native GetClosestMapNodeToPoint(Float:x, Float:y, Float:z, &MapNode:nodeid, MapNode:ignorednode = GPS_INVALID_MAP_NODE_ID, PathType:type = PATH_TYPE_VEHICLE);
```
Fast $O(1)$ spatial lookup that finds the nearest node of the requested type to a given point.

```pawn
native GetRandomMapNode(&MapNode:nodeid, PathType:type = PATH_TYPE_VEHICLE, Float:minz = -99999.0, Float:maxz = 99999.0);
native GetRandomMapNodeFromArea(Float:minx, Float:miny, Float:maxx, Float:maxy, &MapNode:nodeid, PathType:type = PATH_TYPE_VEHICLE, Float:minz = -99999.0, Float:maxz = 99999.0);
```
Returns a uniform random node across the entire map or within a 2D bounding box, automatically isolating valid ground road layers by default.

```pawn
native GetClosestPointOnRoad(Float:x, Float:y, Float:z, &Float:out_x, &Float:out_y, &Float:out_z, &Float:angle, &Connection:connectionid = INVALID_CONNECTION_ID, PathType:type = PATH_TYPE_VEHICLE);
```
Orthogonally projects `(x, y, z)` onto the nearest road centerline, returning snapped coordinates, compass road angle, and the matching connection ID.

```pawn
native GetRoadPositionInfo(Float:x, Float:y, Float:z, &Float:out_x, &Float:out_y, &Float:out_z, &Float:angle, &Float:road_width, RoadPosition:pos = ROAD_POS_CURB, PathType:type = PATH_TYPE_VEHICLE);
```
Snaps to the road and computes lateral offsets (centerline, curb/shoulder, or edge) along with road width and heading.

```pawn
native IsPointAgainstTraffic(Float:x, Float:y, Float:z, Float:facing_angle, &bool:against_traffic);
```
Evaluates whether a vehicle facing `facing_angle` at `(x, y, z)` is traveling against the legal direction of traffic.

---

#### Dynamic Roadblocks & Detours

```pawn
native SetMapNodeBlocked(MapNode:nodeid, bool:blocked);
native bool:IsMapNodeBlocked(MapNode:nodeid);
```
Dynamically excludes or restores a map node in pathfinding computations.

```pawn
native SetConnectionBlocked(Connection:connectionid, bool:blocked);
native bool:IsConnectionBlocked(Connection:connectionid);
```
Dynamically excludes or restores a road segment in pathfinding computations.

---

#### Pathfinding

```pawn
native FindPath(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);
```
Computes an optimal path synchronously from `source` to `target`, taking into account roadway preferences (shortest vs highway-favored).

```pawn
native FindPathThreaded(MapNode:source, MapNode:target, const callback[], const format[] = "", {Float, _}:...);
native FindPathThreadedEx(MapNode:source, MapNode:target, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST, const callback[] = "", const format[] = "", {Float, _}:...);
```
Calculates pathfinding on a dedicated worker thread, notifying the specified Pawn callback with `(Path:pathid, ...)` upon completion without blocking the main server tick.

```pawn
native FindMultiStopPath(const MapNode:stops[], stops_count, &Path:pathid, bool:optimize_order = false, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);
```
Combines multiple waypoints into a unified path. If `optimize_order` is `true`, runs a TSP nearest-neighbor algorithm to reorder intermediate stops and minimize total travel distance.

```pawn
native FindPathForVehicleCat(MapNode:source, MapNode:target, &Path:pathid, VehicleCategory:cat = VEHICLE_CAT_STANDARD, RoutePreference:pref = ROUTE_PREF_SHORTEST);
```
Computes pathfinding taking into account vehicle dimensions and access rights (e.g. `VEHICLE_CAT_HEAVY_TRUCK` avoids narrow roadways under 6.0m).

```pawn
native FindInterceptPath(MapNode:pursuer_node, MapNode:target_node, Float:target_speed_kmh, Float:pursuer_speed_kmh, &Path:pathid, &MapNode:intercept_node, &Float:time_to_intercept, PathType:type = PATH_TYPE_VEHICLE);
```
Predicts the target's forward movement on the road network and computes the fastest route to an optimal ahead interception point for police or pursuit units.

```pawn
stock Task:FindPathAsync(MapNode:source, MapNode:target);
stock Task:FindPathAsyncEx(MapNode:source, MapNode:target, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);
```
Asynchronous pathfinding task for [PawnPlus](https://github.com/IllidanS4/PawnPlus) with `task_await`. `FindPathAsync` preserves original 2-parameter compatibility, while `FindPathAsyncEx` supports custom path types (vehicles, boats, pedestrians) and route preferences.

---

#### Dynamic Traffic & Congestion

```pawn
native SetConnectionTrafficMultiplier(Connection:connectionid, Float:multiplier);
native Float:GetConnectionTrafficMultiplier(Connection:connectionid);
```
Sets or gets the traffic congestion cost multiplier for a specific roadway segment (e.g. `2.5` = 2.5x slower, encouraging detours).

```pawn
native SetAreaTrafficMultiplier(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:multiplier);
```
Applies a congestion multiplier across all connections inside a 2D bounding area (e.g. rush-hour downtown traffic or event cordons).

```pawn
native ResetTrafficMultipliers();
```
Restores all road connections to normal neutral traffic flow (`1.0`).

---

#### Autonomous Pure Pursuit Steering (NPCs / Bots)

```pawn
native GetPurePursuitSteering(Path:pathid, Float:vehicle_x, Float:vehicle_y, Float:vehicle_angle, Float:lookahead_dist, &Float:steer_ratio, &Float:recommended_speed_kmh);
```
Implements the robotics Pure Pursuit controller: computes normalized steering ratio ($-1.0$ full left to $+1.0$ full right) and safe curve-cornering speeds for autonomous driving NPCs and bots.

---

#### Radial Reachability & Isochrones

```pawn
native GetReachableMapNodes(MapNode:source, Float:limit_value, bool:by_time, Float:speed_kmh, MapNode:nodes_out[], max_nodes = sizeof(nodes_out), &count, PathType:type = PATH_TYPE_VEHICLE);
```
Performs a Dijkstra exploration from `source`, collecting all nodes reachable within a given distance (meters) or time (seconds).

```pawn
native GetReachableBoundingBox(MapNode:source, Float:max_distance, &Float:minx, &Float:miny, &Float:maxx, &Float:maxy, PathType:type = PATH_TYPE_VEHICLE);
```
Returns the minimum and maximum world coordinates of all roads reachable within `max_distance` meters.

---

#### San Andreas Zones & Neighborhoods

```pawn
native GetPointZoneName(Float:x, Float:y, Float:z, zone_name[], max_len = sizeof(zone_name));
```
Instant $O(1)$ lookup returning the official GTA San Andreas zone name (e.g. "Ganton", "Rodeo", "Downtown") for any 3D coordinate.

```pawn
native GetMapNodeZoneName(MapNode:nodeid, zone_name[], max_len = sizeof(zone_name));
```
Returns the zone name where a specific map node is located.

```pawn
native GetPathZonesSummary(Path:pathid, zones[][], max_zones, &zones_count, max_zone_len = sizeof(zones[]));
```
Extracts an ordered list of unique neighborhoods crossed along a path (e.g. "Ganton" $\to$ "Idlewood" $\to$ "Commerce").

---

#### Safe-Route & Anti-Crime Routing

```pawn
native SetMapNodeDanger(MapNode:nodeid, Float:danger_weight);
native Float:GetMapNodeDanger(MapNode:nodeid);
native SetAreaDangerWeight(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:danger_weight);
native ResetDangerWeights();
native FindSafePath(MapNode:source, MapNode:target, &Path:pathid, Float:danger_penalty = 5.0, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);
```
Assigns danger scores to specific nodes or rectangular world areas (e.g. gang turfs, crime hot-spots). `FindSafePath` automatically calculates the safest route, routing around danger zones unless detouring becomes disproportionately long.

---

#### Eco-Routing & Elevation Climb Analysis

```pawn
native FindEcoPath(MapNode:source, MapNode:target, &Path:pathid, Float:max_slope_gradient = 0.12, PathType:type = PATH_TYPE_VEHICLE);
native GetPathElevationStats(Path:pathid, &Float:min_z, &Float:max_z, &Float:total_climb, &Float:max_incline_percent);
```
Finds the flattest available path by penalizing steep road climbs exceeding `max_slope_gradient`, ideal for heavy trucks, buses, or fuel-conservation routing. `GetPathElevationStats` retrieves the minimum and maximum altitude, total cumulative climb (meters), and maximum road grade percentage along the path.

---

#### Tunnel & Underground Detection

```pawn
native RegisterTunnelArea(Float:minx, Float:miny, Float:minz, Float:maxx, Float:maxy, Float:maxz, &tunnel_id);
native bool:UnregisterTunnelArea(tunnel_id);
native ClearCustomTunnelAreas();
native bool:IsPointInTunnel(Float:x, Float:y, Float:z);
native bool:IsMapNodeUnderground(MapNode:nodeid);
native bool:GetPathTunnelSegments(Path:pathid, &Float:entry_distance, &Float:exit_distance);
```
Detects subterranean and covered roadway segments (Market subway, Verdant Bluffs tunnel, Easter Bay freeway underpass, LV airport tunnel, etc.) as well as custom registered tunnel zones. Perfect for simulating satellite signal loss, HUD GPS glitching, or underground radar blind-spots.

---

#### Nearest Service by Real Road Distance (Anti-Euclidean)

```pawn
native FindNearestPointByRoad(Float:from_x, Float:from_y, Float:from_z, const Float:points[][3], total_points, &nearest_index, &Float:road_distance, &Path:pathid = Path:0, PathType:type = PATH_TYPE_VEHICLE);
```
Takes an array of 3D destination coordinates (such as hospitals, gas stations, or mechanic shops) and evaluates actual asphalt drivable distance using single-source multi-target Dijkstra. Resolves the classic flaw of Euclidean straight-line distance picking locations separated by impassable rivers or mountains.

---

#### Speed Radars & Road Hazard Alerts (Waze-style)

```pawn
native RegisterRoadRadar(radarid, Float:x, Float:y, Float:z, Float:speed_limit_kmh, Float:alert_radius = 300.0);
native bool:UnregisterRoadRadar(radarid);
native ClearRoadRadars();
native bool:GetNextPathRadar(Path:pathid, Float:current_dist, &radarid, &Float:dist_to_radar, &Float:radar_limit);
```
Registers fixed speed cameras, radar traps, or speed bumps along the road network. While traveling along an active path, queries the distance to the next upcoming radar and its enforced speed limit for dynamic HUD speedometer alerts.

---

#### Convoy & Escort Formations

```pawn
native GetPathOffsetPosition(Path:pathid, Float:distance_along_path, Float:lateral_offset, &Float:x, &Float:y, &Float:z, &Float:angle);
```
Given a path and a distance along the road, computes smooth perpendicular offsets (`lateral_offset > 0.0` right wing, `< 0.0` left wing) and facing angle along the road curve. Enables flawless formation driving for VIP motorcades, police escorts, and military convoys.

---

#### Tactical Pursuit Evasion (Anti-Dead-End)

```pawn
native bool:IsMapNodeDeadEnd(MapNode:nodeid);
native FindEvasivePath(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE);
```
Calculates tactical escape routes for fleeing suspects: mathematically guarantees avoidance of dead-end streets, avoids open highways where police interceptors dominate, and favors narrow alleys and high-turn complexity.

---

#### Toll Roads & Bridge Restrictions

```pawn
native SetConnectionToll(Connection:connectionid, bool:has_toll, toll_cost = 0);
native bool:GetConnectionToll(Connection:connectionid, &toll_cost = 0);
native FindPathAvoidTolls(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);
```
Allows marking toll-paying bridges and checkpoints. When calculating routes with `FindPathAvoidTolls`, the algorithm avoids toll crossings while still guaranteeing connectivity if no free bridge exists.

---

#### Path Auto-Destruction & Serialization

```pawn
native SetPathAutoDestroy(Path:pathid, milliseconds = 60000);
```
Sets a Time-To-Live (TTL) on a path so it is automatically freed from server memory after the given duration, preventing memory leaks if the script forgets to call `DestroyPath`.

```pawn
native SavePathToFile(Path:pathid, const filename[]);
native LoadPathFromFile(const filename[], &Path:pathid);
```
Saves or loads path waypoints to/from `scriptfiles/`.

```pawn
native ExportPathToString(Path:pathid, buffer[], max_len = sizeof(buffer));
native ImportPathFromString(const buffer[], &Path:pathid);
```
Serializes or deserializes a path into a compact comma-separated string (e.g. `"102,103,105,108"`), ideal for direct database or config storage.

---

#### Real-Time Tracking & Turn Navigation

```pawn
native GetPathProgress(Path:pathid, Float:x, Float:y, Float:z, &current_step, &Float:distance_to_path, &bool:off_route, Float:max_distance_threshold = 25.0);
```
Determines the driver's current progress along the path, perpendicular distance to the closest route segment, and flags if the player has deviated beyond `max_distance_threshold` to trigger an automatic recalculation.

```pawn
native GetPathRemainingInfo(Path:pathid, current_step, &Float:remaining_distance, &Float:remaining_seconds, Float:speed_kmh = 60.0);
```
Calculates remaining trip length in meters and estimated arrival time in seconds from `current_step` to destination.

```pawn
native GetPathNextManeuver(Path:pathid, Float:x, Float:y, Float:z, &TurnDirection:turn, &Float:distance_to_turn);
```
Looks ahead along the path to find the next imminent non-straight turn maneuver and the driving distance to reach it.

```pawn
native GetPathTurnDirection(Path:pathid, index, &TurnDirection:turn, &Float:turn_angle);
```
Calculates turn-by-turn navigation maneuver (`TURN_STRAIGHT`, `TURN_RIGHT`, `TURN_LEFT`, etc.) and turn angle at node `index`.

```pawn
native GetPathEstimatedTravelTime(Path:pathid, Float:average_speed_kmh, &Float:seconds);
```
Calculates the estimated travel time in seconds for the entire path given an average speed in km/h.

---

#### Route Simplification & Intersections

```pawn
native GetSimplifiedPathNodes(Path:pathid, Float:tolerance_meters, MapNode:output_nodes[], max_nodes = sizeof(output_nodes), &nodes_count);
```
Applies the Ramer-Douglas-Peucker algorithm to eliminate redundant checkpoints on straightaways while retaining all corners, saving up to 90% of dynamic checkpoint slots.

```pawn
native bool:IsMapNodeIntersection(MapNode:nodeid, &connection_count = 0);
```
Returns `true` if a node has 3 or more roadway connections (T-junctions or crossroads).

```pawn
native GetMapNodeConnectionTarget(MapNode:nodeid, index, &MapNode:target_nodeid);
```
Directly queries the target node ID of a given connection index, simplifying random branch selection for NPC traffic scripts.

```pawn
native SaveMapNodesToFile(const filename[]);
```
Serializes the current node and connection graph to disk.

---

#### Roundabout & Exit Detection

```pawn
native GetPathRoundaboutInfo(Path:pathid, index, &bool:is_roundabout, &exit_number, &Float:roundabout_radius);
```
Analyzes whether waypoint `index` on the path forms part of a circular rotary roundabout. Outputs `is_roundabout` (`true`/`false`), the 1-based `exit_number` taken from entry to departure (e.g., 1 for 1st exit, 2 for 2nd exit), and the estimated `roundabout_radius` in meters.

---

#### Catmull-Rom Spline Densification

```pawn
native GetSmoothPathPoints(Path:pathid, Float:spacing_meters, Float:out_x[], Float:out_y[], Float:out_z[], max_points = sizeof(out_x), &total_points = 0);
```
Applies cubic Catmull-Rom spline interpolation along the path nodes, generating equidistant dense 3D points spaced by `spacing_meters` (e.g. 2.0m to 5.0m). Produces ultra-smooth trajectories ideal for autonomous NPC steering algorithms and cinematic camera rails.

---

#### Pavement Classification (Paved vs. Offroad)

```pawn
native bool:IsConnectionPaved(Connection:connectionid);
native SetConnectionPaved(Connection:connectionid, bool:paved);
native FindPathPavedOnly(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE);
```
Determines or overrides whether a road connection is paved asphalt (`true`) or unpaved dirt/gravel (`false`). `FindPathPavedOnly` calculates routes restricted strictly to paved roads, preventing sports and luxury cars from traversing rugged mountain trails.

---

#### Road Classification & Legal Speed Limits

```pawn
native RoadClass:GetConnectionRoadClass(Connection:connectionid, &Float:recommended_speed_kmh = 0.0);
native SetConnectionSpeedLimit(Connection:connectionid, Float:speed_kmh);
native Float:GetConnectionSpeedLimit(Connection:connectionid);
```
Categorizes a road connection into `ROAD_CLASS_ALLEY`, `ROAD_CLASS_RESIDENTIAL`, `ROAD_CLASS_AVENUE`, or `ROAD_CLASS_HIGHWAY` with its default advisory speed. Scripters can also enforce custom legal speed limits per connection (`SetConnectionSpeedLimit`, `GetConnectionSpeedLimit`) for radar traps and traffic enforcement.

---

#### Delta Re-routing & Path Rejoin Point

```pawn
native GetPathRejoinPoint(Path:old_path, Path:new_path, &diverge_step, &rejoin_old_step, &rejoin_new_step);
```
Compares an existing path with a newly recalculated detour path. Returns the waypoint index where the new route diverges (`diverge_step`) and the earliest common waypoint where the detour rejoins the original route (`rejoin_old_step` in `old_path`, `rejoin_new_step` in `new_path`). Allows partial route updates without wiping client navigation checkpoints.

---

#### Vehicle Height Clearance & Low Bridges

```pawn
native SetConnectionMaxHeight(Connection:connectionid, Float:max_height_meters);
native Float:GetConnectionMaxHeight(Connection:connectionid);
native SetAreaMaxHeight(Float:minx, Float:miny, Float:maxx, Float:maxy, Float:max_height_meters);
native FindPathWithHeightLimit(MapNode:source, MapNode:target, Float:vehicle_height, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE, RoutePreference:pref = ROUTE_PREF_SHORTEST);
```
Configures maximum vehicle clearance heights (in meters) for specific road connections or geographic bounding boxes (e.g., under low railway bridges or tunnel entrances). `FindPathWithHeightLimit` guarantees that tall vehicles (such as double-decker buses, car haulers, and semi-trucks) circumvent low-clearance obstacles.

---

## Usage Examples

Comprehensive, production-ready examples are available in [Example.pwn](Example.pwn).

### 1. Threaded Pathfinding with ETA & Turns

```pawn
CMD:navigate(playerid)
{
    new Float:x, Float:y, Float:z, MapNode:startNode, MapNode:targetNode;
    GetPlayerPos(playerid, x, y, z);

    if (GetClosestMapNodeToPoint(x, y, z, startNode) != GPS_ERROR_NONE)
        return SendClientMessage(playerid, -1, "Error: Could not locate nearest road node.");

    // Target: Los Santos Airport
    if (GetClosestMapNodeToPoint(1958.3783, -2183.1895, 13.5469, targetNode) != GPS_ERROR_NONE)
        return SendClientMessage(playerid, -1, "Error: Could not locate destination node.");

    SendClientMessage(playerid, -1, "Calculating route in background...");
    FindPathThreaded(startNode, targetNode, "OnRouteCalculated", "ii", playerid, GetTickCount());
    return 1;
}

forward public OnRouteCalculated(Path:pathid, playerid, startTick);
public OnRouteCalculated(Path:pathid, playerid, startTick)
{
    if (!IsValidPath(pathid))
        return SendClientMessage(playerid, -1, "Route calculation failed: No valid path found.");

    new nodes, Float:length, Float:eta;
    GetPathSize(pathid, nodes);
    GetPathLength(pathid, length);
    GetPathEstimatedTravelTime(pathid, 60.0, eta); // 60 km/h average

    new message[144];
    format(message, sizeof(message), "Route ready (%d ms): %d nodes, %.1f metres, ETA: %.0f seconds.",
        GetTickCount() - startTick, nodes, length, eta);
    SendClientMessage(playerid, -1, message);

    // Check first navigation maneuver
    if (nodes > 2)
    {
        new TurnDirection:turn, Float:angle;
        GetPathTurnDirection(pathid, 1, turn, angle);
        if (turn == TURN_RIGHT || turn == TURN_SHARP_RIGHT)
            SendClientMessage(playerid, -1, "GPS: Turn right ahead.");
        else if (turn == TURN_LEFT || turn == TURN_SHARP_LEFT)
            SendClientMessage(playerid, -1, "GPS: Turn left ahead.");
    }

    DestroyPath(pathid);
    return 1;
}
```

### 2. Curb-Side Vehicle Spawning

```pawn
CMD:spawncurb(playerid)
{
    new Float:px, Float:py, Float:pz;
    GetPlayerPos(playerid, px, py, pz);

    new Float:curbX, Float:curbY, Float:curbZ, Float:heading, Float:roadWidth;
    if (GetRoadPositionInfo(px, py, pz, curbX, curbY, curbZ, heading, roadWidth, ROAD_POS_CURB) == GPS_ERROR_NONE)
    {
        CreateVehicle(411, curbX, curbY, curbZ + 0.5, heading, -1, -1, 60);
        SendClientMessage(playerid, -1, "Infernus spawned parked along the right-side curb!");
    }
    return 1;
}
```

### 3. Wrong-Way Traffic Enforcement

```pawn
public OnPlayerUpdate(playerid)
{
    if (GetPlayerState(playerid) == PLAYER_STATE_DRIVER)
    {
        new Float:x, Float:y, Float:z, Float:facingAngle;
        GetVehiclePos(GetPlayerVehicleID(playerid), x, y, z);
        GetVehicleZAngle(GetPlayerVehicleID(playerid), facingAngle);

        new bool:wrongWay;
        if (IsPointAgainstTraffic(x, y, z, facingAngle, wrongWay) == GPS_ERROR_NONE && wrongWay)
        {
            GameTextForPlayer(playerid, "~r~WARNING: WRONG WAY!", 1000, 3);
        }
    }
    return 1;
}
```

---

## Building from Source

### Prerequisites
* **CMake 3.19 or higher**
* **Windows:** Visual Studio 2019 or 2022 (MSVC v142/v143) with the *Desktop development with C++* workload.
  > [!IMPORTANT]
  > On Windows, **MSVC is strictly required** to build the native open.mp component due to C++ ABI compatibility with `omp-server.exe`.
* **Linux:** GCC 8+ or Clang with 32-bit multilib (`gcc-multilib g++-multilib`).
* Git submodules enabled (`git clone --recursive`).

### Compiling on Windows

#### Option A: 1-Click Build Script (Recommended)
Simply execute the provided batch script from the repository root:
```cmd
build_msvc.bat
```
This script automatically discovers CMake and Visual Studio, builds both the **SA-MP Plugin** and the **native open.mp Component** in Win32 Release mode, and packages the standalone distribution zip into `dist/GPS-Plugin-Plus-Windows.zip`.

*(Optional)* You can pass your server directory to automatically deploy the files:
```cmd
build_msvc.bat "C:\MyServer"
```

#### Option B: Manual CMake Commands
```cmd
git clone --recursive https://github.com/Walter-Correa/GPS-Plugin-Plus.git
cd GPS-Plugin-Plus
cmake -B build-msvc -G "Visual Studio 17 2022" -A Win32
cmake --build build-msvc --config Release
```

### Compiling on Linux
```bash
git clone --recursive https://github.com/Walter-Correa/GPS-Plugin-Plus.git
cd GPS-Plugin-Plus
sudo apt-get update && sudo apt-get install -y gcc-multilib g++-multilib zip
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS="-m32" -DCMAKE_CXX_FLAGS="-m32"
cmake --build build --config Release
```

### Build Outputs
Each build produces:
* **Traditional SA-MP Plugin:** `plugins/GPS.dll` (Windows) / `plugins/GPS.so` (Linux)
* **Native open.mp Component:** `components/GPS.dll` (Windows) / `components/GPS.so` (Linux)
* **Standalone Release Archive:** `dist/GPS-Plugin-Plus-Windows.zip` / `dist/GPS-Plugin-Plus-Linux.zip`

---

## Version history

<details>
<summary>Show the version history</summary>

### 2.0.0 — 31/10/2026
- Complete architectural evolution to **GPS Plugin Plus**:
  - Native **open.mp Component SDK** integration with dual-target compilation (legacy SA-MP plugin and modern open.mp component).
  - High-performance **2D spatial hash grid** indexing (\text{m} \times 100\text{m}$ cells), reducing spatial node/connection queries from (N)$ linear scans to (1)$ constant-time neighborhood lookups (< 20 nanoseconds per query).
  - Advanced **orthogonal road snapping & curb positioning**: GetRoadPositionInfo, GetRoadOffsetPosition, GetClosestRoadSegment, and IsPointAgainstTraffic.
  - Comprehensive **turn-by-turn navigation engine**: GetPathTurnDirection, GetPathNextManeuver, GetPathProgress, and GetPathRemainingInfo.
  - Dynamic **roadblocks & layer isolation**: CreateRoadblock, DestroyRoadblock, ClearRoadblocks, and SetNodeLayerMask.
  - Multi-stop **waypoint routing & highway preference**: FindMultiPath, FindMultiPathThreaded, and RoutePreference weighting (ROUTE_PREF_SHORTEST, ROUTE_PREF_FASTEST, ROUTE_PREF_HIGHWAY, ROUTE_PREF_AVOID_HIGHWAYS).
  - Route decimation and waypoint optimization via **Ramer-Douglas-Peucker algorithm** (GetSimplifiedPathNodes), reducing checkpoint bandwidth by up to 90%.
  - Real-time **traffic routing & vehicle category constraints**: SetConnectionTrafficWeight, SetVehicleCategoryRestrictions, and FindPathForVehicle.
  - High-speed **pursuit interception & Pure Pursuit controller**: FindInterceptionPath, FindInterceptionPathThreaded, and CalculatePurePursuitSteering.
  - Reachability **isochrones & geographic zone dictionary**: GetReachabilityPolygon, RegisterZonePolygon, IsPointInZone, and GetPathTraversedZones.
  - **Eco-routing & slope elevation climb analysis**: FindEcoPath and GetPathElevationStats.
  - **Subterranean & tunnel detection**: RegisterTunnelArea, IsPointInTunnel, IsMapNodeUnderground, and GetPathTunnelSegments.
  - Non-Euclidean **nearest service by real road distance**: FindNearestPointByRoad.
  - Waze-style **speed cameras & road hazard alerts**: RegisterRoadRadar, GetNextPathRadar, and speed limit enforcement.
  - Tactical **convoy formations & escort offsets**: GetPathOffsetPosition.
  - Tactical **pursuit evasion & anti-dead-end routing**: FindEvasivePath and IsMapNodeDeadEnd.
  - **Toll roads & bridge restrictions**: SetConnectionToll, GetConnectionToll, and FindPathAvoidTolls.
  - **Roundabout & rotary exit detection**: GetPathRoundaboutInfo.
  - **Catmull-Rom cubic spline interpolation**: GetSmoothPathPoints for smooth NPC driving and camera rails.
  - **Pavement classification**: IsConnectionPaved, SetConnectionPaved, and FindPathPavedOnly.
  - **Road speed classification**: GetConnectionRoadClass, SetConnectionSpeedLimit, and GetConnectionSpeedLimit.
  - **Delta re-routing & divergence detection**: GetPathRejoinPoint.
  - **Vehicle height clearance & low-bridge avoidance**: SetConnectionMaxHeight, SetAreaMaxHeight, and FindPathWithHeightLimit.
  - **Path memory management & serialization**: SetPathAutoDestroy, SavePathToFile, LoadPathFromFile, ExportPathToString, and ImportPathFromString.
  - Full asynchronous support for **PawnPlus tasks** with PathType and RoutePreference.
  - Formal adoption of the **Mozilla Public License 2.0 (MPL-2.0)** and standalone build packages.

### 1.4.1 — 24/01/2023
- Gracefully handle missing server.cfg without throwing exceptions.
- Renamed INVALID_PATH_ID to INVALID_GPS_PATH_ID to prevent naming collisions with open.mp standard libraries.
- CMake build pipeline improvements and multiplatform CI integration.

### 1.4.0 — 21/04/2019
- Modern refactoring and enhanced thread-safety.
- Added option to disable automatic map node loading on server startup via gps_load_nodes.
- Fixed node and connection deletion edge cases.

### 1.3.0 — 12/03/2019
- Rewrote file parsing using optimized streams.
- Improved A* pathfinding heuristics and memory footprint.
- Added GetPathNodeIndex.

### 1.2.0 — 28/01/2019
- Fixed critical pathfinding algorithm issues, delivering up to 30x faster query execution.
- Major refactoring of internal path structures.

### 1.1.0 — 15/01/2019
- Added GetRandomMapNode and intersection detection.
- Introduced custom callback parameter passing for threaded pathfinding.

### 1.0.0 — 05/01/2019
- Initial release of samp-gps-plugin.
- Basic A* pathfinding using GPS.dat dataset.
- Threaded path calculation using std::thread.

</details>

---

## Credits & Acknowledgements

* [@Walter-Correa](https://github.com/Walter-Correa) - Current maintainer, 2D spatial hash grid optimization, open.mp native component integration, road snapping & curb alignment, turn directions & ETA calculations, dynamic roadblocks, layer isolation, and documentation.
* [@AmyrAhmady](https://github.com/AmyrAhmady) - Predecessor and maintainer of the [samp-gps-plugin](https://github.com/AmyrAhmady/samp-gps-plugin) fork that served as the base for this project.
* [@kristoisberg](https://github.com/kristoisberg) - Original creator and architect of `samp-gps-plugin`.
* [@FreddieCrew](https://github.com/FreddieCrew) - Map node and spatial algorithm contributions.
* Gamer_Z - Original creator of the RouteConnector plugin and pioneer of map node extraction for San Andreas.
* NaS - Author of the refined `GPS.dat` dataset.
* [@Southclaws](https://github.com/Southclaws), [@IllidanS4](https://github.com/IllidanS4), [@Hual](https://github.com/Hual) - Invaluable architectural insights, community tooling, and PawnPlus integration.
* [open.mp Team](https://github.com/openmultiplayer) - Creators of the open.mp multiplayer platform and Component SDK.

---

## License

This project is licensed under the Mozilla Public License 2.0 (MPL-2.0). See the [LICENSE](LICENSE) file for details.
