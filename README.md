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
  Copy the binary to the `components/` directory and update `config.json`:
  ```json
  {
    "components": [
      "GPS"
    ]
  }
  ```

* **Option B: Legacy Plugin**
  Copy the binary to the `plugins/` directory and update `config.json`:
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
native Task:FindPathAsync(MapNode:source, MapNode:target);
```
Asynchronous pathfinding task for [PawnPlus](https://github.com/IllidanS4/PawnPlus) with `task_await`.

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
