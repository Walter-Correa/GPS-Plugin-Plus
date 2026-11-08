# GPS Plugin Plus

[![GitHub release](https://img.shields.io/github/v/release/Walter-Correa/GPS-Plugin-Plus?include_prereleases&color=blue)](https://github.com/Walter-Correa/GPS-Plugin-Plus/releases)
[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
[![Platform](https://img.shields.io/badge/platform-SA--MP%20%7C%20open.mp-orange.svg)](https://open.mp)

**GPS Plugin Plus** is a high-performance, multi-threaded pathfinding, real-time navigation, autonomous NPC vehicle steering, and spatial analysis suite engineered specifically for **San Andreas Multiplayer (SA-MP)** and **open.mp**.

It empowers Pawn developers to build intelligent, state-of-the-art navigation systems: **Threaded A\* Route Calculations**, **Real-Time Turn-by-Turn Maneuvers**, **Catmull-Rom Spline Curve Smoothing**, **Douglas-Peucker Route Decimation**, **Autonomous Pure Pursuit Bot Steering**, **High-Speed Police Intercept Vectors**, **Dynamic Traffic Jams & Roadblocks**, **Waze-Style Speed Radars**, and **Nearest Service Queries by Real Road Distance**—with full asynchronous worker thread support (zero server tick lag on threaded queries) and drop-in compatibility.

> [!NOTE]
> This project is conceptually inspired by the original [samp-gps-plugin](https://github.com/kristoisberg/samp-gps-plugin) by [@kristoisberg](https://github.com/kristoisberg) and its fork by [@AmyrAhmady](https://github.com/AmyrAhmady). It has been completely re-architected and modernized from the ground up, introducing dual-stack architecture (native open.mp Component + SA-MP Plugin), a native multi-threaded C++ worker pool, PawnPlus task async coroutines, multi-modal routing (vehicles, pedestrians, boats), and 105+ high-performance spatial natives.

---

## Comparison: Legacy GPS Plugin vs. GPS Plugin Plus

| Feature / Capability | Legacy GPS Plugin (Amyr Ahmady) | GPS Plugin Plus |
| :--- | :--- | :--- |
| **Platform Architecture** | SA-MP plugin wrapper only | ✅ Native open.mp Component (`GPS-Component-Plus`) + SA-MP Plugin (`GPS-Plugin-Plus`) |
| **Asynchronous Engine** | Tick-blocking synchronous execution or basic thread | ✅ Multi-threaded C++ worker pool + PawnPlus task promises (`task_await(FindPathAsync)` / `await`) |
| **Transport Modalities** | Vehicles only | ✅ Unified multi-modal routing (`PathType`): Vehicles, Pedestrians, and Boats |
| **Route Preferences** | Distance only | ✅ 8 heuristics: Shortest, Highway, Avoid Highway, Safest, Eco, Evasive, Toll-Free, Paved Only |
| **Turn-by-Turn Navigation** | ❌ None | ✅ Built-in tracker (`GetPathProgress`, `GetPathNextManeuver`) with 7 turn directions & distance |
| **Autonomous Bot Steering** | ❌ None | ✅ Built-in **Pure Pursuit** controller calculating lookahead steering ratios ($-1.0$ to $+1.0$) and cornering speed |
| **Dynamic Traffic & Roadblocks** | Static unmodifiable node graph | ✅ Real-time edge traffic multipliers (`SetConnectionTrafficMultiplier`) and dynamic roadblocks |
| **Route Geometry & Smoothing** | Piecewise jagged polygon lines | ✅ **Catmull-Rom cubic spline** densification (`GetSmoothPathPoints`) for smooth NPC driving |
| **Route Decimation** | ❌ None | ✅ **Douglas-Peucker** route decimation (`GetSimplifiedPathNodes`) reducing node bandwidth by up to 90% |
| **Roundabout Analysis** | ❌ None | ✅ Automatic rotary detection, radius calculation, and exit counting ("Take the 2nd exit") |
| **Law Enforcement AI** | ❌ None | ✅ Dynamic police intercept vectors (`FindInterceptPath`) cutting off suspects ahead of time |
| **Tactical Evasion AI** | ❌ None | ✅ Fugitive anti-dead-end routing (`FindEvasivePath`, `IsMapNodeDeadEnd`) avoiding traps |
| **Spatial Proximity Queries** | Linear distance checks | ✅ Anti-Euclidean nearest service by **real road distance** (`FindNearestPointByRoad`) |
| **Roadside Snapping & Alignment** | Snaps to road center | ✅ **Curb-side alignment** (`GetRoadPositionInfo`) for parked vehicles and mechanic missions |
| **Speed Radars & Hazard Alerts** | ❌ None | ✅ Waze-style road radar detector (`RegisterRoadRadar`, `GetNextPathRadar`) |
| **Vehicle Physical Limits** | ❌ None | ✅ Bridge clearance limits (`SetConnectionMaxHeight`) & vehicle categories (`VehicleCategory`) |
| **Memory & Serialization** | Manual deletion only | ✅ Automatic TTL garbage collection (`SetPathAutoDestroy`) and plain-text path file/string import/export |

---

## Documentation Hub

Comprehensive, modular guides and technical references are available in the [`docs/`](docs/) directory:

| Guide | Description |
| :--- | :--- |
| **[📖 Getting Started](docs/getting-started.md)** | Step-by-step setup, installation for SA-MP & open.mp, `GPS.dat` configuration, and drop-in migration. |
| **[🗺️ Pathfinding & Routing](docs/pathfinding-and-routing.md)** | Synchronous vs. threaded execution, PawnPlus async tasks, multi-modal routing, roadblocks, and traffic congestion. |
| **[🧭 Navigation & Geometry](docs/navigation-and-geometry.md)** | Real-time GPS tracking, turn-by-turn guidance, roundabout detection, Catmull-Rom splines, and route decimation. |
| **[🤖 AI & Traffic Simulation](docs/ai-and-traffic-simulation.md)** | Pure Pursuit bot steering, police intercept vectors, fugitive evasion, speed radars, and real road-distance queries. |
| **[📍 Map Nodes & Spatial Analytics](docs/map-nodes-and-spatial.md)** | Map node graph lifecycle, multi-lane roads, curb-side alignment, neighborhood zones, and path serialization. |
| **[📚 Complete API Reference](docs/api-reference.md)** | Exhaustive technical dictionary covering all 105 native functions, callbacks, enums, and error codes. |

---

## Quick Start & Installation

### 1. File Placement

Place the binaries, database, and include header into your server development environment:

```
[Server Root]/
  ├── plugins/
  │     └── GPS.dll             (or GPS.so on Linux)
  ├── components/               (Optional: open.mp native component)
  │     └── GPS.dll             (or GPS.so on Linux)
  ├── pawno/include/            (or qawno/include/)
  │     └── GPS.inc
  └── scriptfiles/
        └── GPS.dat             <-- MANDATORY: Node database
```

### 2. Server Configuration

Install **either** the SA-MP plugin in `plugins/` **or** the open.mp component in `components/` (do not install both simultaneously).

* **SA-MP (`server.cfg`):**
  Windows:
  ```cfg
  plugins GPS.dll
  ```
  Linux:
  ```cfg
  plugins GPS.so
  ```

* **open.mp (`config.json`):**
  * **Option A: Native open.mp Component (Recommended):**
    Place `GPS.dll` (or `GPS.so`) into your server's `components/` directory. Open.mp automatically detects and loads all components in `components/`; no entry in `config.json` is required.
  * **Option B: Legacy Plugin Mode:**
    Place `GPS.dll` (or `GPS.so`) into `plugins/` and add it to `config.json`:
    ```json
    {
      "pawn": {
        "legacy_plugins": ["GPS"]
      }
    }
    ```

### 3. Include in Your Script

```pawn
#include <open.mp> // Or <a_samp>
#include <GPS>
```

### 4. Basic Example

Compute a high-speed highway route and track navigation in real time:

```pawn
new Path:g_PlayerRoute[MAX_PLAYERS] = { INVALID_GPS_PATH_ID, ... };

CMD:navigate(playerid, params[])
{
    new Float:px, Float:py, Float:pz;
    GetPlayerPos(playerid, px, py, pz);

    new MapNode:start_node, MapNode:dest_node;
    GetClosestMapNodeToPoint(px, py, pz, start_node);
    GetClosestMapNodeToPoint(2232.0, 2452.0, 10.8, dest_node); // Las Venturas Strip

    // Compute route asynchronously on native worker thread
    FindPathThreadedEx(
        start_node, dest_node,
        PATH_TYPE_VEHICLE, ROUTE_PREF_HIGHWAYS,
        "OnPlayerRouteReady", "i", playerid
    );
    return 1;
}

forward OnPlayerRouteReady(Path:path, playerid);
public OnPlayerRouteReady(Path:path, playerid)
{
    if (!IsValidPath(path)) return SendClientMessage(playerid, 0xE74C3CAA, "[GPS] No route found.");

    g_PlayerRoute[playerid] = path;

    new Float:dist, Float:eta;
    GetPathLength(path, dist);
    GetPathEstimatedTravelTime(path, 80.0, eta);

    new msg[128];
    format(msg, sizeof(msg), "[GPS] Route calculated! Distance: %.1f km | ETA: %.1f minutes", dist / 1000.0, eta / 60.0);
    SendClientMessage(playerid, 0x2ECC71AA, msg);

    // Auto-destroy path from memory after 5 minutes (absolute TTL)
    SetPathAutoDestroy(path, 300000);
    return 1;
}
```

> [!TIP]
> Need to migrate an existing gamemode from Amyr Ahmady's GPS Plugin? Read the **[Migration Guide in docs/getting-started.md](docs/getting-started.md#4-drop-in-migration-from-legacy-gps)** for instant 1:1 drop-in compatibility.

---

## Complete API Reference

All 105 native functions, callbacks, style enumerations, and constants are exhaustively documented with signature parameters, return values, and code snippets in **[`docs/api-reference.md`](docs/api-reference.md)**:

* **[Map Node Management](docs/api-reference.md#1-map-node-management):** `CreateMapNode`, `DestroyMapNode`, `IsValidMapNode`, `GetMapNodePos`, `GetRandomMapNode`...
* **[Road Connections & Lanes](docs/api-reference.md#2-road-connections--lanes):** `CreateConnection`, `GetConnectionLaneInfo`, `GetConnectionLanePos`, `IsConnectionOneWay`...
* **[Spatial Queries & Snapping](docs/api-reference.md#3-spatial-queries--geometric-snapping):** `GetClosestMapNodeToPoint`, `GetClosestPointOnRoad`, `GetRoadPositionInfo`...
* **[Pathfinding & Routing](docs/api-reference.md#4-pathfinding--routing):** `FindPath`, `FindPathThreadedEx`, `FindMultiStopPath`, `FindPathForVehicleCat`...
* **[Dynamic Traffic & Roadblocks](docs/api-reference.md#5-dynamic-traffic--roadblocks):** `SetMapNodeBlocked`, `SetConnectionBlocked`, `SetConnectionTrafficMultiplier`...
* **[Turn-by-Turn Navigation & Tracking](docs/api-reference.md#6-turn-by-turn-navigation--tracking):** `GetPathProgress`, `GetPathRemainingInfo`, `GetPathNextManeuver`...
* **[Route Geometry & Splines](docs/api-reference.md#7-route-geometry--splines):** `GetSimplifiedPathNodes`, `GetSmoothPathPoints`, `GetPathOffsetPosition`...
* **[Autonomous AI, Interception & Evasion](docs/api-reference.md#8-autonomous-ai-interception--evasion):** `GetPurePursuitSteering`, `FindInterceptPath`, `FindEvasivePath`...
* **[Radars, Isochrones & Nearest Service](docs/api-reference.md#9-radars-isochrones--nearest-service):** `FindNearestPointByRoad`, `RegisterRoadRadar`, `GetReachableMapNodes`...
* **[Tunnels, Danger & Elevation](docs/api-reference.md#10-tunnels-danger--elevation):** `RegisterTunnelArea`, `FindSafePath`, `FindEcoPath`, `GetPathElevationStats`...
* **[Road Characteristics & Speed Limits](docs/api-reference.md#11-road-characteristics--speed-limits):** `GetConnectionRoadClass`, `SetConnectionSpeedLimit`, `IsConnectionPaved`...
* **[Height Clearances & Vehicle Categories](docs/api-reference.md#12-height-clearances--vehicle-categories):** `SetConnectionMaxHeight`, `FindPathWithHeightLimit`...
* **[Path Serialization & Persistence](docs/api-reference.md#13-path-serialization--persistence):** `SetPathAutoDestroy`, `SavePathToFile`, `ExportPathToString`...
* **[San Andreas Geographic Zones](docs/api-reference.md#14-san-andreas-geographic-zones):** `GetPointZoneName`, `GetMapNodeZoneName`, `GetPathZonesSummary`...
* **[PawnPlus Asynchronous Tasks](docs/api-reference.md#15-pawnplus-asynchronous-tasks):** `FindPathAsync`, `FindPathAsyncEx`...
* **[Constants, Enums & Error Codes](docs/api-reference.md#16-constants-enums--error-codes):** `GPS_ERROR`, `PathType`, `RoutePreference`, `TurnDirection`...

---

## Building from Source

### Prerequisites
* CMake 3.15+
* Visual Studio 2022 (Windows) with Desktop C++ workload
* GCC / G++ 9+ with 32-bit multilib (`gcc-multilib g++-multilib`) (Linux)
* Git submodules: clone with `--recursive` or run `git submodule update --init --recursive` to fetch `open.mp-sdk` (required for building the open.mp component).

### Compiling on Windows (1-Click Script)
Run the provided automated build script:
```cmd
build_msvc.bat
```
*(Automatically configures CMake for Win32, compiles Release binaries, stages packages, and builds `dist/GPS-Plugin-Plus-Windows.zip`).*

### Compiling on Linux
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS="-m32" -DCMAKE_CXX_FLAGS="-m32"
make -j$(nproc)
```

---

## Version history

<details>
<summary>Show the version history</summary>

### 2.0.0 — 08/11/2026
- Complete architectural evolution to **GPS Plugin Plus**:
  - Native **open.mp Component SDK** integration with dual-target compilation (legacy SA-MP plugin and modern open.mp component).
  - High-performance **2D spatial hash grid** indexing ($100\text{m} \times 100\text{m}$ cells), reducing spatial node/connection queries from $\mathcal{O}(N)$ linear scans to $\mathcal{O}(1)$ constant-time neighborhood lookups (sub-millisecond spatial lookups and microsecond-level closest point queries).
  - Advanced **orthogonal road snapping & curb positioning**: `GetRoadPositionInfo`, `GetClosestPointOnRoad`, and `IsPointAgainstTraffic`.
  - Comprehensive **turn-by-turn navigation engine**: `GetPathTurnDirection`, `GetPathNextManeuver`, `GetPathProgress`, and `GetPathRemainingInfo`.
  - Dynamic **roadblocks & layer isolation**: `SetMapNodeBlocked`, `SetConnectionBlocked`, and traffic rerouting.
  - Multi-stop **waypoint routing & highway preference**: `FindMultiStopPath` and `RoutePreference` weighting (`ROUTE_PREF_SHORTEST`, `ROUTE_PREF_HIGHWAYS`, `ROUTE_PREF_AVOID_HIGHWAYS`, `ROUTE_PREF_SAFEST`, `ROUTE_PREF_ECO`, `ROUTE_PREF_EVASIVE`, `ROUTE_PREF_AVOID_TOLLS`, `ROUTE_PREF_PAVED_ONLY`).
  - Route decimation and waypoint optimization via **iterative Ramer-Douglas-Peucker algorithm** (`GetSimplifiedPathNodes`), using an explicit stack to eliminate recursion overhead and reduce checkpoint bandwidth by up to 90%.
  - Real-time **traffic routing & vehicle category constraints**: `SetConnectionTrafficMultiplier`, `FindPathForVehicleCat`.
  - High-speed **pursuit interception & Pure Pursuit controller**: `FindInterceptPath` and `GetPurePursuitSteering` with SA-MP standard angle convention.
  - Reachability **isochrones & geographic zone dictionary**: `GetReachableMapNodes`, `GetPointZoneName`, and `GetPathZonesSummary`.
  - **Eco-routing & slope elevation climb analysis**: `FindEcoPath` and `GetPathElevationStats`.
  - **Subterranean & tunnel detection**: `RegisterTunnelArea`, `IsPointInTunnel`, `IsMapNodeUnderground`, and `GetPathTunnelSegments`.
  - Non-Euclidean **nearest service by real road distance**: `FindNearestPointByRoad` with optional path handle instantiation to prevent memory leaks when paths are unneeded.
  - Waze-style **speed cameras & road hazard alerts**: `RegisterRoadRadar`, `GetNextPathRadar`.
  - Tactical **convoy formations & escort offsets**: `GetPathOffsetPosition` with normalized bearing calculations.
  - Tactical **pursuit evasion & anti-dead-end routing**: `FindEvasivePath` and `IsMapNodeDeadEnd`.
  - **Toll roads & bridge restrictions**: `SetConnectionToll`, `GetConnectionToll`, and `FindPathAvoidTolls`.
  - **Roundabout & rotary exit detection**: `GetPathRoundaboutInfo`.
  - **Catmull-Rom cubic spline interpolation**: `GetSmoothPathPoints` for smooth NPC driving and camera rails.
  - **Pavement classification**: `IsConnectionPaved`, `SetConnectionPaved`, and `FindPathPavedOnly`.
  - **Road speed classification**: `GetConnectionRoadClass`, `SetConnectionSpeedLimit`, and `GetConnectionSpeedLimit`.
  - **Delta re-routing & divergence detection**: `GetPathRejoinPoint`.
  - **Vehicle height clearance & low-bridge avoidance**: `SetConnectionMaxHeight`, `SetAreaMaxHeight`, and `FindPathWithHeightLimit`.
  - **Path memory management & serialization**: `SetPathAutoDestroy`, `SavePathToFile`, `LoadPathFromFile`, `ExportPathToString`, and `ImportPathFromString`.
  - **Concurrency & Stability Hardening**:
    - Dedicated worker thread pool with bounded concurrency (`gps_worker_threads`, clamped between 1 and 16) and graceful shutdown synchronization, preventing thread exhaustion.
    - Strict RAII reader-writer synchronization (`std::shared_mutex`), preventing data races between concurrent pathfinders and main-thread map mutations.
    - Zero-copy internal vector references across high-frequency natives (`GetMapNodeConnection`, `GetPathNode`, `GetPathLaneNodePos`, `GetPathTurnDirection`, `GetPathZonesSummary`).
    - String output bounds validation and memory-safe buffer copying (`PawnUtils::GetArray`) in `GetPointZoneName`, `GetMapNodeZoneName`, and `ExportPathToString`.
    - Initialized configuration reader fallback values preventing undefined behavior.
    - Full exception safety boundary wrapping all C native entry points and AMX callbacks.
    - AMX script-level path ownership: paths are tracked per AMX instance and automatically reclaimed on gamemode or filterscript unload.
    - Admissible A* heuristic scaling ($\frac{h}{\max v}$) guaranteeing mathematically optimal shortest routes across all vehicle and preference weightings.
    - Strict input bounds validation and path traversal sanitization (`scriptfiles/` sandbox).
    - Fully static C/C++ runtime (`/MT`) for MSVC (zero external runtime dependencies) and static libgcc/libstdc++ targeting GLIBC 2.28+ for Linux.
  - Full asynchronous support for **PawnPlus tasks** with `PathType` and `RoutePreference`.
  - Comprehensive 6-guide documentation suite in `docs/`.
  - Formal adoption of the **Mozilla Public License 2.0 (MPL-2.0)** and standalone build packages.

### 1.4.1 — 24/01/2023
- Gracefully handle missing `server.cfg` without throwing exceptions.
- Renamed `INVALID_PATH_ID` to `INVALID_GPS_PATH_ID` to prevent naming collisions with open.mp standard libraries.
- CMake build pipeline improvements and multiplatform CI integration.

### 1.4.0 — 21/04/2019
- Modern refactoring and enhanced thread-safety.
- Added option to disable automatic map node loading on server startup via `gps_load_nodes`.
- Fixed node and connection deletion edge cases.

### 1.3.0 — 12/03/2019
- Rewrote file parsing using optimized streams.
- Improved A* pathfinding heuristics and memory footprint.
- Added `GetPathNodeIndex`.

### 1.2.0 — 28/01/2019
- Fixed critical pathfinding algorithm issues, delivering up to 30x faster query execution.
- Major refactoring of internal path structures.

### 1.1.0 — 15/01/2019
- Added `GetRandomMapNode` and intersection detection.
- Introduced custom callback parameter passing for threaded pathfinding.

### 1.0.0 — 05/01/2019
- Initial release of samp-gps-plugin.
- Basic A* pathfinding using `GPS.dat` dataset.
- Threaded path calculation using `std::thread`.

</details>

---

## Author & Credits

* [@Walter-Correa](https://github.com/Walter-Correa) - Architecture and implementation of **GPS Plugin Plus**, with the assistance of **Artificial Intelligence**.
* [@AmyrAhmady](https://github.com/AmyrAhmady) - Predecessor and maintainer of early forks of `samp-gps-plugin`.
* [@kristoisberg](https://github.com/kristoisberg) - Original creator and architect of `samp-gps-plugin`.
* [@FreddieCrew](https://github.com/FreddieCrew) - Map node and spatial algorithm contributions.
* [@Southclaws](https://github.com/Southclaws), [@IllidanS4](https://github.com/IllidanS4), and [@Hual](https://github.com/Hual) - Invaluable architectural insights, community tooling, and PawnPlus integration.

---

## License

This project is licensed under the **Mozilla Public License 2.0 (MPL-2.0)**. See the [LICENSE](LICENSE) file for complete legal terms.
