# GPS Plugin Plus - Documentation Hub

Welcome to the technical documentation for **GPS Plugin Plus** (SA-MP & open.mp). This hub provides comprehensive architectural deep-dives, developer tutorials, algorithmic explanations, and an exhaustive API reference covering every feature of the pathfinding, navigation, spatial analysis, and AI simulation engine.

---

## Documentation Sections

| Section | Description |
| :--- | :--- |
| **[📖 Getting Started](getting-started.md)** | Requirements, installation for SA-MP & open.mp, `GPS.dat` setup, drop-in migration from legacy GPS, and your first path script. |
| **[🗺️ Pathfinding & Routing](pathfinding-and-routing.md)** | Synchronous vs. threaded execution, PawnPlus async tasks, multi-modal routing (vehicles, pedestrians, boats), routing preferences, roadblocks, traffic congestion, safe-routing, eco-routing, tolls, height clearances, and vehicle category constraints. |
| **[🧭 Navigation & Geometry](navigation-and-geometry.md)** | Real-time GPS progress tracking, ETA and distance metrics, turn-by-turn guidance (`TurnDirection`), roundabout detection with exit counting, Catmull-Rom spline path smoothing, Douglas-Peucker route decimation, delta re-routing rejoin points, and lateral offset calculation. |
| **[🤖 AI & Traffic Simulation](ai-and-traffic-simulation.md)** | Pure Pursuit autonomous bot steering controller, high-speed police interception vectors, tactical fugitive anti-dead-end evasion, Waze-style speed radars, nearest service lookup by real road distance, reachability isochrones, tunnel detection, and wrong-way traffic enforcement. |
| **[📍 Map Nodes & Spatial Analytics](map-nodes-and-spatial.md)** | Graph node architecture, dynamic node & connection lifecycle, multi-lane road geometry, spatial proximity queries, curb-side vehicle snapping, San Andreas neighborhood zones, path text serialization, and automatic memory TTL cleanup. |
| **[📚 Complete API Reference](api-reference.md)** | Exhaustive technical dictionary covering all 105 native functions, callbacks, enums, error codes, and PawnPlus asynchronous stocks. |

---

## Quick Navigation Matrix

Find immediate guidance based on your development goal:

* **I want to install the plugin or open.mp component:**
  * Read [Installation & Setup](getting-started.md#1-prerequisites--installation).
  * Configure `server.cfg` (SA-MP) or `config.json` (open.mp).
* **I am migrating an existing gamemode from Amyr Ahmady's GPS Plugin:**
  * Read [Drop-In Migration from Legacy GPS](getting-started.md#4-drop-in-migration-from-legacy-gps).
  * Drop-in compatible: no existing code changes required (see the migration notes).
* **I want to calculate routes asynchronously without lagging the server tick rate:**
  * Read [Threaded & Asynchronous Pathfinding](pathfinding-and-routing.md#1-synchronous-vs-asynchronous-pathfinding).
  * Use `FindPathThreaded` / `FindPathThreadedEx` with a callback or `task_await(FindPathAsync(source, target))` with PawnPlus tasks.
* **I want to build a real-time turn-by-turn navigation HUD (Waze / Google Maps style):**
  * Read [Real-Time Navigation Tracking & Maneuvers](navigation-and-geometry.md#1-real-time-path-tracking--progress).
  * Use `GetPathProgress`, `GetPathRemainingInfo`, and `GetPathNextManeuver`.
* **I want autonomous NPC bot vehicles to drive smoothly along roads:**
  * Read [Autonomous Pure Pursuit Steering](ai-and-traffic-simulation.md#1-autonomous-npc-driving-pure-pursuit-controller).
  * Use `GetPurePursuitSteering` for dynamic steering angle ratios and curvature-adaptive cornering speeds.
  * Use `GetSmoothPathPoints` for Catmull-Rom spline curve densification.
* **I want police cruisers to calculate intercept vectors and cut off a fleeing suspect:**
  * Read [High-Speed Police Interception AI](ai-and-traffic-simulation.md#2-high-speed-police-pursuit-interception).
  * Use `FindInterceptPath` with pursuer and suspect current speeds.
* **I want fleeing suspects / bot fugitives to avoid dead-ends, alleys, and police chokepoints:**
  * Read [Tactical Fugitive Evasion Routing](ai-and-traffic-simulation.md#3-tactical-fugitive-evasion-routing).
  * Use `FindEvasivePath` and `IsMapNodeDeadEnd`.
* **I want to find the nearest Hospital or Repair Station by real road driving distance:**
  * Read [Nearest Service by Real Road Distance](ai-and-traffic-simulation.md#5-nearest-service-by-real-road-distance).
  * Use `FindNearestPointByRoad` to avoid Euclidean line-of-sight errors through mountains and rivers.
* **I want to park or spawn a broken-down vehicle precisely aligned with the roadside curb:**
  * Read [Curb-Side Snapping & Road Alignment](map-nodes-and-spatial.md#4-road-position--curb-side-alignment).
  * Use `GetRoadPositionInfo(x, y, z, out_x, out_y, out_z, angle, road_width, ROAD_POS_CURB)`.
* **I want to simulate dynamic traffic jams, road closures, or toll booths:**
  * Read [Dynamic Roadblocks & Congestion](pathfinding-and-routing.md#5-dynamic-roadblocks--detours) and [Toll Roads & Bridge Passes](pathfinding-and-routing.md#9-toll-roads--bridge-passes).
  * Use `SetConnectionTrafficMultiplier`, `SetConnectionBlocked`, and `SetConnectionToll`.

---

## Compatibility & System Requirements

* **Game Platforms:**
  * **SA-MP:** `0.3.7-R1` through `0.3.7-R5` (x86 32-bit).
  * **open.mp:** All active releases via native Component (`GPS-Component-Plus`) or Legacy Plugin (`GPS-Plugin-Plus`).
* **Operating Systems:**
  * **Windows:** Windows 7 / 8 / 10 / 11, Windows Server 2008 R2+ (32-bit x86). Zero external dependencies: statically linked runtime (`/MT`), requiring only standard Windows system libraries (`KERNEL32.dll`). No MSVC Redistributable required.
  * **Linux:** Debian 10+, Ubuntu 20.04+, RHEL/Alma/Rocky 8+ (32-bit x86, glibc ≥ 2.28).
* **Compiler Support:**
  * Community Pawn Compiler (Pawn 3.10.x by Southclaws/Zeex) - *Recommended*.
  * Official Pawn 3.2.3664 (legacy SA-MP default).
* **Optional Dependencies:**
  * **PawnPlus** (by IllidanS4): Enables modern task-based asynchronous programming via `task_await(FindPathAsync(...))` (or `await` syntax with `#define PP_SYNTAX_AWAIT`).
