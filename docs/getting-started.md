# Getting Started with GPS Plugin Plus

This guide walks you through installing **GPS Plugin Plus**, understanding its architecture, configuring it for **SA-MP** or **open.mp**, seamlessly migrating legacy GPS implementations, and creating your very first pathfinding script in Pawn.

---

## 1. Prerequisites & Installation

GPS Plugin Plus consists of three core components:
1. **Server Binary:** `GPS.dll` (Windows) or `GPS.so` (Linux), compiled as a 32-bit native extension.
2. **Include Header:** `GPS.inc` providing strongly-typed Pawn tags, enumerations, and native bindings.
3. **Map Node Database:** `GPS.dat` (~7.3 MB binary file) containing the spatial graph of San Andreas roads, footpaths, waterways, and lane connections.

### Step 1: File Placement

Copy the downloaded files into your server root directory:

```
[Server Root]/
  ├── plugins/
  │     └── GPS.dll             (or GPS.so on Linux for SA-MP)
  ├── components/               (Optional: for open.mp native component mode)
  │     └── GPS.dll             (or GPS.so on Linux)
  ├── pawno/include/            (or qawno/include/)
  │     └── GPS.inc
  └── scriptfiles/
        └── GPS.dat             <-- MANDATORY: Pathfinding database
```

> [!IMPORTANT]
> The `GPS.dat` file **must** be placed directly inside the `scriptfiles/` directory. When the server launches, the plugin parses this binary database to build its in-memory spatial graph. If `GPS.dat` is missing or corrupted, the plugin will fail to initialize.

---

## 2. Server Configuration

### For SA-MP Servers (`server.cfg`)

Open your `server.cfg` file and append `GPS` to the `plugins` line:

* **Windows:**
  ```cfg
  plugins GPS.dll
  ```
* **Linux:**
  ```cfg
  plugins GPS.so
  ```

*(If other plugins are already loaded, separate them with spaces, e.g. `plugins crashdetect.dll PawnPlus.dll GPS.dll`)*.

---

### For open.mp Servers (`config.json`)

GPS Plugin Plus supports both open.mp's native Component architecture and the legacy Plugin wrapper.

#### Option A: Native open.mp Component (Recommended)
Place the compiled binary in `components/` and add it to the `"components"` array in `config.json`:
```json
{
  "components": [
    "GPS"
  ]
}
```

#### Option B: Legacy Plugin Mode
Place the compiled binary in `plugins/` and add it to the `"plugins"` array:
```json
{
  "plugins": [
    "GPS"
  ]
}
```

---

## 3. Script Inclusion

Add `#include <GPS>` to your gamemode or filterscript after your core server include:

### Standard Setup
```pawn
#include <a_samp>      // Or <open.mp>
#include <GPS>
```

### With PawnPlus Async Task Support (Recommended)
If your project utilizes [PawnPlus](https://github.com/IllidanS4/PawnPlus), include `<PawnPlus>` **before** `<GPS>`. This automatically enables modern `Task:FindPathAsync` and `Task:FindPathAsyncEx` wrappers:

```pawn
#include <a_samp>      // Or <open.mp>
#include <PawnPlus>    // Include PawnPlus FIRST
#include <GPS>         // GPS.inc will detect _PawnPlus_included
```

---

## 4. Drop-In Migration from Legacy GPS

GPS Plugin Plus was built to be a **100% backward-compatible drop-in replacement** for the original GPS plugin by Amyr Ahmady.

If your gamemode already uses the original GPS plugin:
1. Replace your old `GPS.dll` / `GPS.so` with the new **GPS Plugin Plus** binary.
2. Replace `pawno/include/GPS.inc` with the new `GPS.inc`.
3. Keep your existing `scriptfiles/GPS.dat`.
4. Recompile your gamemode. **Zero code modifications are required.**

### Legacy Compatibility Matrix

| Legacy GPS Symbol | GPS Plugin Plus Standard | Notes |
| :--- | :--- | :--- |
| `INVALID_MAP_NODE_ID` | `GPS_INVALID_MAP_NODE_ID` / `INVALID_MAP_NODE_ID` | Fully aliased (`MapNode:-1`) |
| `INVALID_PATH_ID` | `GPS_INVALID_PATH_ID` / `INVALID_GPS_PATH_ID` | Fully aliased (`Path:-1`) |
| `FindPath(src, dst, &path)` | `FindPath(src, dst, &path, ...)` | Default parameters maintain exact legacy signature |
| `FindPathThreaded(src, dst, cb, ...)` | `FindPathThreaded(src, dst, cb, ...)` | 100% identical signature & callback behavior |
| `GetPathSize(path, &size)` | `GetPathSize(path, &size)` | Identical |
| `GetPathLength(path, &len)` | `GetPathLength(path, &len)` | Identical |
| `GetPathNode(path, idx, &node)` | `GetPathNode(path, idx, &node)` | Identical |
| `DestroyPath(path)` | `DestroyPath(path)` | Identical |

---

## 5. Your First Pathfinding Scripts

### Example 1: Synchronous Path Calculation
Ideal for immediate spatial calculations where execution latency is negligible:

```pawn
#include <a_samp>
#include <GPS>

CMD:gpsdemo(playerid, params[])
{
    new Float:px, Float:py, Float:pz;
    GetPlayerPos(playerid, px, py, pz);

    // 1. Find closest road node to player
    new MapNode:start_node;
    GetClosestMapNodeToPoint(px, py, pz, start_node);

    // 2. Find closest road node to Los Santos Airport
    new MapNode:dest_node;
    GetClosestMapNodeToPoint(1958.0, -2181.0, 13.5, dest_node);

    // 3. Calculate path synchronously
    new Path:path_id;
    new err = FindPath(start_node, dest_node, path_id);

    if (err == GPS_ERROR_NONE && IsValidPath(path_id))
    {
        new node_count;
        new Float:total_length;
        GetPathSize(path_id, node_count);
        GetPathLength(path_id, total_length);

        new msg[128];
        format(msg, sizeof(msg), "[GPS] Route found! Nodes: %d | Total Distance: %.1f meters", node_count, total_length);
        SendClientMessage(playerid, 0x2ECC71AA, msg);

        // Remember to free memory when done!
        DestroyPath(path_id);
    }
    else
    {
        SendClientMessage(playerid, 0xE74C3CAA, "[GPS] Unable to calculate a route to the destination.");
    }
    return 1;
}
```

---

### Example 2: Multi-Threaded Path Calculation (Non-Blocking)
Recommended for long routes across San Andreas to ensure 0 server tick lag:

```pawn
#include <a_samp>
#include <GPS>

CMD:gotosf(playerid, params[])
{
    new Float:px, Float:py, Float:pz;
    GetPlayerPos(playerid, px, py, pz);

    new MapNode:start_node, MapNode:dest_node;
    GetClosestMapNodeToPoint(px, py, pz, start_node);
    GetClosestMapNodeToPoint(-1988.0, 158.0, 27.5, dest_node); // San Carlos Pier, SF

    SendClientMessage(playerid, 0x3498DBAA, "[GPS] Calculating high-speed highway route to San Fierro...");

    // Offload pathfinding calculation to native background worker thread
    FindPathThreadedEx(
        start_node, dest_node,
        PATH_TYPE_VEHICLE, ROUTE_PREF_HIGHWAYS,
        "OnRouteCalculated", "ii", _:start_node, playerid
    );
    return 1;
}

// Callback invoked in server main thread when calculation completes
forward OnRouteCalculated(Path:path, start_node, playerid);
public OnRouteCalculated(Path:path, start_node, playerid)
{
    if (!IsPlayerConnected(playerid))
    {
        DestroyPath(path);
        return 1;
    }

    if (!IsValidPath(path))
    {
        SendClientMessage(playerid, 0xE74C3CAA, "[GPS] No viable highway route found.");
        return 1;
    }

    new Float:distance, Float:eta_seconds;
    GetPathLength(path, distance);
    GetPathEstimatedTravelTime(path, 80.0, eta_seconds); // Estimated at 80 km/h

    new msg[144];
    format(msg, sizeof(msg), "[GPS] Route Ready! Distance: %.1f km | ETA: %.1f minutes", 
        distance / 1000.0, eta_seconds / 60.0);
    SendClientMessage(playerid, 0x2ECC71AA, msg);

    // Auto-destroy path from memory after 60 seconds of inactivity
    SetPathAutoDestroy(path, 60000);
    return 1;
}
```

---

### Example 3: Modern PawnPlus Async / Await
Write clean, linear asynchronous code with zero callback fragmentation:

```pawn
#include <a_samp>
#include <PawnPlus>
#include <GPS>

CMD:routeasync(playerid, params[])
{
    new Float:px, Float:py, Float:pz;
    GetPlayerPos(playerid, px, py, pz);

    new MapNode:src, MapNode:dst;
    GetClosestMapNodeToPoint(px, py, pz, src);
    GetClosestMapNodeToPoint(2232.0, 2452.0, 10.8, dst); // Las Venturas Strip

    SendClientMessage(playerid, 0xF1C40FAA, "[GPS] Computing route with PawnPlus task...");

    // Wait asynchronously for background worker thread without blocking tick
    task_fork()
    {
        new Path:path = Path:task_await(FindPathAsyncEx(src, dst, PATH_TYPE_VEHICLE, ROUTE_PREF_SHORTEST));

        if (IsValidPath(path))
        {
            new Float:len;
            GetPathLength(path, len);
            
            new str[128];
            format(str, sizeof(str), "[GPS] Async path ready! Distance: %.1f meters.", len);
            SendClientMessage(playerid, 0x2ECC71AA, str);

            DestroyPath(path);
        }
        else
        {
            SendClientMessage(playerid, 0xE74C3CAA, "[GPS] Failed to calculate route.");
        }
    }
    return 1;
}
```

---

## Next Steps

Explore the rest of the documentation suite to leverage the full power of GPS Plugin Plus:
* **[Pathfinding & Routing](pathfinding-and-routing.md):** Deep-dive into traffic multipliers, roadblock detours, safe-routing, eco-routing, tolls, and multi-stop TSP optimization.
* **[Navigation & Geometry](navigation-and-geometry.md):** Implement real-time turn-by-turn HUD guidance, roundabout detection, and Catmull-Rom spline path smoothing.
* **[AI & Traffic Simulation](ai-and-traffic-simulation.md):** Build autonomous NPC driving bots using Pure Pursuit steering and police pursuit intercept AI.
* **[Complete API Reference](api-reference.md):** Consult the full signature directory of all 105 natives.
