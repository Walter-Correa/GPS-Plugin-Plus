# Navigation & Geometry Suite

The **Navigation & Geometry Suite** provides real-time GPS tracking, turn-by-turn maneuver detection, roundabout analysis, curve smoothing, and route simplification algorithms. These routines allow developers to build professional, dynamic in-car GPS navigators, Google Maps/Waze-style HUDs, and smooth NPC path traversal.

---

## 1. Real-Time Path Tracking & Progress

As a player drives, their vehicle position can be dynamically mapped onto an active `Path` to measure progress, distance remaining, and off-route deviations:

```pawn
native GetPathProgress(
    Path:pathid, 
    Float:x, Float:y, Float:z, 
    &current_step, 
    &Float:distance_to_path, 
    &bool:off_route, 
    Float:max_distance_threshold = 25.0
);

native GetPathRemainingInfo(
    Path:pathid, 
    current_step, 
    &Float:remaining_distance, 
    &Float:remaining_seconds, 
    Float:speed_kmh = 60.0
);

native GetPathEstimatedTravelTime(
    Path:pathid, 
    Float:average_speed_kmh, 
    &Float:seconds
);
```

### Live Navigation Tracking Loop

```pawn
// Called every 500ms via player timer
UpdatePlayerNavigationHUD(playerid, Path:route)
{
    new Float:vx, Float:vy, Float:vz;
    GetPlayerPos(playerid, vx, vy, vz);

    new current_step, Float:dist_from_road, bool:is_off_route;
    GetPathProgress(route, vx, vy, vz, current_step, dist_from_road, is_off_route, 30.0);

    if (is_off_route)
    {
        // Player took a wrong turn; trigger automatic recalculation
        PlayerPlaySound(playerid, 1056, 0.0, 0.0, 0.0);
        SendClientMessage(playerid, 0xE74C3CAA, "[GPS] Recalculating route...");
        RecalculatePlayerRoute(playerid);
        return;
    }

    // Query distance and ETA to final destination
    new Float:rem_distance, Float:rem_seconds;
    GetPathRemainingInfo(route, current_step, rem_distance, rem_seconds, 70.0);

    new hud_text[96];
    format(hud_text, sizeof(hud_text), "~y~ETA: ~w~%.0fm %.0fs ~y~| Dist: ~w~%.1f km", 
        rem_seconds / 60.0, floatmod(rem_seconds, 60.0), rem_distance / 1000.0);
    PlayerTextDrawSetString(playerid, g_NavETADraw[playerid], hud_text);
}
```

---

## 2. Turn-by-Turn Guidance & Maneuvers

Detect upcoming turns, highway merges, sharp corners, and U-turns ahead of the vehicle:

```pawn
enum TurnDirection
{
    TURN_NONE        = 0, // No turn / Dead-end
    TURN_STRAIGHT    = 1, // Continue straight ahead
    TURN_RIGHT       = 2, // Standard right turn (45° to 120°)
    TURN_LEFT        = 3, // Standard left turn (45° to 120°)
    TURN_SHARP_RIGHT = 4, // Hairpin / Sharp right (> 120°)
    TURN_SHARP_LEFT  = 5, // Hairpin / Sharp left (> 120°)
    TURN_UTURN       = 6  // 180° turnaround maneuver
}

native GetPathTurnDirection(Path:pathid, index, &TurnDirection:turn, &Float:turn_angle);

native GetPathNextManeuver(
    Path:pathid, 
    Float:x, Float:y, Float:z, 
    &TurnDirection:turn, 
    &Float:distance_to_turn
);
```

### Visual Maneuver Display Example

```pawn
new TurnDirection:next_turn, Float:dist_to_turn;
GetPathNextManeuver(route, vx, vy, vz, next_turn, dist_to_turn);

switch (next_turn)
{
    case TURN_RIGHT:
        format(str, sizeof(str), "In %.0fm, Turn Right", dist_to_turn);
    case TURN_LEFT:
        format(str, sizeof(str), "In %.0fm, Turn Left", dist_to_turn);
    case TURN_SHARP_RIGHT:
        format(str, sizeof(str), "In %.0fm, Sharp Right", dist_to_turn);
    case TURN_SHARP_LEFT:
        format(str, sizeof(str), "In %.0fm, Sharp Left", dist_to_turn);
    case TURN_UTURN:
        format(str, sizeof(str), "In %.0fm, Make a U-Turn", dist_to_turn);
    default:
        format(str, sizeof(str), "Continue straight for %.1f km", dist_to_turn / 1000.0);
}
```

---

## 3. Roundabout Detection & Exit Counting

GPS Plugin Plus automatically recognizes circular rotary junctions and computes the exact exit branch to take:

```pawn
native GetPathRoundaboutInfo(
    Path:pathid, 
    index, 
    &bool:is_roundabout, 
    &exit_number, 
    &Float:roundabout_radius
);
```

```pawn
new bool:is_rotary, exit_idx, Float:radius;
GetPathRoundaboutInfo(path, step_index, is_rotary, exit_idx, radius);

if (is_rotary)
{
    new alert[128];
    format(alert, sizeof(alert), "[GPS] At the roundabout (Radius: %.0fm), take the %d%s exit.", 
        radius, exit_idx, (exit_idx == 1 ? "st" : (exit_idx == 2 ? "nd" : "rd")));
    SendClientMessage(playerid, 0x3498DBAA, alert);
}
```

---

## 4. Route Simplification (Douglas-Peucker Decimation)

High-detail paths generated across highways may contain hundreds of redundant collinear nodes. The **Douglas-Peucker algorithm** decimates the node count while strictly preserving geometry within a distance tolerance ($\epsilon$):

```pawn
native GetSimplifiedPathNodes(
    Path:pathid, 
    Float:tolerance_meters, 
    MapNode:output_nodes[], 
    max_nodes = sizeof(output_nodes), 
    &nodes_count
);
```

### Practical Benefits:
* **Network Bandwidth Reduction:** Reduce a 600-node route down to 25 critical turning points for sending to client scripts or RPCs.
* **Race Track Checkpoints:** Place checkpoints only at meaningful curves rather than every 5 meters.

```pawn
new MapNode:key_nodes[64], count;
// Decimate path with a 6.0 meter tolerance threshold
GetSimplifiedPathNodes(long_route, 6.0, key_nodes, sizeof(key_nodes), count);
printf("Path simplified from raw geometry into %d key decision nodes.", count);
```

---

## 5. Catmull-Rom Spline Densification (Curve Smoothing)

Raw road nodes form piecewise linear line segments. When steering autonomous NPC vehicles (or animating cinematic cameras), piecewise lines cause jerky, robotic turns. 

`GetSmoothPathPoints` passes the path through a cubic **Catmull-Rom spline interpolator**, generating smooth, curvature-continuous coordinates:

```pawn
native GetSmoothPathPoints(
    Path:pathid, 
    Float:spacing_meters, 
    Float:out_x[], Float:out_y[], Float:out_z[], 
    max_points = sizeof(out_x), 
    &total_points = 0
);
```

```pawn
new Float:spline_x[500], Float:spline_y[500], Float:spline_z[500];
new total_pts;

// Generate smooth coordinate points every 2.5 meters along the entire path
GetSmoothPathPoints(path, 2.5, spline_x, spline_y, spline_z, sizeof(spline_x), total_pts);

for (new i = 0; i < total_pts; i++)
{
    // Feed continuous coordinates to bot driver or camera interpolation
}
```

---

## 6. Intersection Queries & Topology

Inspect topological node connections to identify multi-way crossroads, T-junctions, and highway on-ramps:

```pawn
native bool:IsMapNodeIntersection(MapNode:nodeid, &connection_count = 0);
native GetMapNodeConnectionTarget(MapNode:nodeid, index, &MapNode:target_nodeid);
```

```pawn
new branches;
if (IsMapNodeIntersection(node, branches) && branches >= 4)
{
    // Node is a 4-way (or greater) major city intersection
    // Ideal for traffic light synchronizers or police checkpoint spawns
}
```

---

## 7. Delta Re-routing & Path Rejoin Point

When a driver deviates or takes a temporary detour, recalculating a 50 km route from scratch wastes CPU time. `GetPathRejoinPoint` calculates where the new detour intersects back into the existing route:

```pawn
native GetPathRejoinPoint(
    Path:old_path, Path:new_path, 
    &diverge_step, 
    &rejoin_old_step, 
    &rejoin_new_step
);
```

```pawn
new diverge_idx, rejoin_old_idx, rejoin_new_idx;
GetPathRejoinPoint(original_path, detour_path, diverge_idx, rejoin_old_idx, rejoin_new_idx);
printf("Detour branches at step %d and successfully rejoins original path at step %d.", 
    diverge_idx, rejoin_old_idx);
```

---

## 8. Lateral Path Offsets & Convoy Math

Compute parallel coordinate vectors to the left or right of a calculated route centerline:

```pawn
native GetPathOffsetPosition(
    Path:pathid, 
    Float:distance_along_path, 
    Float:lateral_offset, 
    &Float:x, &Float:y, &Float:z, &Float:angle
);
```

### Convoy & Motorcade Positioning
* **`lateral_offset < 0.0`:** Position offset to the Left (e.g. `-3.5` meters for left lane / escort).
* **`lateral_offset > 0.0`:** Position offset to the Right (e.g. `+3.5` meters for right lane).

```pawn
new Float:escort_x, Float:escort_y, Float:escort_z, Float:escort_angle;

// Calculate position for an escort vehicle 15 meters behind VIP, 3.5m to the right
GetPathOffsetPosition(vip_path, current_dist - 15.0, 3.5, escort_x, escort_y, escort_z, escort_angle);
SetVehiclePos(escort_vehicle, escort_x, escort_y, escort_z);
SetVehicleZAngle(escort_vehicle, escort_angle);
```
