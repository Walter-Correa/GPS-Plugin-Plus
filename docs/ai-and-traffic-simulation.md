# AI & Traffic Simulation Suite

The **AI & Traffic Simulation Suite** provides specialized algorithms for autonomous vehicle steering, law enforcement pursuit interception, tactical fugitive evasion, real road-distance service queries, reachability isochrones, tunnel detection, and traffic law compliance.

---

## 1. Autonomous NPC Driving (Pure Pursuit Controller)

Steering an NPC vehicle (e.g. via FCNPC or custom vehicle sync packets) along a path requires more than aiming directly at the next node. Sharp turns cause bots to oscillate or overshoot the road.

The **Pure Pursuit** controller calculates an ideal lookahead point along the path arc, yielding a normalized steering wheel ratio ($-1.0$ to $+1.0$) and a recommended cornering speed:

```pawn
native GetPurePursuitSteering(
    Path:pathid, 
    Float:vehicle_x, Float:vehicle_y, Float:vehicle_angle, 
    Float:lookahead_dist, 
    &Float:steer_ratio, 
    &Float:recommended_speed_kmh
);
```

### Parameters & Mathematical Outputs:
* **`lookahead_dist`:** Distance ahead to evaluate (typically $8.0$m for city corners, $20.0$m for freeways).
* **`steer_ratio`:** Normalized steering input:
  * `-1.0`: Full Left lock.
  * `0.0`: Center / Straight.
  * `+1.0`: Full Right lock.
* **`recommended_speed_kmh`:** Automatically slows down on tight hairpins and accelerates on straight highways.

```pawn
// NPC Autonomous Steering Tick (called every 50ms)
public OnBotVehicleUpdate(botid, vehicleid, Path:bot_path)
{
    new Float:vx, Float:vy, Float:vz, Float:v_angle;
    GetVehiclePos(vehicleid, vx, vy, vz);
    GetVehicleZAngle(vehicleid, v_angle);

    new Float:steer, Float:target_speed;
    GetPurePursuitSteering(bot_path, vx, vy, v_angle, 12.0, steer, target_speed);

    // Apply steer to NPC driver controls
    SetBotSteering(botid, steer);
    SetBotCruiseSpeed(botid, target_speed);
    return 1;
}
```

---

## 2. High-Speed Police Pursuit Interception

Chasing a suspect directly from behind is inefficient. Real-world police units use **intercept vectors** to calculate where a fleeing vehicle will be in the near future and dispatch patrol units to cut them off at downstream intersections:

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

```pawn
new Path:intercept_route;
new MapNode:cutoff_node;
new Float:seconds_to_intercept;

// Police cruiser travels at 120 km/h toward suspect travelling at 100 km/h
FindInterceptPath(
    police_node, suspect_node, 
    100.0, 120.0, 
    intercept_route, cutoff_node, seconds_to_intercept
);

if (IsValidPath(intercept_route))
{
    printf("[DISPATCH] Cruiser will intercept suspect at node %d in %.1f seconds!", 
        _:cutoff_node, seconds_to_intercept);
}
```

---

## 3. Tactical Fugitive Evasion Routing

When fleeing law enforcement, criminals avoid dead-end cul-de-sacs, narrow dead-end alleys, and closed bridge bottlenecks. 

`FindEvasivePath` penalizes nodes with low exit degree ($\le 1$) and maximizes future branching entropy:

```pawn
native bool:IsMapNodeDeadEnd(MapNode:nodeid);
native FindEvasivePath(MapNode:source, MapNode:target, &Path:pathid, PathType:type = PATH_TYPE_VEHICLE);
```

```pawn
new Path:fugitive_path;
// Calculates escape route through high-connectivity avenues with multiple exits
FindEvasivePath(suspect_node, safehouse_node, fugitive_path);
```

---

## 4. Speed Radars & Road Hazard Alerts (Waze-Style)

Register static or mobile speed enforcement cameras. The plugin queries upcoming path nodes and alerts drivers before they enter the enforcement zone:

```pawn
native RegisterRoadRadar(radarid, Float:x, Float:y, Float:z, Float:speed_limit_kmh, Float:alert_radius = 300.0);
native bool:UnregisterRoadRadar(radarid);
native ClearRoadRadars();

native bool:GetNextPathRadar(
    Path:pathid, 
    Float:current_dist, 
    &radarid, 
    &Float:dist_to_radar, 
    &Float:radar_limit
);
```

### In-Car Radar Detector Example
```pawn
new radar_id, Float:dist_to_camera, Float:camera_limit;

if (GetNextPathRadar(player_path, current_car_distance, radar_id, dist_to_camera, camera_limit))
{
    if (dist_to_camera <= 200.0)
    {
        new warning[128];
        format(warning, sizeof(warning), "~r~RADAR AHEAD! ~w~%.0fm | Limit: %.0f km/h", 
            dist_to_camera, camera_limit);
        PlayerTextDrawSetString(playerid, g_RadarWarningDraw[playerid], warning);
        PlayerPlaySound(playerid, 1057, 0.0, 0.0, 0.0);
    }
}
```

---

## 5. Nearest Service by Real Road Distance

Standard Euclidean distance checks (`GetPlayerDistanceFromPoint`) often select a destination that appears 100 meters away across a mountain or river, requiring a 15-minute detour.

`FindNearestPointByRoad` executes a multi-target Dijkstra search across the road network, guaranteeing the destination with the **shortest real driving distance**:

```pawn
native FindNearestPointByRoad(
    Float:from_x, Float:from_y, Float:from_z, 
    const Float:points[][3], 
    total_points, 
    &nearest_index, 
    &Float:road_distance, 
    &Path:pathid = Path:0, 
    PathType:type = PATH_TYPE_VEHICLE
);
```

```pawn
new Float:hospitals[3][3] = {
    { 1172.0, -1323.0, 15.0 }, // All Saints, LS
    { 2026.0, -1420.0, 16.9 }, // Jefferson, LS
    { -2655.0, 635.0, 14.4 }   // San Fierro Medical
};

new closest_idx, Float:actual_km, Path:route_to_hospital;
FindNearestPointByRoad(
    px, py, pz, 
    hospitals, sizeof(hospitals), 
    closest_idx, actual_km, route_to_hospital
);

printf("Closest hospital is #%d (Real Road Distance: %.1f meters)", closest_idx, actual_km);
```

---

## 6. Reachability Isochrones & Driving Range

Calculate all road nodes that can be reached from a starting location within a given **time** or **distance budget**:

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

### Use Cases:
* **Pizza Delivery / Hot Food Range:** Nodes reachable within 3 minutes ($180$ seconds).
* **Fuel Range:** Nodes reachable with remaining fuel liters.
* **Police Perimeter:** Geographic bounding box for road spikes.

---

## 7. Tunnel & Underground Detection

Simulate realistic GPS satellite signal loss when vehicles enter underground parking structures or railway tunnels:

```pawn
native RegisterTunnelArea(Float:minx, Float:miny, Float:minz, Float:maxx, Float:maxy, Float:maxz, &tunnel_id);
native bool:UnregisterTunnelArea(tunnel_id);
native ClearCustomTunnelAreas();

native bool:IsPointInTunnel(Float:x, Float:y, Float:z);
native bool:IsMapNodeUnderground(MapNode:nodeid);
native bool:GetPathTunnelSegments(Path:pathid, &Float:entry_distance, &Float:exit_distance);
```

```pawn
if (IsPointInTunnel(vx, vy, vz))
{
    // Disable in-car GPS minimap or simulate satellite static
    PlayerTextDrawSetString(playerid, g_GPSStatusDraw[playerid], "~r~GPS SIGNAL LOST (TUNNEL)");
}
```

---

## 8. Wrong-Way Traffic Enforcement

Detect reckless drivers driving in the wrong direction on divided freeways and one-way streets:

```pawn
native IsPointAgainstTraffic(
    Float:x, Float:y, Float:z, 
    Float:facing_angle, 
    &bool:against_traffic
);
```

```pawn
new bool:is_wrong_way;
IsPointAgainstTraffic(car_x, car_y, car_z, car_heading, is_wrong_way);

if (is_wrong_way)
{
    SendClientMessage(playerid, 0xE74C3CAA, "[POLICE WARNING] You are driving against traffic!");
}
```

---

## 9. Road Classification & Legal Speed Limits

```pawn
enum RoadClass
{
    ROAD_CLASS_ALLEY       = 0, // 30 km/h
    ROAD_CLASS_RESIDENTIAL = 1, // 50 km/h
    ROAD_CLASS_AVENUE      = 2, // 80 km/h
    ROAD_CLASS_HIGHWAY     = 3  // 120 km/h
}

native RoadClass:GetConnectionRoadClass(Connection:connectionid, &Float:recommended_speed_kmh = 0.0);
native SetConnectionSpeedLimit(Connection:connectionid, Float:speed_kmh);
native Float:GetConnectionSpeedLimit(Connection:connectionid);
```
