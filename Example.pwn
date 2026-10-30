/*
================================================================================
    GPS Plugin Plus - Comprehensive Demonstration Script
    Continuation & Modernization of AmyrAhmady/samp-gps-plugin
    Author: Walter Correa (Walter-Correa)

    Features demonstrated:
    - 2D Spatial Hash Grid pathfinding (FindPath, FindPathThreaded)
    - Dynamic Roadblock rerouting (SetMapNodeBlocked, SetConnectionBlocked)
    - Road Snapping & Parking Offsets (GetClosestPointOnRoad, GetRoadPositionInfo)
    - Wrong-Way Traffic Monitoring (IsPlayerDrivingAgainstTraffic, IsPointAgainstTraffic)
    - Turn-by-Turn Navigation & ETA (GetPathTurnDirection, GetPathEstimatedTravelTime)
    - Smart Height/Layer Bounding (GetRandomMapNodeFromArea with minz, maxz)
================================================================================
*/

#include <a_samp>
#include <GPS>

// -----------------------------------------------------------------------------
// Helper: Detect whether a player in a vehicle is driving against traffic flow
// -----------------------------------------------------------------------------
stock bool:IsPlayerDrivingAgainstTraffic(playerid)
{
    if (!IsPlayerInAnyVehicle(playerid)) return false;

    new vehicleid = GetPlayerVehicleID(playerid);
    new Float:x, Float:y, Float:z, Float:angle;
    GetVehiclePos(vehicleid, x, y, z);
    GetVehicleZAngle(vehicleid, angle);

    new bool:against_traffic = false;
    IsPointAgainstTraffic(x, y, z, angle, against_traffic);
    return against_traffic;
}

// -----------------------------------------------------------------------------
// Example 1: Basic Synchronous Route Calculation with Directions and ETA
// -----------------------------------------------------------------------------
DemonstrateBasicRoute(Float:startX, Float:startY, Float:startZ, Float:destX, Float:destY, Float:destZ)
{
    printf("\n--- [Demo 1: Synchronous Route & Turn-by-Turn Directions] ---");

    new MapNode:startNode, MapNode:destNode;
    if (GetClosestMapNodeToPoint(startX, startY, startZ, startNode) != GPS_ERROR_NONE)
    {
        printf("Error: Could not locate start node!");
        return 0;
    }

    if (GetClosestMapNodeToPoint(destX, destY, destZ, destNode) != GPS_ERROR_NONE)
    {
        printf("Error: Could not locate destination node!");
        return 0;
    }

    new Path:pathid;
    new startTime = GetTickCount();
    new error = FindPath(startNode, destNode, pathid);
    new elapsed = GetTickCount() - startTime;

    if (error != GPS_ERROR_NONE || !IsValidPath(pathid))
    {
        printf("Failed to find path. Error code: %d", error);
        return 0;
    }

    new size;
    new Float:length;
    GetPathSize(pathid, size);
    GetPathLength(pathid, length);

    new Float:travelTimeSeconds;
    GetPathEstimatedTravelTime(pathid, 60.0, travelTimeSeconds); // 60 km/h average speed

    printf("Path found in %d ms! Total nodes: %d | Length: %.2f meters | ETA (at 60 km/h): %.1f seconds",
        elapsed, size, length, travelTimeSeconds);

    // Inspect first 5 step maneuvers
    new inspectLimit = (size < 5) ? size : 5;
    for (new i = 0; i < inspectLimit; i++)
    {
        new TurnDirection:turn;
        new Float:turnAngle;
        GetPathTurnDirection(pathid, i, turn, turnAngle);

        new turnName[20];
        switch (turn)
        {
            case TURN_STRAIGHT:     turnName = "Straight";
            case TURN_RIGHT:        turnName = "Turn Right";
            case TURN_LEFT:         turnName = "Turn Left";
            case TURN_SHARP_RIGHT:  turnName = "Sharp Right";
            case TURN_SHARP_LEFT:   turnName = "Sharp Left";
            case TURN_UTURN:        turnName = "U-Turn";
            default:                turnName = "None / Start";
        }

        printf("  Step %d: Maneuver: %s (Heading delta: %.1f deg)", i + 1, turnName, turnAngle);
    }

    // Always free paths when no longer needed to prevent memory leaks
    DestroyPath(pathid);
    return 1;
}

// -----------------------------------------------------------------------------
// Example 2: Asynchronous Threaded Route Calculation
// -----------------------------------------------------------------------------
forward OnThreadedRouteCalculated(Path:pathid, extraParam);
public OnThreadedRouteCalculated(Path:pathid, extraParam)
{
    printf("\n--- [Demo 2: Asynchronous Threaded Route Callback] ---");
    if (!IsValidPath(pathid))
    {
        printf("Threaded path calculation failed (Extra param: %d)", extraParam);
        return;
    }

    new size;
    new Float:length;
    GetPathSize(pathid, size);
    GetPathLength(pathid, length);

    printf("Threaded route completed! Nodes: %d, Length: %.2f m, Extra ID: %d", size, length, extraParam);
    DestroyPath(pathid);
}

DemonstrateThreadedRoute(MapNode:startNode, MapNode:destNode)
{
    // Computes path in a worker background thread without blocking server ticks
    FindPathThreaded(startNode, destNode, "OnThreadedRouteCalculated", "i", 1337);
}

// -----------------------------------------------------------------------------
// Example 3: Dynamic Roadblocks and Automatic Detour Calculation
// -----------------------------------------------------------------------------
DemonstrateRoadblockDetour(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 3: Dynamic Roadblocks & Detour Rerouting] ---");

    new Path:normalPath;
    FindPath(startNode, destNode, normalPath);

    new Float:normalLength = 0.0;
    if (IsValidPath(normalPath))
    {
        GetPathLength(normalPath, normalLength);
        printf("Normal route distance without blockades: %.2f meters", normalLength);

        // Find a middle node along this route and place a police roadblock on it
        new size;
        GetPathSize(normalPath, size);
        if (size > 2)
        {
            new MapNode:blockNode;
            GetPathNode(normalPath, size / 2, blockNode);

            // Block node dynamically
            SetMapNodeBlocked(blockNode, true);
            printf("Placed police roadblock on node %d!", _:blockNode);

            // Calculate detour
            new Path:detourPath;
            new error = FindPath(startNode, destNode, detourPath);
            if (error == GPS_ERROR_NONE && IsValidPath(detourPath))
            {
                new Float:detourLength;
                GetPathLength(detourPath, detourLength);
                printf("Detour calculated! New distance: %.2f meters (Delta: +%.2f m)",
                    detourLength, detourLength - normalLength);
                DestroyPath(detourPath);
            }
            else
            {
                printf("No alternative path found around the roadblock!");
            }

            // Remove roadblock to restore normal traffic flow
            SetMapNodeBlocked(blockNode, false);
            printf("Roadblock cleared from node %d.", _:blockNode);
        }

        DestroyPath(normalPath);
    }
}

// -----------------------------------------------------------------------------
// Example 4: Orthogonal Road Snapping & Curb Positioning
// -----------------------------------------------------------------------------
DemonstrateRoadSnappingAndParking(Float:playerX, Float:playerY, Float:playerZ)
{
    printf("\n--- [Demo 4: Road Centerline vs Roadside Curb Positioning] ---");

    new Float:centerX, Float:centerY, Float:centerZ, Float:roadAngle, Connection:connId;
    if (GetClosestPointOnRoad(playerX, playerY, playerZ, centerX, centerY, centerZ, roadAngle, connId) == GPS_ERROR_NONE)
    {
        printf("Centerline Snapping: Position: (%.2f, %.2f, %.2f) | Heading: %.1f deg | Connection: %d",
            centerX, centerY, centerZ, roadAngle, _:connId);
    }

    new Float:curbX, Float:curbY, Float:curbZ, Float:curbAngle, Float:roadWidth;
    // ROAD_POS_CURB offsets to (road_width / 2.0) + 1.3m, perfect for sidewalk parking
    if (GetRoadPositionInfo(playerX, playerY, playerZ, curbX, curbY, curbZ, curbAngle, roadWidth, ROAD_POS_CURB) == GPS_ERROR_NONE)
    {
        printf("Curb/Sidewalk Snapping: Position: (%.2f, %.2f, %.2f) | Road Width: %.1f m",
            curbX, curbY, curbZ, roadWidth);
    }

    new Float:edgeX, Float:edgeY, Float:edgeZ, Float:edgeAngle;
    // ROAD_POS_EDGE offsets to (road_width / 2.0), right beside barriers/guardrails
    if (GetRoadPositionInfo(playerX, playerY, playerZ, edgeX, edgeY, edgeZ, edgeAngle, roadWidth, ROAD_POS_EDGE) == GPS_ERROR_NONE)
    {
        printf("Asphalt Edge Snapping: Position: (%.2f, %.2f, %.2f)",
            edgeX, edgeY, edgeZ);
    }
}

// -----------------------------------------------------------------------------
// Example 5: Safe Territory Spawn with Layer Isolation
// -----------------------------------------------------------------------------
DemonstrateSafeRandomNode(Float:minX, Float:minY, Float:maxX, Float:maxY)
{
    printf("\n--- [Demo 5: Safe Random Node in Area (Layer Isolation)] ---");

    new MapNode:safeNode;
    // By default, GetRandomMapNodeFromArea ignores airborne interiors (Z > 500) and submerged roads (Z <= 1.0)
    if (GetRandomMapNodeFromArea(minX, minY, maxX, maxY, safeNode) == GPS_ERROR_NONE)
    {
        new Float:nx, Float:ny, Float:nz;
        GetMapNodePos(safeNode, nx, ny, nz);
        printf("Selected safe land vehicle node %d at (%.2f, %.2f, %.2f)",
            _:safeNode, nx, ny, nz);
    }
    else
    {
        printf("No valid vehicle nodes found in specified bounding box.");
    }
}

// -----------------------------------------------------------------------------
// Example 6: Real-Time HUD Tracking, Next Maneuver & Off-Route Rerouting
// -----------------------------------------------------------------------------
DemonstrateRealtimeNavigation(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 6: Real-Time HUD Tracking & Off-Route Detection] ---");

    new Path:pathid;
    if (FindPath(startNode, destNode, pathid) != GPS_ERROR_NONE || !IsValidPath(pathid))
    {
        return;
    }

    // Simulate vehicle position near starting area
    new Float:vehicleX = 1540.0, Float:vehicleY = -1670.0, Float:vehicleZ = 13.5;

    new currentStep;
    new Float:distanceToPath;
    new bool:offRoute;
    GetPathProgress(pathid, vehicleX, vehicleY, vehicleZ, currentStep, distanceToPath, offRoute, 25.0);

    new Float:remainingDist, Float:remainingSec;
    GetPathRemainingInfo(pathid, currentStep, remainingDist, remainingSec, 60.0);

    new TurnDirection:nextTurn;
    new Float:distanceToTurn;
    GetPathNextManeuver(pathid, vehicleX, vehicleY, vehicleZ, nextTurn, distanceToTurn);

    printf("HUD: Step %d | Dist to Path: %.1fm | Off Route: %s | Remaining: %.1fm (ETA: %.1fs)",
        currentStep, distanceToPath, offRoute ? ("YES (Recalculating...)") : ("NO"), remainingDist, remainingSec);
    printf("Next Maneuver: %d at %.1f meters ahead", _:nextTurn, distanceToTurn);

    DestroyPath(pathid);
}

// -----------------------------------------------------------------------------
// Example 7: Multi-Stop Optimized Delivery Route (e.g. Courier / Trucking)
// -----------------------------------------------------------------------------
DemonstrateMultiStopDelivery(MapNode:hubNode, MapNode:dropA, MapNode:dropB, MapNode:dropC)
{
    printf("\n--- [Demo 7: Multi-Stop Optimized Delivery Route] ---");

    new MapNode:stops[4];
    stops[0] = hubNode;
    stops[1] = dropA;
    stops[2] = dropB;
    stops[3] = dropC;

    new Path:multiPath;
    // optimize_order = true solves TSP to minimize overall driving distance
    if (FindMultiStopPath(stops, sizeof(stops), multiPath, true, PATH_TYPE_VEHICLE, ROUTE_PREF_HIGHWAYS) == GPS_ERROR_NONE)
    {
        new size;
        new Float:length;
        GetPathSize(multiPath, size);
        GetPathLength(multiPath, length);

        printf("Multi-Stop route calculated successfully! Total nodes: %d | Combined Length: %.2f m",
            size, length);
        DestroyPath(multiPath);
    }
}

// -----------------------------------------------------------------------------
// Example 8: Douglas-Peucker Checkpoint Decimation (Saving Streamer Limits)
// -----------------------------------------------------------------------------
DemonstratePathSimplification(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 8: Douglas-Peucker Checkpoint Decimation] ---");

    new Path:longPath;
    if (FindPath(startNode, destNode, longPath) == GPS_ERROR_NONE && IsValidPath(longPath))
    {
        new originalCount;
        GetPathSize(longPath, originalCount);

        new MapNode:decimatedNodes[128];
        new decimatedCount = 0;
        // 10.0m tolerance preserves every corner and curve while eliminating straight highway redundancy
        GetSimplifiedPathNodes(longPath, 10.0, decimatedNodes, sizeof(decimatedNodes), decimatedCount);

        printf("Original Path Nodes: %d -> Simplified Checkpoints: %d (%.1f%% reduction!)",
            originalCount, decimatedCount, 100.0 - ((float(decimatedCount) / float(originalCount)) * 100.0));

        DestroyPath(longPath);
    }
}

// -----------------------------------------------------------------------------
// Example 9: Map Node Intersection & AI Road Choice
// -----------------------------------------------------------------------------
DemonstrateIntersectionDetection(MapNode:nodeid)
{
    printf("\n--- [Demo 9: Intersection Detection for NPC Traffic] ---");

    new connCount;
    new bool:isJunction = IsMapNodeIntersection(nodeid, connCount);

    printf("Node %d is junction: %s (Total connected road branches: %d)",
        _:nodeid, isJunction ? ("TRUE (3+ ways)") : ("FALSE"), connCount);

    for (new i = 0; i < connCount; i++)
    {
        new MapNode:targetNode;
        if (GetMapNodeConnectionTarget(nodeid, i, targetNode) == GPS_ERROR_NONE)
        {
            printf("  Branch %d leads to Node %d", i + 1, _:targetNode);
        }
    }
}

// -----------------------------------------------------------------------------
// Example 10: Dynamic Live Traffic & Congestion Detours
// -----------------------------------------------------------------------------
DemonstrateLiveTraffic(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 10: Live Traffic Congestion & Detours] ---");

    new Path:normalPath;
    FindPath(startNode, destNode, normalPath);

    // Simulate traffic jam: 4x slower cost in downtown area
    SetAreaTrafficMultiplier(1400.0, -1800.0, 1700.0, -1500.0, 4.0);

    new Path:trafficPath;
    FindPath(startNode, destNode, trafficPath);

    new Float:len1, Float:len2;
    GetPathLength(normalPath, len1);
    GetPathLength(trafficPath, len2);
    printf("Clear traffic distance: %.1fm | Congested detour distance: %.1fm", len1, len2);

    ResetTrafficMultipliers();
    DestroyPath(normalPath);
    DestroyPath(trafficPath);
}

// -----------------------------------------------------------------------------
// Example 11: Police Pursuit Interception Prediction
// -----------------------------------------------------------------------------
DemonstrateTacticalIntercept(MapNode:policeNode, MapNode:fugitiveNode)
{
    printf("\n--- [Demo 11: Police Pursuit Tactical Interception] ---");

    new Path:interceptPath;
    new MapNode:interceptNode;
    new Float:timeToIntercept;

    // Fugitive driving at 90 km/h, Police pursuit unit responding at 130 km/h
    if (FindInterceptPath(policeNode, fugitiveNode, 90.0, 130.0, interceptPath, interceptNode, timeToIntercept) == GPS_ERROR_NONE)
    {
        printf("Tactical Intercept Point: Node %d | Estimated Time: %.1f seconds",
            _:interceptNode, timeToIntercept);
        DestroyPath(interceptPath);
    }
}

// -----------------------------------------------------------------------------
// Example 12: Pure Pursuit Autonomous Bot Driving Controller
// -----------------------------------------------------------------------------
DemonstratePurePursuit(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 12: Autonomous NPC Pure Pursuit Controller] ---");

    new Path:pathid;
    if (FindPath(startNode, destNode, pathid) == GPS_ERROR_NONE)
    {
        new Float:botX = 1545.0, Float:botY = -1670.0, Float:botAngle = 90.0;
        new Float:steerRatio, Float:recommendedSpeed;

        GetPurePursuitSteering(pathid, botX, botY, botAngle, 10.0, steerRatio, recommendedSpeed);
        printf("Pure Pursuit Output: Steer Ratio: %.2f (-1.0 left to +1.0 right) | Speed: %.1f km/h",
            steerRatio, recommendedSpeed);

        DestroyPath(pathid);
    }
}

// -----------------------------------------------------------------------------
// Example 13: Radial Reachability & Isochrones
// -----------------------------------------------------------------------------
DemonstrateReachability(MapNode:hubNode)
{
    printf("\n--- [Demo 13: Radial Reachability & Isochrones] ---");

    new MapNode:reachable[64];
    new reachedCount;
    // Find all nodes reachable in 30 seconds at 60 km/h
    GetReachableMapNodes(hubNode, 30.0, true, 60.0, reachable, sizeof(reachable), reachedCount);

    new Float:minX, Float:minY, Float:maxX, Float:maxY;
    GetReachableBoundingBox(hubNode, 500.0, minX, minY, maxX, maxY);

    printf("Reachable nodes in 30s: %d | Coverage Box: (%.1f, %.1f) to (%.1f, %.1f)",
        reachedCount, minX, minY, maxX, maxY);
}

// -----------------------------------------------------------------------------
// Example 14: San Andreas Zone Lookups & Route Neighborhood Summary
// -----------------------------------------------------------------------------
DemonstrateZones(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 14: Native San Andreas Zone Lookups] ---");

    new zone[32];
    GetPointZoneName(2490.0, -1670.0, 13.5, zone);
    printf("Coordinate (2490.0, -1670.0) is in: %s", zone);

    new Path:pathid;
    if (FindPath(startNode, destNode, pathid) == GPS_ERROR_NONE)
    {
        new zonesList[8][32];
        new zonesCount;
        GetPathZonesSummary(pathid, zonesList, sizeof(zonesList), zonesCount);

        printf("Route itinerary crosses %d zones:", zonesCount);
        for (new i = 0; i < zonesCount; i++)
        {
            printf("  Leg %d: %s", i + 1, zonesList[i]);
        }
        DestroyPath(pathid);
    }
}

// -----------------------------------------------------------------------------
// Example 15: Path Auto-Destruction TTL & String Serialization
// -----------------------------------------------------------------------------
DemonstrateSerializationAndTTL(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 15: Serialization & Anti-Leak TTL] ---");

    new Path:pathid;
    if (FindPath(startNode, destNode, pathid) == GPS_ERROR_NONE)
    {
        // Set auto-destruction safety timer in case script fails to free
        SetPathAutoDestroy(pathid, 30000); // Auto-deletes in 30s

        new buffer[256];
        ExportPathToString(pathid, buffer);
        printf("Exported Path String (first 60 chars): %.60s...", buffer);

        new Path:importedPath;
        if (ImportPathFromString(buffer, importedPath) == GPS_ERROR_NONE)
        {
            printf("Successfully re-imported path ID %d from database string!", _:importedPath);
            DestroyPath(importedPath);
        }

        DestroyPath(pathid);
    }
}

// -----------------------------------------------------------------------------
// Example 16: Heavy Truck vs Motorcycle Category Restrictions
// -----------------------------------------------------------------------------
DemonstrateVehicleCategoryRouting(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 16: Heavy Truck vs Motorcycle Category Routing] ---");

    new Path:truckPath, Path:motoPath;
    FindPathForVehicleCat(startNode, destNode, truckPath, VEHICLE_CAT_HEAVY_TRUCK);
    FindPathForVehicleCat(startNode, destNode, motoPath, VEHICLE_CAT_MOTORCYCLE);

    new Float:truckLen, Float:motoLen;
    GetPathLength(truckPath, truckLen);
    GetPathLength(motoPath, motoLen);

    printf("Heavy Truck Path (width >= 6m): %.1fm | Motorcycle Path (shortcuts): %.1fm",
        truckLen, motoLen);

    DestroyPath(truckPath);
    DestroyPath(motoPath);
}

// -----------------------------------------------------------------------------
// Example 17: Safe-Route Anti-Crime Routing
// -----------------------------------------------------------------------------
DemonstrateSafeRoute(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 17: Safe-Route Anti-Crime Routing] ---");

    // Mark an area as a high-crime red zone (e.g. gang territory)
    SetAreaDangerWeight(1800.0, -1950.0, 2200.0, -1650.0, 3.5);

    new Path:normalPath, Path:safePath;
    FindPath(startNode, destNode, normalPath);
    FindSafePath(startNode, destNode, safePath, 8.0); // 8.0 danger avoidance factor

    new Float:normLen, Float:safeLen;
    GetPathLength(normalPath, normLen);
    GetPathLength(safePath, safeLen);

    printf("Normal Route: %.1fm | Safe Route (detouring red zone): %.1fm",
        normLen, safeLen);

    ResetDangerWeights();
    DestroyPath(normalPath);
    DestroyPath(safePath);
}

// -----------------------------------------------------------------------------
// Example 18: Eco-Routing & Elevation Climb Stats
// -----------------------------------------------------------------------------
DemonstrateEcoRouting(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 18: Eco-Routing & Elevation Climb Stats] ---");

    new Path:ecoPath;
    if (FindEcoPath(startNode, destNode, ecoPath, 0.10) == GPS_ERROR_NONE) // Max 10% incline preference
    {
        new Float:minZ, Float:maxZ, Float:totalClimb, Float:maxIncline;
        GetPathElevationStats(ecoPath, minZ, maxZ, totalClimb, maxIncline);

        printf("Eco Route: Min Z: %.1f | Max Z: %.1f | Total Climb: %.1fm | Max Incline: %.1f%%",
            minZ, maxZ, totalClimb, maxIncline);

        DestroyPath(ecoPath);
    }
}

// -----------------------------------------------------------------------------
// Example 19: Tunnel & Underground Detection
// -----------------------------------------------------------------------------
DemonstrateTunnels(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 19: Tunnel & Underground Detection] ---");

    new Path:pathid;
    if (FindPath(startNode, destNode, pathid) == GPS_ERROR_NONE)
    {
        new Float:entryDist, Float:exitDist;
        if (GetPathTunnelSegments(pathid, entryDist, exitDist))
        {
            printf("Path crosses a tunnel! Entry at: %.1fm | Exit at: %.1fm", entryDist, exitDist);
        }
        else
        {
            printf("Path stays entirely above ground (no tunnels).");
        }
        DestroyPath(pathid);
    }
}

// -----------------------------------------------------------------------------
// Example 20: Nearest Facility by True Road Distance (Anti-Euclidean)
// -----------------------------------------------------------------------------
DemonstrateNearestServiceByRoad(Float:playerX, Float:playerY, Float:playerZ)
{
    printf("\n--- [Demo 20: Nearest Service by Real Road Network Distance] ---");

    // 3 Hospitals: All Saints (LS), Jefferson (LS), Crippen Memorial (Montgomery)
    new Float:hospitals[3][3] = {
        { 1177.0, -1323.0, 14.0 },
        { 2033.0, -1405.0, 17.2 },
        { 1242.0, 327.0, 19.5 }
    };

    new nearestIdx, Float:roadDist, Path:navPath;
    if (FindNearestPointByRoad(playerX, playerY, playerZ, hospitals, sizeof(hospitals), nearestIdx, roadDist, navPath) == GPS_ERROR_NONE)
    {
        printf("Closest Hospital by Asphalt: #%d at %.1f meters (Euclidean distance bypassed!)",
            nearestIdx, roadDist);

        if (IsValidPath(navPath))
        {
            DestroyPath(navPath);
        }
    }
}

// -----------------------------------------------------------------------------
// Example 21: Waze-Style Speed Radars on Path
// -----------------------------------------------------------------------------
DemonstrateSpeedRadars(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 21: Speed Radars & Early Road Hazards (Waze-style)] ---");

    RegisterRoadRadar(1, 1500.0, -1700.0, 13.5, 80.0, 250.0);
    RegisterRoadRadar(2, 800.0, -1800.0, 10.0, 60.0, 250.0);

    new Path:pathid;
    if (FindPath(startNode, destNode, pathid) == GPS_ERROR_NONE)
    {
        new radarId, Float:distToRadar, Float:radarLimit;
        if (GetNextPathRadar(pathid, 50.0, radarId, distToRadar, radarLimit))
        {
            printf("Radar Alert: ID %d at %.1fm ahead! Speed Limit: %.0f km/h",
                radarId, distToRadar, radarLimit);
        }
        DestroyPath(pathid);
    }

    ClearRoadRadars();
}

// -----------------------------------------------------------------------------
// Example 22: Convoy and Escort Formation
// -----------------------------------------------------------------------------
DemonstrateConvoyOffsets(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 22: Convoy & Escort Formation Offsets] ---");

    new Path:pathid;
    if (FindPath(startNode, destNode, pathid) == GPS_ERROR_NONE)
    {
        new Float:leadDist = 120.0;
        new Float:escortLeftX, Float:escortLeftY, Float:escortLeftZ, Float:angLeft;
        new Float:escortRightX, Float:escortRightY, Float:escortRightZ, Float:angRight;

        // Escort 1: 15m behind leader, 3m to the left
        GetPathOffsetPosition(pathid, leadDist - 15.0, -3.0, escortLeftX, escortLeftY, escortLeftZ, angLeft);
        // Escort 2: 15m behind leader, 3m to the right
        GetPathOffsetPosition(pathid, leadDist - 15.0, 3.0, escortRightX, escortRightY, escortRightZ, angRight);

        printf("Leader at %.0fm | Left Wing: (%.1f, %.1f) | Right Wing: (%.1f, %.1f)",
            leadDist, escortLeftX, escortLeftY, escortRightX, escortRightY);

        DestroyPath(pathid);
    }
}

// -----------------------------------------------------------------------------
// Example 23: Tactical Evasive Escape Routing
// -----------------------------------------------------------------------------
DemonstrateEvasiveEscape(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 23: Tactical Evasive Escape Routing (Anti-Dead-End)] ---");

    new Path:evasivePath;
    if (FindEvasivePath(startNode, destNode, evasivePath) == GPS_ERROR_NONE)
    {
        new Float:len;
        GetPathLength(evasivePath, len);
        printf("Evasive Pursuit Route calculated: %.1fm (Avoids highways and dead-ends, favors alleys)", len);
        DestroyPath(evasivePath);
    }
}

// -----------------------------------------------------------------------------
// Example 24: Toll Road Restrictions
// -----------------------------------------------------------------------------
DemonstrateTollAvoidance(MapNode:startNode, MapNode:destNode)
{
    printf("\n--- [Demo 24: Toll Road & Bridge Restrictions] ---");

    new Path:tollFreePath;
    if (FindPathAvoidTolls(startNode, destNode, tollFreePath) == GPS_ERROR_NONE)
    {
        new Float:len;
        GetPathLength(tollFreePath, len);
        printf("Toll-Free Route: %.1fm", len);
        DestroyPath(tollFreePath);
    }
}

// -----------------------------------------------------------------------------
// Main Entry: Executes test suite
// -----------------------------------------------------------------------------
main()
{
    print("=================================================");
    print("   GPS Plugin Plus - Feature Demonstration     ");
    print("=================================================");

    // Coordinates: Los Santos Downtown to Santa Maria Beach
    new Float:lsDowntownX = 1544.0, Float:lsDowntownY = -1675.0, Float:lsDowntownZ = 13.5;
    new Float:smBeachX = 358.0, Float:smBeachY = -2033.0, Float:smBeachZ = 7.8;

    DemonstrateBasicRoute(lsDowntownX, lsDowntownY, lsDowntownZ, smBeachX, smBeachY, smBeachZ);

    new MapNode:nodeA, MapNode:nodeB;
    if (GetClosestMapNodeToPoint(lsDowntownX, lsDowntownY, lsDowntownZ, nodeA) == GPS_ERROR_NONE &&
        GetClosestMapNodeToPoint(smBeachX, smBeachY, smBeachZ, nodeB) == GPS_ERROR_NONE)
    {
        DemonstrateRoadblockDetour(nodeA, nodeB);
        DemonstrateThreadedRoute(nodeA, nodeB);
        DemonstrateRealtimeNavigation(nodeA, nodeB);
        DemonstratePathSimplification(nodeA, nodeB);
        DemonstrateIntersectionDetection(nodeA);
        DemonstrateLiveTraffic(nodeA, nodeB);
        DemonstratePurePursuit(nodeA, nodeB);
        DemonstrateReachability(nodeA);
        DemonstrateZones(nodeA, nodeB);
        DemonstrateSerializationAndTTL(nodeA, nodeB);
        DemonstrateVehicleCategoryRouting(nodeA, nodeB);
        DemonstrateSafeRoute(nodeA, nodeB);
        DemonstrateEcoRouting(nodeA, nodeB);
        DemonstrateTunnels(nodeA, nodeB);
        DemonstrateSpeedRadars(nodeA, nodeB);
        DemonstrateConvoyOffsets(nodeA, nodeB);
        DemonstrateEvasiveEscape(nodeA, nodeB);
        DemonstrateTollAvoidance(nodeA, nodeB);
    }

    new MapNode:nodeC, MapNode:nodeD;
    GetClosestMapNodeToPoint(1000.0, -1500.0, 13.5, nodeC);
    GetClosestMapNodeToPoint(2000.0, -1200.0, 20.0, nodeD);
    if (IsValidMapNode(nodeA) && IsValidMapNode(nodeB) && IsValidMapNode(nodeC) && IsValidMapNode(nodeD))
    {
        DemonstrateMultiStopDelivery(nodeA, nodeC, nodeD, nodeB);
        DemonstrateTacticalIntercept(nodeA, nodeC);
    }

    DemonstrateRoadSnappingAndParking(lsDowntownX, lsDowntownY, lsDowntownZ);
    DemonstrateSafeRandomNode(1400.0, -1800.0, 1600.0, -1600.0);
    DemonstrateNearestServiceByRoad(lsDowntownX, lsDowntownY, lsDowntownZ);

    print("\nDemonstration finished successfully!");
}
