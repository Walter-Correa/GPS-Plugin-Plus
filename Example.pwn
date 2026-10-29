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
    }

    DemonstrateRoadSnappingAndParking(lsDowntownX, lsDowntownY, lsDowntownZ);
    DemonstrateSafeRandomNode(1400.0, -1800.0, 1600.0, -1600.0);

    print("\nDemonstration finished successfully!");
}
