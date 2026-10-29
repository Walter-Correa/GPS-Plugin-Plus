#pragma once

#include "container.h"
#include "callback.h"
#include "common.h"


namespace Pathfinder
{
	Path* FindPathInternal(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE);
	int FindPath(Node* start, Node* target, PathType type = PATH_TYPE_VEHICLE);
	void FindPathThreaded(Node* start, Node* target, Callback* callback, PathType type = PATH_TYPE_VEHICLE);
};
