#pragma once

#include <vector>
#include <atomic>
#include <mutex>
#include "common.h"

class Node
{
public:
	Node(int id, float x, float y, float z, PathType type = PATH_TYPE_VEHICLE, uint32_t flags = NODE_FLAG_NONE);
	Node(const Node&) = delete;
	Node& operator=(const Node&) = delete;

	void setForDeletion();
	bool isSetForDeletion() const;

	void addConnection(Connection* connection);
	void removeConnection(Connection* connection);
	const std::vector<Connection*>& getConnections() const;
	int getOutDegree() const;

	void addToPath(Path* path, int index);
	void removeFromPath(Path* path);
	int getPositionInPath(Path* path);
	int getPathCount() const;

	int getID() const;

	float getX() const;
	float getY() const;
	float getZ() const;

	PathType getType() const;
	void setType(PathType type);

	uint32_t getFlags() const;
	void setFlags(uint32_t flags);

	float getDistanceFromPoint(float pointX, float pointY, float pointZ) const;
	float getDistanceFromNode(Node* other) const;

	float getAngleFromPoint(float pointX, float pointY, bool facing_angle = false) const;
	float getAngleFromNode(Node* other, bool facing_angle = false) const;

	bool isBlocked() const;
	void setBlocked(bool blocked);

	float getDangerWeight() const;
	void setDangerWeight(float weight);

private:
	int id_;
	bool is_set_for_deletion_;
	std::atomic<bool> blocked_;
	std::atomic<float> danger_weight_;
	std::atomic<int> out_degree_{0};

	float x_;
	float y_;
	float z_;

	PathType type_;
	uint32_t flags_;

	std::vector<Connection*> connections_;

	std::vector<std::pair<Path*, int>> paths_;
	mutable std::mutex paths_lock_;
};
