#include <cmath>
#include <algorithm>
#include <mutex>
#include "node.h"

Node::Node(const int id, const float x, const float y, const float z, const PathType type, const uint32_t flags)
	: id_(id), is_set_for_deletion_(false), blocked_(false), danger_weight_(0.0f),
	  x_(x), y_(y), z_(z), type_(type), flags_(flags)
{
}

void Node::setForDeletion()
{
	is_set_for_deletion_ = true;
}

bool Node::isSetForDeletion() const
{
	return is_set_for_deletion_;
}

void Node::addConnection(Connection* connection)
{
	connections_.push_back(connection);
	out_degree_.fetch_add(1, std::memory_order_relaxed);
}

void Node::removeConnection(Connection* connection)
{
	const auto it = std::find(connections_.begin(), connections_.end(), connection);
	if (it != connections_.end())
	{
		connections_.erase(it);
		out_degree_.fetch_sub(1, std::memory_order_relaxed);
	}
}

const std::vector<Connection*>& Node::getConnections() const
{
	return connections_;
}

int Node::getOutDegree() const
{
	return out_degree_.load(std::memory_order_relaxed);
}

void Node::addToPath(Path* path, int index)
{
	std::lock_guard<std::mutex> lock(paths_lock_);
	paths_.emplace_back(path, index);
}

int Node::getPositionInPath(Path* path)
{
	std::lock_guard<std::mutex> lock(paths_lock_);
	const auto it = std::find_if(paths_.begin(), paths_.end(), [&path](const std::pair<Path*, int>& pair) { return pair.first == path; });
	return (it == paths_.end()) ? -1 : it->second;
}

void Node::removeFromPath(Path* path)
{
	std::lock_guard<std::mutex> lock(paths_lock_);
	const auto it = std::find_if(paths_.begin(), paths_.end(), [&path](const std::pair<Path*, int>& pair) { return pair.first == path; });
	if (it != paths_.end())
	{
		paths_.erase(it);
	}
}

int Node::getPathCount() const
{
	std::lock_guard<std::mutex> lock(paths_lock_);
	return static_cast<int>(paths_.size());
}

int Node::getID() const
{
	return id_;
}

float Node::getX() const
{
	return x_;
}

float Node::getY() const
{
	return y_;
}

float Node::getZ() const
{
	return z_;
}

PathType Node::getType() const
{
	return type_;
}

void Node::setType(const PathType type)
{
	type_ = type;
}

uint32_t Node::getFlags() const
{
	return flags_;
}

void Node::setFlags(const uint32_t flags)
{
	flags_ = flags;
}

float Node::getDistanceFromPoint(const float pointX, const float pointY, const float pointZ) const
{
	const float dx = pointX - x_;
	const float dy = pointY - y_;
	const float dz = pointZ - z_;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float Node::getDistanceFromNode(Node* other) const
{
	return getDistanceFromPoint(other->getX(), other->getY(), other->getZ());
}

float Node::getAngleFromPoint(const float pointX, const float pointY, const bool facing_angle) const
{
	if (facing_angle)
	{
		float angle = std::atan2(x_ - pointX, pointY - y_) * 180.0f / 3.14159265358979323846f;
		if (angle < 0.0f)
		{
			angle += 360.0f;
		}
		if (angle >= 360.0f)
		{
			angle -= 360.0f;
		}
		return angle;
	}

	return std::atan2(pointY - y_, pointX - x_) * 180.0f / 3.14159265f;
}

float Node::getAngleFromNode(Node* other, const bool facing_angle) const
{
	return getAngleFromPoint(other->getX(), other->getY(), facing_angle);
}

bool Node::isBlocked() const
{
	return blocked_.load(std::memory_order_relaxed);
}

void Node::setBlocked(const bool blocked)
{
	blocked_.store(blocked, std::memory_order_relaxed);
}

float Node::getDangerWeight() const
{
	return danger_weight_.load(std::memory_order_relaxed);
}

void Node::setDangerWeight(const float weight)
{
	danger_weight_.store((weight < 0.0f) ? 0.0f : weight, std::memory_order_relaxed);
}
