#include <cmath>

#include "connection.h"
#include "node.h"


Connection::Connection(const int id, Node* source, Node* target, const float distance, const float road_width, const int left_lanes, const int right_lanes, const bool one_way)
{
	id_ = id;
	source_ = source;
	target_ = target;
	distance_ = distance;

	road_width_ = road_width;
	left_lanes_ = left_lanes;
	right_lanes_ = right_lanes;
	one_way_ = one_way;
	blocked_ = false;
	traffic_multiplier_ = 1.0f;
}


int Connection::getId() const
{
	return id_;
}


Node* Connection::getSource() const
{
	return source_;
}


Node* Connection::getTarget() const
{
	return target_;
}


float Connection::getDistance() const
{
	return distance_;
}


float Connection::getRoadWidth() const
{
	return road_width_;
}


int Connection::getLeftLanes() const
{
	return left_lanes_;
}


int Connection::getRightLanes() const
{
	return right_lanes_;
}


bool Connection::isOneWay() const
{
	return one_way_;
}


bool Connection::isBlocked() const
{
	return blocked_;
}


void Connection::setBlocked(const bool blocked)
{
	blocked_ = blocked;
}


float Connection::getTrafficMultiplier() const
{
	return traffic_multiplier_;
}


void Connection::setTrafficMultiplier(const float multiplier)
{
	traffic_multiplier_ = (multiplier < 0.01f) ? 0.01f : multiplier;
}


bool Connection::getLanePosition(const int lane_index, float& out_x, float& out_y, float& out_z) const
{
	if (source_ == nullptr || target_ == nullptr)
	{
		return false;
	}

	const auto x1 = source_->getX(), y1 = source_->getY();
	const auto x2 = target_->getX(), y2 = target_->getY();

	auto dx = x2 - x1;
	auto dy = y2 - y1;
	const auto len = std::sqrt(dx * dx + dy * dy);

	if (len <= 0.0001f)
	{
		out_x = x2;
		out_y = y2;
		out_z = target_->getZ();
		return true;
	}

	dx /= len;
	dy /= len;

	const auto perp_x = dy;
	const auto perp_y = -dx;

	const auto lanes = (right_lanes_ > 0) ? right_lanes_ : 1;
	const auto total_lanes = (left_lanes_ > 0 ? left_lanes_ : 1) + lanes;
	const auto lane_width = road_width_ / total_lanes;

	const auto offset = (lane_width * 0.5f) + (lane_index * lane_width);

	out_x = x2 + (perp_x * offset);
	out_y = y2 + (perp_y * offset);
	out_z = target_->getZ();
	return true;
}
