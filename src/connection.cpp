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
	has_toll_ = false;
	toll_cost_ = 0;
	is_paved_ = (road_width_ >= 5.5f);
	speed_limit_ = 0.0f;
	max_height_ = 0.0f;
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


bool Connection::hasToll() const
{
	return has_toll_;
}


void Connection::setToll(const bool has_toll, const int cost)
{
	has_toll_ = has_toll;
	toll_cost_ = cost;
}


int Connection::getTollCost() const
{
	return toll_cost_;
}


bool Connection::isPaved() const
{
	return is_paved_;
}


void Connection::setPaved(const bool paved)
{
	is_paved_ = paved;
}


RoadClass Connection::getRoadClass() const
{
	if ((target_ != nullptr && (target_->getFlags() & NODE_FLAG_HIGHWAY) != 0) || (road_width_ >= 14.0f && right_lanes_ >= 2))
	{
		return ROAD_CLASS_HIGHWAY;
	}
	if (right_lanes_ >= 2 || road_width_ >= 10.0f)
	{
		return ROAD_CLASS_AVENUE;
	}
	if (road_width_ < 5.5f)
	{
		return ROAD_CLASS_ALLEY;
	}
	return ROAD_CLASS_RESIDENTIAL;
}


float Connection::getSpeedLimit() const
{
	if (speed_limit_ > 0.0f)
	{
		return speed_limit_;
	}

	switch (getRoadClass())
	{
	case ROAD_CLASS_ALLEY: return 30.0f;
	case ROAD_CLASS_RESIDENTIAL: return 50.0f;
	case ROAD_CLASS_AVENUE: return 80.0f;
	case ROAD_CLASS_HIGHWAY: return 120.0f;
	default: return 50.0f;
	}
}


void Connection::setSpeedLimit(const float speed)
{
	speed_limit_ = (speed < 0.0f) ? 0.0f : speed;
}


float Connection::getMaxHeight() const
{
	return max_height_;
}


void Connection::setMaxHeight(const float height)
{
	max_height_ = (height < 0.0f) ? 0.0f : height;
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
