#pragma once

#include "common.h"


class Connection
{
public:
	Connection(int id, Node* source, Node* target, float distance, float road_width = 8.0f, int left_lanes = 1, int right_lanes = 1, bool one_way = false);

	int getId() const;
	Node* getSource() const;
	Node* getTarget() const;
	float getDistance() const;

	float getRoadWidth() const;
	int getLeftLanes() const;
	int getRightLanes() const;
	bool isOneWay() const;

	bool isBlocked() const;
	void setBlocked(bool blocked);

	bool getLanePosition(int lane_index, float& out_x, float& out_y, float& out_z) const;

private:
	int id_;
	Node* source_;
	Node* target_;
	float distance_;

	float road_width_;
	int left_lanes_;
	int right_lanes_;
	bool one_way_;
	bool blocked_;
};
