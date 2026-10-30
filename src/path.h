#pragma once

#include <deque>
#include <chrono>

#include "node.h"


class Path
{
public:
	Path(float length);
	int getID() const;
	void setID(int id);
	void addNodeToStart(Node* node);
	void addNodeToEnd(Node* node);
	float getLength() const;
	void setLength(float length);
	std::deque<Node*> getNodes() const;

	void setTTL(uint32_t ttl_ms);
	bool isExpired() const;

private:
	int id_;
	std::deque<Node*> nodes_;
	float length_;
	bool has_ttl_;
	std::chrono::steady_clock::time_point expiry_time_;
};