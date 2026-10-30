#include "path.h"


Path::Path(const float length)
{
	id_ = INVALID_PATH_ID;
	length_ = length;
	has_ttl_ = false;
}


int Path::getID() const
{
	return id_;
}


void Path::setID(const int id)
{
	id_ = id;
}


void Path::addNodeToStart(Node* node)
{
	nodes_.push_front(node);
}


void Path::addNodeToEnd(Node* node)
{
	nodes_.push_back(node);
}


float Path::getLength() const
{
	return length_;
}


void Path::setLength(const float length)
{
	length_ = length;
}


std::deque<Node*> Path::getNodes() const
{
	return nodes_;
}


void Path::setTTL(const uint32_t ttl_ms)
{
	if (ttl_ms == 0)
	{
		has_ttl_ = false;
	}
	else
	{
		has_ttl_ = true;
		expiry_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(ttl_ms);
	}
}


bool Path::isExpired() const
{
	if (!has_ttl_)
	{
		return false;
	}
	return std::chrono::steady_clock::now() >= expiry_time_;
}
