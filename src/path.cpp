#include "path.h"


Path::Path(const float length, AMX* owner)
{
	id_ = INVALID_PATH_ID;
	length_ = length;
	has_ttl_ = false;
	owner_amx_ = owner;
}


Path::~Path()
{
	for (auto node : nodes_)
	{
		if (node != nullptr)
		{
			node->removeFromPath(this);
		}
	}
	nodes_.clear();
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
	nodes_.insert(nodes_.begin(), node);
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


const std::vector<Node*>& Path::getNodes() const
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


std::chrono::steady_clock::time_point Path::getExpiryTime() const
{
	return expiry_time_;
}


AMX* Path::getOwnerAmx() const
{
	return owner_amx_;
}


void Path::setOwnerAmx(AMX* amx)
{
	owner_amx_ = amx;
}
