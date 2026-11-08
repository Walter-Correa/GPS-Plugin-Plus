#pragma once

#include <vector>
#include <chrono>

#include "node.h"

struct tagAMX;
typedef struct tagAMX AMX;


class Path
{
public:
	Path(float length, AMX* owner = nullptr);
	virtual ~Path();

	int getID() const;
	void setID(int id);
	void addNodeToStart(Node* node);
	void addNodeToEnd(Node* node);
	float getLength() const;
	void setLength(float length);
	const std::vector<Node*>& getNodes() const;

	void setTTL(uint32_t ttl_ms);
	bool isExpired() const;
	std::chrono::steady_clock::time_point getExpiryTime() const;

	AMX* getOwnerAmx() const;
	void setOwnerAmx(AMX* amx);

private:
	int id_;
	std::vector<Node*> nodes_;
	float length_;
	bool has_ttl_;
	std::chrono::steady_clock::time_point expiry_time_;
	AMX* owner_amx_;
};
