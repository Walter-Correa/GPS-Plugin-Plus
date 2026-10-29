#include <fstream>
#include <sstream>
#include <iostream>

#include "file.h"
#include "container.h"
#include "connection.h"
#include "node.h"


namespace File
{
	bool LoadNodes(const std::string& filename, int& line_count, int& node_count, int& connection_count)
	{
		std::ifstream file("scriptfiles/" + filename);

		if (!file.good())
		{
			return false;
		}

		Container::LockExclusive();

		std::string buffer;
		auto type = 0, id = 0, id2 = 0, direction = 0, ignore = 0;
		auto x = 0.0f, y = 0.0f, z = 0.0f;

		while (std::getline(file, buffer))
		{
			std::istringstream input(buffer);

			input >> type;

			switch (type)
			{
			case 0:
			{
				input >> x >> y >> z >> ignore >> id;

				auto node_type = 0;
				auto node_flags = 0u;

				if (input >> node_type)
				{
					input >> node_flags;
				}

				if (Container::Nodes::Add(id, x, y, z, static_cast<PathType>(node_type), node_flags))
				{
					node_count++;
				}

				break;
			}

			case 1:
			{
				input >> id >> id2 >> direction;

				auto road_width = 8.0f;
				auto left_lanes = 1;
				auto right_lanes = 1;
				auto oneway = 0;

				if (input >> road_width)
				{
					input >> left_lanes >> right_lanes >> oneway;
				}

				if (direction != 2 && Container::Connections::Add(id, id2, road_width, left_lanes, right_lanes, oneway != 0))
				{
					connection_count++;
				}

				break;
			}
			}

			line_count++;
		}

		Container::UnlockExclusive();

		file.close();
		return true;
	}


	bool SaveNodes(const std::string& filename)
	{
		std::ofstream file("scriptfiles/" + filename);

		if (!file.good())
		{
			return false;
		}

		for (const auto node : Container::Nodes::GetAll())
		{
			if (!node.second->isSetForDeletion())
			{
				file << 0 << " " << node.second->getX() << " " << node.second->getY() << " " << node.second->getZ() << " " << -1 << " " << node.second->getID() << " " << static_cast<int>(node.second->getType()) << " " << node.second->getFlags() << '\n';
			}
		}

		for (const auto connection : Container::Connections::GetAll())
		{
			file << 1 << " " << connection.second->getSource()->getID() << " " << connection.second->getTarget()->getID() << " " << 0 << " " << connection.second->getRoadWidth() << " " << connection.second->getLeftLanes() << " " << connection.second->getRightLanes() << " " << (connection.second->isOneWay() ? 1 : 0) << '\n';
		}

		file.close();
		return true;
	}
};
