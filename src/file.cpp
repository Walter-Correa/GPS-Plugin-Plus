#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <locale>

#include "file.h"
#include "container.h"
#include "connection.h"
#include "node.h"


namespace File
{
	bool IsSafeFilePath(const std::string& path)
	{
		if (path.empty() || path.length() > 255)
		{
			return false;
		}
		if (path.find(':') != std::string::npos)
		{
			return false;
		}
		if (path.front() == '/' || path.front() == '\\')
		{
			return false;
		}
		if (path.find("..") != std::string::npos)
		{
			return false;
		}
		if (path.find('\\') != std::string::npos)
		{
			return false;
		}
		for (char c : path)
		{
			if (static_cast<unsigned char>(c) < 32 || c == '*' || c == '?' || c == '"' || c == '<' || c == '>' || c == '|')
			{
				return false;
			}
		}
		return true;
	}

	bool LoadNodes(const std::string& filename, int& line_count, int& node_count, int& connection_count)
	{
		if (!IsSafeFilePath(filename))
		{
			return false;
		}

		std::ifstream file("scriptfiles/" + filename);
		if (!file.is_open())
		{
			file.clear();
			file.open(filename);
		}

		if (!file.good())
		{
			return false;
		}

		file.imbue(std::locale::classic());

		Container::GraphWriterLock lock(Container::GetGraphMutex());

		std::string buffer;

		while (std::getline(file, buffer))
		{
			if (buffer.empty()) continue;
			std::istringstream input(buffer);

			int type = -1;
			if (!(input >> type)) continue;

			if (type == 0)
			{
				float x = 0.0f, y = 0.0f, z = 0.0f;
				int ignore = 0, id = -1;
				if (!(input >> x >> y >> z >> ignore >> id)) continue;
				if (id < 0 || id > 1000000) continue;

				int node_type = 0;
				uint32_t node_flags = 0u;

				if (input >> node_type)
				{
					input >> node_flags;
				}

				if (Container::Nodes::Add(id, x, y, z, static_cast<PathType>(node_type), node_flags))
				{
					node_count++;
				}
			}
			else if (type == 1)
			{
				int id = -1, id2 = -1, direction = 0;
				if (!(input >> id >> id2 >> direction)) continue;
				if (id < 0 || id > 1000000 || id2 < 0 || id2 > 1000000) continue;

				float road_width = 8.0f;
				int left_lanes = 1;
				int right_lanes = 1;
				int oneway = 0;

				if (input >> road_width)
				{
					input >> left_lanes >> right_lanes >> oneway;
				}

				if (direction != 2 && Container::Connections::Add(id, id2, road_width, left_lanes, right_lanes, oneway != 0))
				{
					connection_count++;
				}
			}

			line_count++;
		}

		file.close();
		Container::Nodes::RebuildTypeIndices();
		return true;
	}


	bool SaveNodes(const std::string& filename)
	{
		if (!IsSafeFilePath(filename))
		{
			return false;
		}

		std::ofstream file("scriptfiles/" + filename);
		if (!file.is_open())
		{
			file.clear();
			file.open(filename);
		}

		if (!file.good())
		{
			return false;
		}

		file.imbue(std::locale::classic());
		file << std::setprecision(9);

		std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());

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
