#include <map>
#include <cmath>
#include <thread>
#include <shared_mutex>
#include <mutex>
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include <algorithm>
#include <charconv>
#include <memory>
#include <chrono>

#include "natives.h"
#include "callback.h"
#include "connection.h"
#include "node.h"
#include "path.h"
#include "file.h"
#include "amx_handler.h"
#include "zones.h"
#include "tunnels.h"
#include "radars.h"
#include "worker_pool.h"
#include "pawn_utils.h"


namespace Natives
{
	std::mt19937 mersenne{ std::random_device{}() };

	using PawnUtils::WriteRef;

	inline bool IsValidPathType(int type, bool allow_any = true)
	{
		if (allow_any)
		{
			return type >= PATH_TYPE_ANY && type <= PATH_TYPE_BOAT;
		}
		return type >= PATH_TYPE_VEHICLE && type <= PATH_TYPE_BOAT;
	}

	inline bool IsValidRoutePref(int pref)
	{
		return pref >= ROUTE_PREF_SHORTEST && pref <= ROUTE_PREF_PAVED_ONLY;
	}

	inline bool IsValidVehicleCat(int cat)
	{
		return cat >= VEHICLE_CAT_STANDARD && cat <= VEHICLE_CAT_MOTORCYCLE;
	}

	inline bool IsValidRoadClass(int cls)
	{
		return cls >= ROAD_CLASS_ALLEY && cls <= ROAD_CLASS_HIGHWAY;
	}

	inline float GetAngleDelta(float a, float b)
	{
		float diff = std::abs(a - b);
		while (diff > 180.0f) diff = std::abs(diff - 360.0f);
		return diff;
	}

	struct LegalRoadInfo
	{
		float ux;
		float uy;
		float angle;
		int side_lanes;
		float edge_offset;
		float max_offroad_dist;
		bool in_median;
		bool is_vehicle;
	};

	bool ComputeLegalRoadInfo(const Connection* conn, float x, float y, LegalRoadInfo& out)
	{
		if (conn == nullptr) return false;
		const Node* src = conn->getSource();
		const Node* tgt = conn->getTarget();
		if (src == nullptr || tgt == nullptr) return false;

		const float ax = src->getX(), ay = src->getY();
		const float bx = tgt->getX(), by = tgt->getY();
		const float vx = bx - ax, vy = by - ay;
		const float len = std::sqrt(vx * vx + vy * vy);
		if (len <= 0.0001f) return false;

		const float ux = vx / len;
		const float uy = vy / len;
		const float forward_angle = src->getAngleFromNode(const_cast<Node*>(tgt), true);
		const float reverse_angle = const_cast<Node*>(tgt)->getAngleFromNode(const_cast<Node*>(src), true);

		out.is_vehicle = (src->getType() == PATH_TYPE_VEHICLE);
		out.in_median = false;

		if (conn->isOneWay())
		{
			const bool legal_fwd = !conn->isWrongWay();
			out.ux = legal_fwd ? ux : -ux;
			out.uy = legal_fwd ? uy : -uy;
			out.angle = legal_fwd ? forward_angle : reverse_angle;
			out.side_lanes = std::max(1, conn->getTravelLanes());
			out.edge_offset = out.is_vehicle ? (0.5f * static_cast<float>(out.side_lanes) * GTA_LANE_WIDTH) : (0.5f * conn->getRoadWidth());
		}
		else
		{
			const float cross = vx * (y - ay) - vy * (x - ax);
			const float perp_dist = std::abs(cross) / len;
			const float gap = conn->getCenterGap();
			out.in_median = (gap > 0.0f && perp_dist < gap);

			if (cross <= 0.0f)
			{
				out.ux = ux;
				out.uy = uy;
				out.angle = forward_angle;
				out.side_lanes = std::max(1, conn->getRightLanes());
			}
			else
			{
				out.ux = -ux;
				out.uy = -uy;
				out.angle = reverse_angle;
				out.side_lanes = std::max(1, conn->getLeftLanes());
			}
			out.edge_offset = out.is_vehicle ? (gap + static_cast<float>(out.side_lanes) * GTA_LANE_WIDTH) : (0.5f * conn->getRoadWidth());
		}

		out.max_offroad_dist = out.edge_offset + 4.0f;
		return true;
	}

	struct LaneSeg
	{
		float sx, sy, sz;
		float ex, ey, ez;
		float ux, uy;
		float d;
		float raw_sx, raw_sy, raw_ex, raw_ey;
		float raw_len;
		const Connection* conn;
	};

	static bool BuildPathLaneSegments(const Path* path, int build_start, std::vector<LaneSeg>& out_segs, float max_len_m = -1.0f, bool* out_reaches_end = nullptr)
	{
		if (path == nullptr) return false;
		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());
		if (count < 2) return false;

		const int num_segs = count - 1;
		build_start = std::clamp(build_start, 0, num_segs - 1);

		out_segs.clear();
		const int remaining_segs = num_segs - build_start;
		out_segs.reserve((max_len_m > 0.0f) ? std::min(remaining_segs, 32) : remaining_segs);

		float accum_len = 0.0f;
		int built_end = build_start;
		for (int k = build_start; k < num_segs; ++k)
		{
			Node* a = nodes[k];
			Node* b = nodes[k + 1];
			LaneSeg s{};
			s.sz = a->getZ();
			s.ez = b->getZ();

			const float vx = b->getX() - a->getX();
			const float vy = b->getY() - a->getY();
			const float len = std::sqrt(vx * vx + vy * vy);
			s.raw_len = len;
			if (len > 0.0001f)
			{
				s.ux = vx / len;
				s.uy = vy / len;
			}

			Connection* conn = nullptr;
			for (auto c : a->getConnections())
			{
				if (c != nullptr && c->getSource() == a && c->getTarget() == b)
				{
					conn = c;
					break;
				}
			}

			s.d = (conn != nullptr) ? conn->getLaneOffset(conn->getTravelLanes() - 1) : 0.0f;
			s.conn = conn;
			const float rx = s.uy;
			const float ry = -s.ux;
			s.raw_sx = a->getX() + rx * s.d;
			s.raw_sy = a->getY() + ry * s.d;
			s.raw_ex = b->getX() + rx * s.d;
			s.raw_ey = b->getY() + ry * s.d;
			s.sx = s.raw_sx;
			s.sy = s.raw_sy;
			s.ex = s.raw_ex;
			s.ey = s.raw_ey;
			out_segs.push_back(s);

			accum_len += len;
			built_end = k + 1;
			if (max_len_m > 0.0f && static_cast<int>(out_segs.size()) >= 8 && accum_len > max_len_m)
			{
				break;
			}
		}

		const int built_count = static_cast<int>(out_segs.size());
		constexpr float kCos10Deg = 0.984807753f;
		for (int i = 0; i < built_count - 1; ++i)
		{
			LaneSeg& s1 = out_segs[i];
			LaneSeg& s2 = out_segs[i + 1];
			if (s1.raw_len <= 0.0001f || s2.raw_len <= 0.0001f)
			{
				continue;
			}

			const float cos_ang = std::clamp(s1.ux * s2.ux + s1.uy * s2.uy, -1.0f, 1.0f);
			if (cos_ang > kCos10Deg)
			{
				const float jx = 0.5f * (s1.raw_ex + s2.raw_sx);
				const float jy = 0.5f * (s1.raw_ey + s2.raw_sy);
				s1.ex = jx;
				s1.ey = jy;
				s2.sx = jx;
				s2.sy = jy;
				continue;
			}

			const float cross_u = s1.ux * s2.uy - s1.uy * s2.ux;
			if (std::abs(cross_u) > 0.0001f)
			{
				const float dx = s2.raw_sx - s1.raw_sx;
				const float dy = s2.raw_sy - s1.raw_sy;
				const float t1 = (dx * s2.uy - dy * s2.ux) / cross_u;
				const float mx = s1.raw_sx + t1 * s1.ux;
				const float my = s1.raw_sy + t1 * s1.uy;

				const Node* joint_node = nodes[build_start + i + 1];
				const float miter_dist = std::hypot(mx - joint_node->getX(), my - joint_node->getY());
				const float max_d = std::max(std::abs(s1.d), std::abs(s2.d));
				const float miter_limit = 2.5f * max_d;

				const bool valid_s1 = ((mx - s1.sx) * s1.ux + (my - s1.sy) * s1.uy) > 0.05f;
				const bool valid_s2 = ((s2.raw_ex - mx) * s2.ux + (s2.raw_ey - my) * s2.uy) > 0.05f;

				if (miter_dist <= miter_limit + 0.001f && valid_s1 && valid_s2)
				{
					s1.ex = mx;
					s1.ey = my;
					s2.sx = mx;
					s2.sy = my;
				}
			}
		}

		const bool reaches_end = (built_end == num_segs);
		if (out_reaches_end != nullptr)
		{
			*out_reaches_end = reaches_end;
		}

		if (reaches_end && path->hasStopPoint() && !out_segs.empty())
		{
			LaneSeg& last = out_segs.back();
			float stop_x = 0.0f, stop_y = 0.0f, stop_z = 0.0f, seg_dist = 0.0f;
			path->getStopPoint(stop_x, stop_y, stop_z, seg_dist);
			const float forward_proj = (stop_x - last.sx) * last.ux + (stop_y - last.sy) * last.uy;
			if (forward_proj > 0.05f)
			{
				last.ex = stop_x;
				last.ey = stop_y;
			}
			else
			{
				last.ex = last.sx + last.ux * 0.05f;
				last.ey = last.sy + last.uy * 0.05f;
			}
			last.ez = stop_z;
		}

		return true;
	}

	static void ProjectOntoLaneSegments(const std::vector<LaneSeg>& segs, int build_start, float px, float py, int min_seg, int max_seg, int& out_best_idx, float& out_best_t, float& out_best_qx, float& out_best_qy, float& out_lat_err)
	{
		const int built_count = static_cast<int>(segs.size());
		const int clamped_min = std::clamp(min_seg - build_start, 0, built_count - 1);
		const int clamped_max = std::clamp(max_seg - build_start, clamped_min, built_count - 1);

		out_best_idx = clamped_min;
		out_best_t = 0.0f;
		out_best_qx = segs[clamped_min].sx;
		out_best_qy = segs[clamped_min].sy;
		float min_d_sq = std::numeric_limits<float>::infinity();

		for (int idx = clamped_min; idx <= clamped_max; ++idx)
		{
			const LaneSeg& s = segs[idx];
			const float abx = s.ex - s.sx;
			const float aby = s.ey - s.sy;
			const float ab_len_sq = abx * abx + aby * aby;

			float t = 0.0f;
			if (ab_len_sq > 0.0001f)
			{
				t = std::clamp(((px - s.sx) * abx + (py - s.sy) * aby) / ab_len_sq, 0.0f, 1.0f);
			}

			const float qx = s.sx + t * abx;
			const float qy = s.sy + t * aby;
			const float d_sq = (px - qx) * (px - qx) + (py - qy) * (py - qy);
			if (d_sq < min_d_sq)
			{
				min_d_sq = d_sq;
				out_best_idx = idx;
				out_best_t = t;
				out_best_qx = qx;
				out_best_qy = qy;
			}
		}

		const LaneSeg& bs = segs[out_best_idx];
		if (bs.ux != 0.0f || bs.uy != 0.0f)
		{
			out_lat_err = (px - out_best_qx) * bs.uy - (py - out_best_qy) * bs.ux;
		}
		else
		{
			out_lat_err = 0.0f;
		}
	}

	static void WalkLanePolylineForward(const std::vector<LaneSeg>& segs, int best_idx, float best_t, float target_dist, float& out_x, float& out_y, float& out_z, float& out_covered)
	{
		const int built_count = static_cast<int>(segs.size());
		out_x = segs.back().ex;
		out_y = segs.back().ey;
		out_z = segs.back().ez;

		const float requested = std::max(0.0f, target_dist);
		float remain = requested;
		for (int idx = best_idx; idx < built_count; ++idx)
		{
			const LaneSeg& s = segs[idx];
			const float start_t = (idx == best_idx) ? best_t : 0.0f;
			const float ax = s.sx + start_t * (s.ex - s.sx);
			const float ay = s.sy + start_t * (s.ey - s.sy);
			const float az = s.sz + start_t * (s.ez - s.sz);
			const float seg_len = std::hypot(s.ex - ax, s.ey - ay);

			if (remain <= seg_len)
			{
				const float frac = (seg_len > 0.0001f) ? (remain / seg_len) : 0.0f;
				out_x = ax + frac * (s.ex - ax);
				out_y = ay + frac * (s.ey - ay);
				out_z = az + frac * (s.ez - az);
				out_covered = requested;
				return;
			}
			remain -= seg_len;

			if (idx + 1 < built_count)
			{
				const LaneSeg& next_s = segs[idx + 1];
				const float gap_dx = next_s.sx - s.ex;
				const float gap_dy = next_s.sy - s.ey;
				const float gap_len = std::hypot(gap_dx, gap_dy);
				if (gap_len > 0.001f)
				{
					if (remain <= gap_len)
					{
						const float frac = remain / gap_len;
						out_x = s.ex + frac * gap_dx;
						out_y = s.ey + frac * gap_dy;
						out_z = s.ez + frac * (next_s.sz - s.ez);
						out_covered = requested;
						return;
					}
					remain -= gap_len;
				}
			}
		}

		out_covered = requested - remain;
	}

	bool ComputePathLaneTarget(const Path* path, float px, float py, float lookahead, int min_seg, int max_seg, int& out_best_seg, float& out_tx, float& out_ty, float& out_tz, float& out_lat_err)
	{
		if (path == nullptr) return false;
		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());
		if (count <= 0) return false;

		if (count == 1)
		{
			out_best_seg = 0;
			out_tx = nodes[0]->getX();
			out_ty = nodes[0]->getY();
			out_tz = nodes[0]->getZ();
			out_lat_err = 0.0f;
			return true;
		}

		const int num_segs = count - 1;
		min_seg = std::clamp(min_seg, 0, num_segs - 1);
		max_seg = std::clamp(max_seg, min_seg, num_segs - 1);
		const int build_start = std::max(0, min_seg - 1);

		std::vector<LaneSeg> segs;
		if (!BuildPathLaneSegments(path, build_start, segs, std::max(0.0f, lookahead) + 60.0f))
		{
			return false;
		}

		int best_idx = 0;
		float best_t = 0.0f, best_qx = 0.0f, best_qy = 0.0f;
		ProjectOntoLaneSegments(segs, build_start, px, py, min_seg, max_seg, best_idx, best_t, best_qx, best_qy, out_lat_err);
		out_best_seg = build_start + best_idx;

		float covered = 0.0f;
		WalkLanePolylineForward(segs, best_idx, best_t, lookahead, out_tx, out_ty, out_tz, covered);
		return true;
	}


	cell AMX_NATIVE_CALL CreateMapNode(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(4);

		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		auto type = PATH_TYPE_VEHICLE;
		auto flags = NODE_FLAG_NONE;

		if (params[0] >= (5 * 4))
		{
			if (!IsValidPathType(params[5], false))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[5]);
		}

		if (params[0] >= (6 * 4))
		{
			flags = static_cast<PathNodeFlags>(params[6]);
		}

		int id;
		{
			Container::GraphWriterLock lock(Container::GetGraphMutex());
			id = Container::Nodes::AllocateID();
			Container::Nodes::Add(id, x, y, z, type, flags);
		}

		if (!WriteRef(amx, params[4], id))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL DestroyMapNode(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		{
			Container::GraphWriterLock lock(Container::GetGraphMutex());
			Container::Nodes::SetForDeletion(node);
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsValidMapNode(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);

		return Container::Nodes::Find(id) != nullptr;
	}


	cell AMX_NATIVE_CALL GetMapNodePos(AMX* amx, cell* params)
	{
		CHECK_PARAMS(4);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		auto x = node->getX(), y = node->getY(), z = node->getZ();

		if (!WriteRef(amx, params[2], x) ||
		    !WriteRef(amx, params[3], y) ||
		    !WriteRef(amx, params[4], z))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeType(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		return static_cast<cell>(node->getType());
	}


	cell AMX_NATIVE_CALL GetMapNodeFlags(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[2], node->getFlags()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL CreateConnection(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		const auto source_id = static_cast<int>(params[1]);
		const auto target_id = static_cast<int>(params[2]);

		auto road_width = 8.0f;
		auto left_lanes = 1;
		auto right_lanes = 1;
		auto one_way = false;

		if (params[0] >= (4 * 4))
		{
			road_width = amx_ctof(params[4]);
		}

		if (params[0] >= (5 * 4))
		{
			left_lanes = static_cast<int>(params[5]);
		}

		if (params[0] >= (6 * 4))
		{
			right_lanes = static_cast<int>(params[6]);
		}

		if (params[0] >= (7 * 4))
		{
			one_way = params[7] != 0;
		}

		int id;
		{
			Container::GraphWriterLock lock(Container::GetGraphMutex());
			id = Container::Connections::Add(source_id, target_id, road_width, left_lanes, right_lanes, one_way);
		}

		if (id == -1)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[3], id))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL DestroyConnection(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		{
			Container::GraphWriterLock lock(Container::GetGraphMutex());
			Container::Connections::Delete(connection);
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionSource(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		if (!WriteRef(amx, params[2], connection->getSource()->getID()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionTarget(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		if (!WriteRef(amx, params[2], connection->getTarget()->getID()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionLaneInfo(AMX* amx, cell* params)
	{
		CHECK_PARAMS(4);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		auto road_width = connection->getRoadWidth();

		if (!WriteRef(amx, params[2], road_width) ||
		    !WriteRef(amx, params[3], connection->getLeftLanes()) ||
		    !WriteRef(amx, params[4], connection->getRightLanes()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionLanePos(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		const auto lane_index = static_cast<int>(params[2]);
		float x = 0.0f, y = 0.0f, z = 0.0f;

		if (!connection->getLanePosition(lane_index, x, y, z))
		{
			return GPS_ERROR_INTERNAL;
		}

		if (!WriteRef(amx, params[3], x) ||
		    !WriteRef(amx, params[4], y) ||
		    !WriteRef(amx, params[5], z))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsConnectionOneWay(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return false;
		}

		return connection->isOneWay() ? 1 : 0;
	}


	cell AMX_NATIVE_CALL IsConnectionWrongWay(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return false;
		}

		return connection->isWrongWay() ? 1 : 0;
	}


	cell AMX_NATIVE_CALL GetHighestMapNodeID(AMX* amx, cell* params)
	{
		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (1 * 4))
		{
			if (!IsValidPathType(params[1], true))
			{
				return INVALID_NODE_ID;
			}
			type = static_cast<PathType>(params[1]);
		}

		return Container::Nodes::GetHighestID(type);
	}


	cell AMX_NATIVE_CALL GetRandomMapNode(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(1);

		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (2 * 4))
		{
			if (!IsValidPathType(params[2], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[2]);
		}

		float min_z = -99999.0f;
		float max_z = 99999.0f;

		if (params[0] >= (3 * 4))
		{
			min_z = amx_ctof(params[3]);
		}

		if (params[0] >= (4 * 4))
		{
			max_z = amx_ctof(params[4]);
		}

		if (min_z <= -90000.0f && max_z >= 90000.0f)
		{
			if (type == PATH_TYPE_VEHICLE || type == PATH_TYPE_PEDESTRIAN)
			{
				min_z = 1.5f;
				max_z = 500.0f;
			}
			else if (type == PATH_TYPE_BOAT)
			{
				min_z = -10.0f;
				max_z = 20.0f;
			}
		}

		Node* node = Container::Nodes::GetRandomNode(type, min_z, max_z);
		if (node == nullptr)
		{
			WriteRef(amx, params[1], INVALID_NODE_ID);
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[1], node->getID()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetRandomMapNodeFromArea(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(5);

		const float min_x = std::min(amx_ctof(params[1]), amx_ctof(params[3]));
		const float max_x = std::max(amx_ctof(params[1]), amx_ctof(params[3]));
		const float min_y = std::min(amx_ctof(params[2]), amx_ctof(params[4]));
		const float max_y = std::max(amx_ctof(params[2]), amx_ctof(params[4]));

		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (6 * 4))
		{
			if (!IsValidPathType(params[6], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[6]);
		}

		float min_z = -99999.0f;
		float max_z = 99999.0f;

		if (params[0] >= (7 * 4))
		{
			min_z = amx_ctof(params[7]);
		}

		if (params[0] >= (8 * 4))
		{
			max_z = amx_ctof(params[8]);
		}

		if (min_z <= -90000.0f && max_z >= 90000.0f)
		{
			if (type == PATH_TYPE_VEHICLE || type == PATH_TYPE_PEDESTRIAN)
			{
				min_z = 1.5f;
				max_z = 500.0f;
			}
			else if (type == PATH_TYPE_BOAT)
			{
				min_z = -10.0f;
				max_z = 20.0f;
			}
		}

		Node* node = Container::Nodes::GetRandomNodeInArea(min_x, min_y, max_x, max_y, type, min_z, max_z);
		if (node == nullptr)
		{
			WriteRef(amx, params[5], INVALID_NODE_ID);
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[5], node->getID()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeConnectionCount(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[2], node->getConnections().size()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeConnection(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto index = static_cast<unsigned int>(params[2]);
		const auto& connections = node->getConnections();

		if (connections.size() <= index)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		if (!WriteRef(amx, params[3], connections[index]->getId()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionBetweenMapNodes(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto source_id = static_cast<int>(params[1]);
		const auto source = Container::Nodes::Find(source_id);

		if (source == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}
		
		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);

		if (target == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		for (const auto connection : source->getConnections())
		{
			if (connection->getTarget() == target)
			{
				if (!WriteRef(amx, params[3], connection->getId()))
				{
					return GPS_ERROR_INVALID_PARAMS;
				}

				return GPS_ERROR_NONE;
			}
		}

		return GPS_ERROR_INVALID_CONNECTION;
	}


	cell AMX_NATIVE_CALL GetDistanceBetweenMapNodes(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto first_id = static_cast<int>(params[1]);
		const auto first = Container::Nodes::Find(first_id);

		if (first == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto second_id = static_cast<int>(params[2]);
		const auto second = Container::Nodes::Find(second_id);

		if (second == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		auto distance = first->getDistanceFromNode(second);

		if (!WriteRef(amx, params[3], distance))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetAngleBetweenMapNodes(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		const auto first_id = static_cast<int>(params[1]);
		const auto first = Container::Nodes::Find(first_id);

		if (first == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto second_id = static_cast<int>(params[2]);
		const auto second = Container::Nodes::Find(second_id);

		if (second == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const bool facing_angle = (params[0] >= (4 * 4)) ? (params[4] != 0) : false;

		const auto angle = first->getAngleFromNode(second, facing_angle);

		if (!WriteRef(amx, params[3], angle))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeDistanceFromPoint(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto x = amx_ctof(params[2]), y = amx_ctof(params[3]), z = amx_ctof(params[4]);

		auto distance = node->getDistanceFromPoint(x, y, z);

		if (!WriteRef(amx, params[5], distance))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeAngleFromPoint(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(4);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto x = amx_ctof(params[2]), y = amx_ctof(params[3]);
		const bool facing_angle = (params[0] >= (5 * 4)) ? (params[5] != 0) : false;

		const auto angle = node->getAngleFromPoint(x, y, facing_angle);

		if (!WriteRef(amx, params[4], angle))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	static cell FindClosestNodeInternal(AMX* amx, cell* params, PathType default_type)
	{
		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		auto ignored = INVALID_NODE_ID;
		auto type = default_type;

		if (params[0] >= (5 * 4))
		{
			ignored = static_cast<int>(params[5]);
		}

		if (params[0] >= (6 * 4))
		{
			if (!IsValidPathType(params[6], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[6]);
		}

		int result = INVALID_NODE_ID;
		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			const auto node = Container::Nodes::FindClosest(x, y, z, type, ignored);
			result = (node != nullptr) ? node->getID() : INVALID_NODE_ID;
		}

		if (result == INVALID_NODE_ID)
		{
			WriteRef(amx, params[4], INVALID_NODE_ID);
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[4], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetClosestMapNodeToPoint(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(4);

		return FindClosestNodeInternal(amx, params, PATH_TYPE_VEHICLE);
	}


	cell AMX_NATIVE_CALL GetClosestMapNodeToPointAngle(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(5);

		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		const auto angle = amx_ctof(params[4]);
		float max_dist = 20.0f;
		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (6 * 4))
		{
			max_dist = amx_ctof(params[6]);
		}

		if (params[0] >= (7 * 4))
		{
			if (!IsValidPathType(params[7], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[7]);
		}

		int result = INVALID_NODE_ID;
		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			const auto node = Container::Nodes::FindClosestWithAngle(x, y, z, angle, max_dist, type);
			result = (node != nullptr) ? node->getID() : INVALID_NODE_ID;
		}

		if (result == INVALID_NODE_ID)
		{
			WriteRef(amx, params[5], INVALID_NODE_ID);
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[5], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SaveMapNodesToFile(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto filename = amx_GetCppString(amx, params[1]);
		if (!File::IsSafeFilePath(filename))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		if (File::SaveNodes(filename))
		{
			return GPS_ERROR_NONE;
		}

		return GPS_ERROR_INTERNAL;
	}


	static cell FindPathInternalWrapper(AMX* amx, cell* params, PathType default_type)
	{
		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);

		if (start == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);

		if (target == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		auto type = default_type;

		if (params[0] >= (4 * 4))
		{
			if (!IsValidPathType(params[4], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[4]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (5 * 4))
		{
			if (!IsValidRoutePref(params[5]))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			pref = static_cast<RoutePreference>(params[5]);
		}

		const auto result = Pathfinder::FindPath(start, target, type, pref, VEHICLE_CAT_STANDARD, amx);

		if (result == -1)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		if (!WriteRef(amx, params[3], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindPath(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		return FindPathInternalWrapper(amx, params, PATH_TYPE_VEHICLE);
	}


	cell AMX_NATIVE_CALL FindMultiStopPath(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		const auto count = static_cast<int>(params[2]);
		if (count < 2 || count > 64)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		cell* stops_addr = PawnUtils::GetArray(amx, params[1], count);
		if (stops_addr == nullptr)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		bool optimize_order = false;
		if (params[0] >= (4 * 4))
		{
			optimize_order = params[4] != 0;
		}

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (5 * 4))
		{
			if (!IsValidPathType(params[5], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[5]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (6 * 4))
		{
			if (!IsValidRoutePref(params[6]))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			pref = static_cast<RoutePreference>(params[6]);
		}

		std::vector<Node*> stop_nodes;
		stop_nodes.reserve(count);

		for (int i = 0; i < count; ++i)
		{
			const int node_id = static_cast<int>(stops_addr[i]);
			Node* node = Container::Nodes::Find(node_id);
			if (node == nullptr || node->isBlocked() || node->isSetForDeletion())
			{
				return GPS_ERROR_INVALID_NODE;
			}
			stop_nodes.push_back(node);
		}

		const int path_id = Pathfinder::FindMultiStopPath(stop_nodes, optimize_order, type, pref, amx);
		if (path_id == INVALID_PATH_ID)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		if (!WriteRef(amx, params[3], path_id))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	static cell FindPathThreadedInternal(AMX* amx, cell* params, PathType type, RoutePreference pref, int format_param_index)
	{
		const auto start_id = static_cast<int>(params[1]);
		const auto target_id = static_cast<int>(params[2]);

		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			auto start = Container::Nodes::Find(start_id);
			if (start == nullptr)
			{
				return GPS_ERROR_INVALID_NODE;
			}

			auto target = Container::Nodes::Find(target_id);
			if (target == nullptr)
			{
				return GPS_ERROR_INVALID_NODE;
			}
		}

		const auto callback_name = amx_GetCppString(amx, params[format_param_index - 1]);

		char* format = nullptr;
		if (params[0] >= static_cast<cell>(format_param_index * sizeof(cell)))
		{
			amx_StrParam(amx, params[format_param_index], format);
		}

		auto amx_obj = AmxHandler::GetAmx(amx);
		if (amx_obj == nullptr)
		{
			return GPS_ERROR_INTERNAL;
		}

		const uint64_t serial = amx_obj->getLoadSerial();
		auto callback = new Callback(amx, serial, callback_name, format, params, format_param_index);
		if (!callback->isValid())
		{
			delete callback;
			return GPS_ERROR_INVALID_PARAMS;
		}

		PathJob job;
		job.start_id = start_id;
		job.target_id = target_id;
		job.type = type;
		job.pref = pref;
		job.cat = VEHICLE_CAT_STANDARD;
		job.amx = amx;
		job.amx_serial = serial;
		job.callback = callback;
		job.yield_count = 0;
		job.first_attempt_time = std::chrono::steady_clock::time_point{};

		if (!WorkerPool::EnqueueJob(std::move(job)))
		{
			delete callback;
			return GPS_ERROR_INTERNAL;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindPathThreaded(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		return FindPathThreadedInternal(amx, params, PATH_TYPE_VEHICLE, ROUTE_PREF_SHORTEST, 4);
	}


	cell AMX_NATIVE_CALL FindPathThreadedEx(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(5);

		if (!IsValidPathType(params[3], true))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}
		const auto type = static_cast<PathType>(params[3]);

		if (!IsValidRoutePref(params[4]))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}
		const auto pref = static_cast<RoutePreference>(params[4]);

		return FindPathThreadedInternal(amx, params, type, pref, 6);
	}


	cell AMX_NATIVE_CALL IsValidPath(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		return path != nullptr;
	}


	cell AMX_NATIVE_CALL GetPathSize(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		if (!WriteRef(amx, params[2], path->getNodes().size()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathLength(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		auto result = path->getLength();

		if (!WriteRef(amx, params[2], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathNode(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto index = static_cast<unsigned int>(params[2]);
		const auto& nodes = path->getNodes();

		if (index >= nodes.size())
		{
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[3], nodes[index]->getID()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathNodeIndex(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto node_id = static_cast<int>(params[2]);
		const auto node = Container::Nodes::Find(node_id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto index = node->getPositionInPath(path);

		if (index == -1)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[3], index))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathLaneNodePos(AMX* amx, cell* params)
	{
		CHECK_PARAMS(6);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto index = static_cast<unsigned int>(params[2]);
		const auto lane_index = static_cast<int>(params[3]);
		const auto& nodes = path->getNodes();

		if (index >= nodes.size())
		{
			return GPS_ERROR_INVALID_NODE;
		}

		auto current_node = nodes[index];
		Connection* connection = nullptr;

		if (index + 1 < nodes.size())
		{
			auto next_node = nodes[index + 1];
			for (auto conn : current_node->getConnections())
			{
				if (conn->getTarget() == next_node)
				{
					connection = conn;
					break;
				}
			}
		}
		else if (index > 0)
		{
			auto prev_node = nodes[index - 1];
			for (auto conn : prev_node->getConnections())
			{
				if (conn->getTarget() == current_node)
				{
					connection = conn;
					break;
				}
			}
		}

		float x = current_node->getX(), y = current_node->getY(), z = current_node->getZ();

		if (connection != nullptr)
		{
			connection->getLanePosition(lane_index, x, y, z);
		}

		if (!WriteRef(amx, params[4], x) ||
		    !WriteRef(amx, params[5], y) ||
		    !WriteRef(amx, params[6], z))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL DestroyPath(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		Container::Paths::Delete(path);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetMapNodeBlocked(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto blocked = params[2] != 0;

		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		node->setBlocked(blocked);
		Container::Nodes::MarkComponentsDirty();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsMapNodeBlocked(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);

		const auto node = Container::Nodes::Find(id);

		if (node == nullptr)
		{
			return 0;
		}

		return node->isBlocked() ? 1 : 0;
	}


	cell AMX_NATIVE_CALL SetConnectionBlocked(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto id = static_cast<int>(params[1]);
		const auto blocked = params[2] != 0;

		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return GPS_ERROR_INVALID_CONNECTION;
		}

		connection->setBlocked(blocked);
		Container::Nodes::MarkComponentsDirty();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsConnectionBlocked(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);

		const auto id = static_cast<int>(params[1]);

		const auto connection = Container::Connections::Find(id);

		if (connection == nullptr)
		{
			return 0;
		}

		return connection->isBlocked() ? 1 : 0;
	}


	cell AMX_NATIVE_CALL GetClosestPointOnRoad(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(7);

		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (9 * 4))
		{
			if (!IsValidPathType(params[9], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[9]);
		}

		float out_x = 0.0f, out_y = 0.0f, out_z = 0.0f, out_angle = 0.0f, out_road_width = 8.0f;
		int out_conn_id = INVALID_CONNECTION_ID;
		bool success = false;
		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			success = Container::Connections::FindClosestRoadPoint(x, y, z, out_x, out_y, out_z, out_angle, out_road_width, out_conn_id, type);
		}

		if (!success)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[4], out_x) ||
		    !WriteRef(amx, params[5], out_y) ||
		    !WriteRef(amx, params[6], out_z) ||
		    !WriteRef(amx, params[7], out_angle))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		if (params[0] >= (8 * 4))
		{
			if (!WriteRef(amx, params[8], out_conn_id))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetRoadPositionInfo(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(8);

		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		int curb = 0;
		auto type = PATH_TYPE_VEHICLE;

		if (params[0] >= (9 * 4))
		{
			curb = static_cast<int>(params[9]);
			if (curb < 0 || curb > 2)
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
		}

		if (params[0] >= (10 * 4))
		{
			if (!IsValidPathType(params[10], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[10]);
		}

		float out_x = 0.0f, out_y = 0.0f, out_z = 0.0f, out_angle = 0.0f, out_road_width = 8.0f;
		int out_conn_id = INVALID_CONNECTION_ID;
		bool success = false;
		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			success = Container::Connections::FindClosestRoadPoint(x, y, z, out_x, out_y, out_z, out_angle, out_road_width, out_conn_id, type);
			if (success)
			{
				const auto conn = Container::Connections::Find(out_conn_id);
				LegalRoadInfo road_info{};
				if (ComputeLegalRoadInfo(conn, x, y, road_info))
				{
					out_angle = road_info.angle;
					out_road_width = conn->getRoadWidth();
					if (curb != 0)
					{
						const float offset = (curb == 1) ? std::max(0.0f, road_info.edge_offset - 1.3f) : (road_info.edge_offset + 1.5f);
						out_x += offset * road_info.uy;
						out_y += offset * (-road_info.ux);
					}
				}
			}
		}

		if (!success)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		if (!WriteRef(amx, params[4], out_x) ||
		    !WriteRef(amx, params[5], out_y) ||
		    !WriteRef(amx, params[6], out_z) ||
		    !WriteRef(amx, params[7], out_angle) ||
		    !WriteRef(amx, params[8], out_road_width))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsPointInValidRoad(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		const float x = amx_ctof(params[1]);
		const float y = amx_ctof(params[2]);
		const float z = amx_ctof(params[3]);
		float radius = 0.0f;
		auto type = PATH_TYPE_VEHICLE;
		float max_z_diff = 4.0f;
		const bool has_conn_ref = (params[0] >= (7 * 4));

		auto write_invalid_conn = [&]()
		{
			if (has_conn_ref)
			{
				WriteRef(amx, params[7], INVALID_CONNECTION_ID);
			}
		};

		if (params[0] >= (4 * 4))
		{
			radius = amx_ctof(params[4]);
		}

		if (params[0] >= (5 * 4))
		{
			if (!IsValidPathType(params[5], true))
			{
				write_invalid_conn();
				return 0;
			}
			type = static_cast<PathType>(params[5]);
		}

		if (params[0] >= (6 * 4))
		{
			max_z_diff = amx_ctof(params[6]);
		}

		if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
		    !std::isfinite(radius) || !std::isfinite(max_z_diff) ||
		    radius < 0.0f || max_z_diff <= 0.0f)
		{
			write_invalid_conn();
			return 0;
		}

		int out_conn_id = INVALID_CONNECTION_ID;
		bool in_road = false;
		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			in_road = Container::Connections::IsPointInRoad(x, y, z, radius, type, max_z_diff, out_conn_id, has_conn_ref);
		}

		if (has_conn_ref)
		{
			WriteRef(amx, params[7], in_road ? out_conn_id : INVALID_CONNECTION_ID);
		}

		return in_road ? 1 : 0;
	}


	cell AMX_NATIVE_CALL IsPointAgainstTraffic(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);

		const auto x = amx_ctof(params[1]), y = amx_ctof(params[2]), z = amx_ctof(params[3]);
		const auto facing_angle = amx_ctof(params[4]);

		float out_x = 0.0f, out_y = 0.0f, out_z = 0.0f, out_angle = 0.0f, out_road_width = 8.0f;
		int out_conn_id = INVALID_CONNECTION_ID;
		bool wrong_way = false;

		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			const auto success = Container::Connections::FindClosestRoadPoint(x, y, z, out_x, out_y, out_z, out_angle, out_road_width, out_conn_id, PATH_TYPE_VEHICLE);

			if (!success)
			{
				return GPS_ERROR_INVALID_CONNECTION;
			}

			const auto conn = Container::Connections::Find(out_conn_id);
			LegalRoadInfo road_info{};
			if (!ComputeLegalRoadInfo(conn, x, y, road_info))
			{
				return GPS_ERROR_INVALID_CONNECTION;
			}

			const float dist_2d = std::hypot(x - out_x, y - out_y);
			if (road_info.in_median || dist_2d > road_info.max_offroad_dist)
			{
				wrong_way = false;
			}
			else
			{
				wrong_way = (GetAngleDelta(facing_angle, road_info.angle) > 90.0f);
			}
		}

		if (!WriteRef(amx, params[5], wrong_way ? 1 : 0))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathTurnDirection(AMX* amx, cell* params)
	{
		CHECK_PARAMS(4);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto index = static_cast<int>(params[2]);
		const auto& nodes = path->getNodes();
		const auto count = static_cast<int>(nodes.size());

		TurnDirection turn = TURN_NONE;
		float turn_angle = 0.0f;

		if (index > 0 && index < (count - 1))
		{
			const auto prev = nodes[index - 1];
			const auto curr = nodes[index];
			const auto next = nodes[index + 1];

			if (prev != nullptr && curr != nullptr && next != nullptr)
			{
				const float angle1 = prev->getAngleFromNode(curr, true);
				const float angle2 = curr->getAngleFromNode(next, true);

				float diff = angle2 - angle1;
				while (diff > 180.0f) diff -= 360.0f;
				while (diff <= -180.0f) diff += 360.0f;

				// In SA-MP compass facing angle, diff < 0 is turning right, diff > 0 is turning left
				turn_angle = -diff;

				const float abs_angle = std::abs(turn_angle);
				if (abs_angle >= 160.0f)
				{
					turn = TURN_UTURN;
				}
				else if (turn_angle > 120.0f)
				{
					turn = TURN_SHARP_RIGHT;
				}
				else if (turn_angle > 20.0f)
				{
					turn = TURN_RIGHT;
				}
				else if (turn_angle < -120.0f)
				{
					turn = TURN_SHARP_LEFT;
				}
				else if (turn_angle < -20.0f)
				{
					turn = TURN_LEFT;
				}
				else
				{
					turn = TURN_STRAIGHT;
				}
			}
		}

		if (!WriteRef(amx, params[3], turn) ||
		    !WriteRef(amx, params[4], turn_angle))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathEstimatedTravelTime(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(id);

		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto speed_kmh = amx_ctof(params[2]);
		if (speed_kmh <= 0.1f)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto speed_ms = speed_kmh / 3.6f;
		const auto seconds = path->getLength() / speed_ms;

		if (!WriteRef(amx, params[3], seconds))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathProgress(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(7);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const float px = amx_ctof(params[2]);
		const float py = amx_ctof(params[3]);
		const float pz = amx_ctof(params[4]);

		float threshold = 25.0f;
		if (params[0] >= (8 * 4))
		{
			threshold = amx_ctof(params[8]);
		}

		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());

		if (count == 0)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		int best_step = 0;
		float min_dist_sq = std::numeric_limits<float>::infinity();

		if (count == 1)
		{
			const float dx = px - nodes[0]->getX();
			const float dy = py - nodes[0]->getY();
			const float dz = pz - nodes[0]->getZ();
			min_dist_sq = dx * dx + dy * dy + dz * dz;
			best_step = 0;
		}
		else
		{
			for (int i = 0; i < count - 1; ++i)
			{
				const float ax = nodes[i]->getX();
				const float ay = nodes[i]->getY();
				const float az = nodes[i]->getZ();

				const float bx = nodes[i + 1]->getX();
				const float by = nodes[i + 1]->getY();
				const float bz = nodes[i + 1]->getZ();

				const float abx = bx - ax;
				const float aby = by - ay;
				const float abz = bz - az;

				const float apx = px - ax;
				const float apy = py - ay;
				const float apz = pz - az;

				const float ab_len_sq = abx * abx + aby * aby + abz * abz;
				float t = 0.0f;
				if (ab_len_sq > 0.0001f)
				{
					t = (apx * abx + apy * aby + apz * abz) / ab_len_sq;
					t = std::max(0.0f, std::min(1.0f, t));
				}

				const float qx = ax + t * abx;
				const float qy = ay + t * aby;
				const float qz = az + t * abz;

				const float d_sq = (px - qx) * (px - qx) + (py - qy) * (py - qy) + (pz - qz) * (pz - qz);
				if (d_sq < min_dist_sq)
				{
					min_dist_sq = d_sq;
					best_step = (t > 0.6f) ? (i + 1) : i;
				}
			}
		}

		const float distance_to_path = std::sqrt(min_dist_sq);
		const bool off_route = (distance_to_path > threshold);

		if (!WriteRef(amx, params[5], best_step) ||
		    !WriteRef(amx, params[6], distance_to_path) ||
		    !WriteRef(amx, params[7], off_route ? 1 : 0))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathRemainingInfo(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(4);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const auto current_step = static_cast<int>(params[2]);
		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());

		float speed_kmh = 60.0f;
		if (params[0] >= (5 * 4))
		{
			speed_kmh = amx_ctof(params[5]);
			if (speed_kmh <= 0.1f)
			{
				speed_kmh = 60.0f;
			}
		}

		float remaining_dist = 0.0f;
		if (current_step >= 0 && current_step < count - 1)
		{
			for (int i = current_step; i < count - 1; ++i)
			{
				remaining_dist += nodes[i]->getDistanceFromNode(nodes[i + 1]);
			}
		}

		const float speed_mps = (speed_kmh * 1000.0f) / 3600.0f;
		const float remaining_sec = remaining_dist / speed_mps;

		if (!WriteRef(amx, params[3], remaining_dist) ||
		    !WriteRef(amx, params[4], remaining_sec))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathNextManeuver(AMX* amx, cell* params)
	{
		CHECK_PARAMS(6);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const float px = amx_ctof(params[2]);
		const float py = amx_ctof(params[3]);
		const float pz = amx_ctof(params[4]);

		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());

		if (count < 3)
		{
			if (!WriteRef(amx, params[5], TURN_NONE) ||
			    !WriteRef(amx, params[6], 0.0f))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			return GPS_ERROR_NONE;
		}

		int current_step = 0;
		float min_dist_sq = std::numeric_limits<float>::infinity();

		for (int i = 0; i < count - 1; ++i)
		{
			const float ax = nodes[i]->getX();
			const float ay = nodes[i]->getY();
			const float az = nodes[i]->getZ();

			const float bx = nodes[i + 1]->getX();
			const float by = nodes[i + 1]->getY();
			const float bz = nodes[i + 1]->getZ();

			const float abx = bx - ax;
			const float aby = by - ay;
			const float abz = bz - az;

			const float apx = px - ax;
			const float apy = py - ay;
			const float apz = pz - az;

			const float ab_len_sq = abx * abx + aby * aby + abz * abz;
			float t = 0.0f;
			if (ab_len_sq > 0.0001f)
			{
				t = (apx * abx + apy * aby + apz * abz) / ab_len_sq;
				t = std::max(0.0f, std::min(1.0f, t));
			}

			const float qx = ax + t * abx;
			const float qy = ay + t * aby;
			const float qz = az + t * abz;

			const float d_sq = (px - qx) * (px - qx) + (py - qy) * (py - qy) + (pz - qz) * (pz - qz);
			if (d_sq < min_dist_sq)
			{
				min_dist_sq = d_sq;
				current_step = (t > 0.6f) ? (i + 1) : i;
			}
		}

		TurnDirection next_turn = TURN_NONE;
		int turn_node_idx = -1;

		for (int i = std::max(1, current_step + 1); i < count - 1; ++i)
		{
			const auto prev = nodes[i - 1];
			const auto curr = nodes[i];
			const auto next = nodes[i + 1];

			const float angle1 = prev->getAngleFromNode(curr, true);
			const float angle2 = curr->getAngleFromNode(next, true);

			float diff = angle2 - angle1;
			while (diff > 180.0f) diff -= 360.0f;
			while (diff <= -180.0f) diff += 360.0f;

			const float turn_angle = -diff;
			const float abs_angle = std::abs(turn_angle);

			if (abs_angle >= 160.0f)
			{
				next_turn = TURN_UTURN;
				turn_node_idx = i;
				break;
			}
			else if (turn_angle > 120.0f)
			{
				next_turn = TURN_SHARP_RIGHT;
				turn_node_idx = i;
				break;
			}
			else if (turn_angle > 20.0f)
			{
				next_turn = TURN_RIGHT;
				turn_node_idx = i;
				break;
			}
			else if (turn_angle < -120.0f)
			{
				next_turn = TURN_SHARP_LEFT;
				turn_node_idx = i;
				break;
			}
			else if (turn_angle < -20.0f)
			{
				next_turn = TURN_LEFT;
				turn_node_idx = i;
				break;
			}
		}

		float dist_to_turn = 0.0f;
		if (turn_node_idx != -1)
		{
			dist_to_turn = std::sqrt(min_dist_sq);
			for (int i = current_step; i < turn_node_idx; ++i)
			{
				dist_to_turn += nodes[i]->getDistanceFromNode(nodes[i + 1]);
			}
		}

		if (!WriteRef(amx, params[5], next_turn) ||
		    !WriteRef(amx, params[6], dist_to_turn))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	static void DouglasPeuckerIterative(const std::vector<Node*>& nodes, int total_count, float epsilon, std::vector<bool>& keep)
	{
		if (total_count <= 2)
		{
			return;
		}

		std::vector<std::pair<int, int>> stack;
		stack.reserve(64);
		stack.emplace_back(0, total_count - 1);

		while (!stack.empty())
		{
			const auto range = stack.back();
			stack.pop_back();

			const int first = range.first;
			const int last = range.second;

			if (last <= first + 1)
			{
				continue;
			}

			float max_dist = 0.0f;
			int index = first;

			const float ax = nodes[first]->getX();
			const float ay = nodes[first]->getY();
			const float bx = nodes[last]->getX();
			const float by = nodes[last]->getY();

			const float dx = bx - ax;
			const float dy = by - ay;
			const float line_len_sq = dx * dx + dy * dy;

			for (int i = first + 1; i < last; ++i)
			{
				float dist = 0.0f;
				const float px = nodes[i]->getX();
				const float py = nodes[i]->getY();

				if (line_len_sq < 0.0001f)
				{
					dist = nodes[i]->getDistanceFromNode(nodes[first]);
				}
				else
				{
					const float cross = std::abs((px - ax) * dy - (py - ay) * dx);
				dist = cross / std::sqrt(line_len_sq);
				}

				if (dist > max_dist)
				{
					max_dist = dist;
					index = i;
				}
			}

			if (max_dist > epsilon)
			{
				keep[index] = true;
				stack.emplace_back(index, last);
				stack.emplace_back(first, index);
			}
		}
	}


	cell AMX_NATIVE_CALL GetSimplifiedPathNodes(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(5);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const float tolerance = amx_ctof(params[2]);
		const auto max_nodes = static_cast<int>(params[4]);
		if (max_nodes <= 0)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		cell* out_array = PawnUtils::GetArray(amx, params[3], max_nodes);
		if (out_array == nullptr)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());

		if (count == 0)
		{
			if (!WriteRef(amx, params[5], 0))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			return GPS_ERROR_NONE;
		}

		std::vector<int> kept_node_ids;
		if (count <= 2 || tolerance <= 0.0f)
		{
			for (int i = 0; i < std::min(count, max_nodes); ++i)
			{
				kept_node_ids.push_back(nodes[i]->getID());
			}
		}
		else
		{
			std::vector<bool> keep(count, false);
			keep[0] = true;
			keep[count - 1] = true;

			DouglasPeuckerIterative(nodes, count, tolerance, keep);

			for (int i = 0; i < count; ++i)
			{
				if (keep[i])
				{
					kept_node_ids.push_back(nodes[i]->getID());
					if (static_cast<int>(kept_node_ids.size()) >= max_nodes)
					{
						break;
					}
				}
			}
		}

		for (size_t i = 0; i < kept_node_ids.size(); ++i)
		{
			out_array[i] = kept_node_ids[i];
		}

		if (!WriteRef(amx, params[5], kept_node_ids.size()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsMapNodeIntersection(AMX* amx, cell* params)
	{
		if (params[0] < (1 * 4))
		{
			GPSLogger::Log("%s: Expecting at least 1 parameter(s), but found %d.", __func__, (int)(params[0] / sizeof(cell)));
			return false;
		}

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);
		if (node == nullptr)
		{
			return false;
		}

		const auto conn_count = static_cast<int>(node->getConnections().size());

		if (params[0] >= (2 * 4))
		{
			WriteRef(amx, params[2], conn_count);
		}

		return (conn_count >= 3) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL GetMapNodeConnectionTarget(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(id);
		if (node == nullptr)
		{
			return GPS_ERROR_INVALID_NODE;
		}

		const auto index = static_cast<size_t>(params[2]);
		const auto& conns = node->getConnections();
		if (index >= conns.size())
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto target = conns[index]->getTarget();
		if (target == nullptr)
		{
			return GPS_ERROR_INTERNAL;
		}

		if (!WriteRef(amx, params[3], target->getID()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetConnectionTrafficMultiplier(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);
		const auto id = static_cast<int>(params[1]);
		const auto mult = amx_ctof(params[2]);
		Container::Connections::SetTrafficMultiplier(id, mult);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionTrafficMultiplier(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const auto id = static_cast<int>(params[1]);
		float mult = Container::Connections::GetTrafficMultiplier(id);
		return amx_ftoc(mult);
	}


	cell AMX_NATIVE_CALL SetAreaTrafficMultiplier(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const auto minx = amx_ctof(params[1]);
		const auto miny = amx_ctof(params[2]);
		const auto maxx = amx_ctof(params[3]);
		const auto maxy = amx_ctof(params[4]);
		const auto mult = amx_ctof(params[5]);
		Container::Connections::SetAreaTrafficMultiplier(minx, miny, maxx, maxy, mult);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL ResetTrafficMultipliers(AMX* amx, cell* params)
	{
		Container::Connections::ResetTrafficMultipliers();
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindInterceptPath(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(7);

		const auto pursuer_id = static_cast<int>(params[1]);
		const auto pursuer = Container::Nodes::Find(pursuer_id);
		if (pursuer == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_speed = amx_ctof(params[3]);
		const auto pursuer_speed = amx_ctof(params[4]);

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (8 * 4))
		{
			if (!IsValidPathType(params[8], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[8]);
		}

		int intercept_node_id = -1;
		float time_to_intercept = 0.0f;
		int path_id = Pathfinder::FindInterceptPath(pursuer, target, target_speed, pursuer_speed, intercept_node_id, time_to_intercept, type, amx);

		if (path_id == INVALID_PATH_ID)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		if (!WriteRef(amx, params[5], path_id) ||
		    !WriteRef(amx, params[6], intercept_node_id) ||
		    !WriteRef(amx, params[7], time_to_intercept))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPurePursuitSteering(AMX* amx, cell* params)
	{
		CHECK_PARAMS(7);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const float vx = amx_ctof(params[2]);
		const float vy = amx_ctof(params[3]);
		const float v_angle = amx_ctof(params[4]);
		float lookahead = amx_ctof(params[5]);
		if (lookahead < 2.0f) lookahead = 2.0f;

		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());
		if (count < 2)
		{
			if (!WriteRef(amx, params[6], 0.0f) ||
			    !WriteRef(amx, params[7], 0.0f))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			return GPS_ERROR_NONE;
		}

		int best_seg = 0;
		float gx = nodes.back()->getX();
		float gy = nodes.back()->getY();
		float gz = nodes.back()->getZ();
		float lat_err = 0.0f;
		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			ComputePathLaneTarget(path, vx, vy, lookahead, 0, count - 2, best_seg, gx, gy, gz, lat_err);
		}

		float target_bearing = std::atan2(vx - gx, gy - vy) * 180.0f / 3.14159265358979323846f;
		if (target_bearing < 0.0f) target_bearing += 360.0f;
		if (target_bearing >= 360.0f) target_bearing -= 360.0f;

		float diff = target_bearing - v_angle;
		while (diff > 180.0f) diff -= 360.0f;
		while (diff <= -180.0f) diff += 360.0f;

		float steer_ratio = std::max(-1.0f, std::min(1.0f, -diff / 35.0f));

		const float abs_diff = std::abs(diff);
		float recommended_speed = 80.0f;
		if (abs_diff > 50.0f) recommended_speed = 25.0f;
		else if (abs_diff > 25.0f) recommended_speed = 45.0f;
		else if (abs_diff > 10.0f) recommended_speed = 65.0f;

		if (!WriteRef(amx, params[6], steer_ratio) ||
		    !WriteRef(amx, params[7], recommended_speed))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathLaneTarget(AMX* amx, cell* params)
	{
		CHECK_PARAMS(9);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		cell* idx_ptr = PawnUtils::GetAddress(amx, params[5]);
		if (idx_ptr == nullptr) return GPS_ERROR_INVALID_PARAMS;

		const float x = amx_ctof(params[2]);
		const float y = amx_ctof(params[3]);
		const float lookahead = amx_ctof(params[4]);
		const int in_idx = static_cast<int>(*idx_ptr);

		int out_idx = in_idx;
		float tx = 0.0f, ty = 0.0f, tz = 0.0f, lat_err = 0.0f;
		bool ok = false;
		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			const int count = static_cast<int>(path->getNodes().size());
			if (count > 0)
			{
				const int num_segs = std::max(1, count - 1);
				const int clamped = std::clamp(in_idx, 0, num_segs - 1);
				const int min_seg = std::max(0, clamped - 1);
				const int max_seg = std::min(num_segs - 1, clamped + 6);
				ok = ComputePathLaneTarget(path, x, y, lookahead, min_seg, max_seg, out_idx, tx, ty, tz, lat_err);
			}
		}

		if (!ok)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		*idx_ptr = static_cast<cell>(out_idx);
		if (!WriteRef(amx, params[6], tx) ||
		    !WriteRef(amx, params[7], ty) ||
		    !WriteRef(amx, params[8], tz) ||
		    !WriteRef(amx, params[9], lat_err))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetReachableMapNodes(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(6);

		const auto source_id = static_cast<int>(params[1]);
		const auto source = Container::Nodes::Find(source_id);
		if (source == nullptr) return GPS_ERROR_INVALID_NODE;

		const float limit = amx_ctof(params[2]);
		const bool by_time = params[3] != 0;
		const float speed = amx_ctof(params[4]);

		const auto max_nodes = static_cast<int>(params[6]);
		if (max_nodes <= 0)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		cell* out_nodes = PawnUtils::GetArray(amx, params[5], max_nodes);
		if (out_nodes == nullptr)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (8 * 4))
		{
			if (!IsValidPathType(params[8], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[8]);
		}

		if (out_nodes == nullptr || max_nodes <= 0)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		std::vector<int> reached = Pathfinder::GetReachableNodes(source, limit, by_time, speed, type);
		const int written = std::min(static_cast<int>(reached.size()), max_nodes);

		for (int i = 0; i < written; ++i)
		{
			out_nodes[i] = reached[i];
		}

		if (!WriteRef(amx, params[7], written))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetReachableBoundingBox(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(6);

		const auto source_id = static_cast<int>(params[1]);
		const auto source = Container::Nodes::Find(source_id);
		if (source == nullptr) return GPS_ERROR_INVALID_NODE;

		const float max_dist = amx_ctof(params[2]);
		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (7 * 4))
		{
			if (!IsValidPathType(params[7], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[7]);
		}

		std::vector<int> reached = Pathfinder::GetReachableNodes(source, max_dist, false, 60.0f, type);
		if (reached.empty())
		{
			return GPS_ERROR_INVALID_NODE;
		}

		float minx = std::numeric_limits<float>::infinity();
		float miny = std::numeric_limits<float>::infinity();
		float maxx = -std::numeric_limits<float>::infinity();
		float maxy = -std::numeric_limits<float>::infinity();

		for (int id : reached)
		{
			auto node = Container::Nodes::Find(id);
			if (node != nullptr)
			{
				minx = std::min(minx, node->getX());
				miny = std::min(miny, node->getY());
				maxx = std::max(maxx, node->getX());
				maxy = std::max(maxy, node->getY());
			}
		}

		if (!WriteRef(amx, params[3], minx) ||
		    !WriteRef(amx, params[4], miny) ||
		    !WriteRef(amx, params[5], maxx) ||
		    !WriteRef(amx, params[6], maxy))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPointZoneName(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(5);

		const float x = amx_ctof(params[1]);
		const float y = amx_ctof(params[2]);
		const float z = amx_ctof(params[3]);
		const int max_len = static_cast<int>(params[5]);
		if (max_len <= 0 || PawnUtils::GetArray(amx, params[4], max_len) == nullptr)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		std::string zone = Zones::GetZoneNameAt(x, y, z);
		amx_SetCppString(amx, params[4], zone, max_len);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeZoneName(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(2);

		const auto node_id = static_cast<int>(params[1]);
		const auto node = Container::Nodes::Find(node_id);
		if (node == nullptr) return GPS_ERROR_INVALID_NODE;

		int max_len = 32;
		if (params[0] >= (3 * 4))
		{
			max_len = static_cast<int>(params[3]);
		}
		if (max_len <= 0 || PawnUtils::GetArray(amx, params[2], max_len) == nullptr)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		std::string zone = Zones::GetZoneNameAt(node->getX(), node->getY(), node->getZ());
		amx_SetCppString(amx, params[2], zone, max_len);

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathZonesSummary(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(4);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const int max_zones = static_cast<int>(params[3]);
		int max_zone_len = 32;
		if (params[0] >= (5 * 4))
		{
			max_zone_len = static_cast<int>(params[5]);
		}
		if (max_zones <= 0 || max_zone_len <= 0)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		std::vector<std::string> zone_names = Zones::GetPathZones(path->getNodes());

		const int count = std::min(static_cast<int>(zone_names.size()), max_zones);
		cell* base_addr = PawnUtils::GetArray(amx, params[2], max_zones);
		if (base_addr == nullptr)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		for (int i = 0; i < count; ++i)
		{
			cell* dest = PawnUtils::GetPawnSubArray(amx, base_addr, i, max_zone_len);
			if (dest != nullptr)
			{
				amx_SetString(dest, zone_names[i].c_str(), 0, 0, max_zone_len);
			}
		}

		if (!WriteRef(amx, params[4], count))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetPathAutoDestroy(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(1);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		uint32_t ttl_ms = 60000;
		if (params[0] >= (2 * 4))
		{
			ttl_ms = static_cast<uint32_t>(params[2]);
		}

		path->setTTL(ttl_ms);
		if (ttl_ms > 0)
		{
			Container::Paths::RegisterExpiry(path->getID(), path->getExpiryTime());
		}
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SavePathToFile(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const auto filename = amx_GetCppString(amx, params[2]);
		if (!File::IsSafeFilePath(filename))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		std::ofstream out("scriptfiles/" + filename);
		if (!out.good()) return GPS_ERROR_INTERNAL;
		out.imbue(std::locale::classic());

		const auto& nodes = path->getNodes();
		out << nodes.size() << "\n";
		for (auto node : nodes)
		{
			out << node->getID() << " ";
		}
		out << "\n";
		out.close();

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL LoadPathFromFile(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto filename = amx_GetCppString(amx, params[1]);
		if (!File::IsSafeFilePath(filename))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		std::ifstream in("scriptfiles/" + filename);
		if (!in.good()) return GPS_ERROR_INTERNAL;
		in.imbue(std::locale::classic());

		size_t count = 0;
		if (!(in >> count) || count == 0) return GPS_ERROR_INVALID_PATH;

		std::vector<Node*> nodes;
		float total_len = 0.0f;
		Node* prev = nullptr;

		for (size_t i = 0; i < count; ++i)
		{
			int node_id = -1;
			if (!(in >> node_id)) break;

			Node* node = Container::Nodes::Find(node_id);
			if (node != nullptr)
			{
				if (prev != nullptr)
				{
					total_len += prev->getDistanceFromNode(node);
				}
				nodes.push_back(node);
				prev = node;
			}
		}

		in.close();

		if (nodes.empty())
		{
			return GPS_ERROR_INVALID_PATH;
		}

		auto path = Container::Paths::Register(nodes, total_len, amx);

		if (!WriteRef(amx, params[2], path->getID()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL ExportPathToString(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);

		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const int max_len = static_cast<int>(params[3]);
		if (max_len <= 0 || PawnUtils::GetArray(amx, params[2], max_len) == nullptr)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}
		std::string str;
		const auto& nodes = path->getNodes();
		for (size_t i = 0; i < nodes.size(); ++i)
		{
			if (i > 0) str += ",";
			str += std::to_string(nodes[i]->getID());
		}

		amx_SetCppString(amx, params[2], str, max_len);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL ImportPathFromString(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);

		const auto str = amx_GetCppString(amx, params[1]);
		if (str.empty()) return GPS_ERROR_INVALID_PARAMS;

		std::stringstream ss(str);
		std::string item;
		std::vector<Node*> nodes;
		float total_len = 0.0f;
		Node* prev = nullptr;

		while (std::getline(ss, item, ','))
		{
			if (item.empty()) continue;
			int id = 0;
			auto [ptr, ec] = std::from_chars(item.data(), item.data() + item.size(), id);
			if (ec != std::errc() || ptr != item.data() + item.size())
			{
				return GPS_ERROR_INVALID_PARAMS;
			}

			Node* node = Container::Nodes::Find(id);
			if (node != nullptr)
			{
				if (prev != nullptr)
				{
					total_len += prev->getDistanceFromNode(node);
				}
				nodes.push_back(node);
				prev = node;
			}
		}

		if (nodes.empty())
		{
			return GPS_ERROR_INVALID_PATH;
		}

		auto path = Container::Paths::Register(nodes, total_len, amx);

		if (!WriteRef(amx, params[2], path->getID()))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindPathForVehicleCat(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		auto cat = VEHICLE_CAT_STANDARD;
		if (params[0] >= (4 * 4))
		{
			if (!IsValidVehicleCat(params[4]))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			cat = static_cast<VehicleCategory>(params[4]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (5 * 4))
		{
			if (!IsValidRoutePref(params[5]))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			pref = static_cast<RoutePreference>(params[5]);
		}

		const auto result = Pathfinder::FindPath(start, target, PATH_TYPE_VEHICLE, pref, cat, amx);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		if (!WriteRef(amx, params[3], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetMapNodeDanger(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);
		const auto id = static_cast<int>(params[1]);
		const auto weight = amx_ctof(params[2]);
		Container::Nodes::SetDangerWeight(id, weight);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetMapNodeDanger(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const auto id = static_cast<int>(params[1]);
		float danger = Container::Nodes::GetDangerWeight(id);
		return amx_ftoc(danger);
	}


	cell AMX_NATIVE_CALL SetAreaDangerWeight(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const auto minx = amx_ctof(params[1]);
		const auto miny = amx_ctof(params[2]);
		const auto maxx = amx_ctof(params[3]);
		const auto maxy = amx_ctof(params[4]);
		const auto weight = amx_ctof(params[5]);
		Container::Nodes::SetAreaDangerWeight(minx, miny, maxx, maxy, weight);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL ResetDangerWeights(AMX* amx, cell* params)
	{
		Container::Nodes::ResetDangerWeights();
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindSafePath(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		float danger_penalty = 5.0f;
		if (params[0] >= (4 * 4))
		{
			danger_penalty = amx_ctof(params[4]);
		}

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (5 * 4))
		{
			if (!IsValidPathType(params[5], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[5]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (6 * 4))
		{
			if (!IsValidRoutePref(params[6]))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			pref = static_cast<RoutePreference>(params[6]);
		}

		const auto result = Pathfinder::FindSafePath(start, target, danger_penalty, type, pref, amx);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		if (!WriteRef(amx, params[3], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindEcoPath(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		float max_slope = 0.12f;
		if (params[0] >= (4 * 4))
		{
			max_slope = amx_ctof(params[4]);
		}

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (5 * 4))
		{
			if (!IsValidPathType(params[5], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[5]);
		}

		const auto result = Pathfinder::FindEcoPath(start, target, max_slope, type, amx);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		if (!WriteRef(amx, params[3], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathElevationStats(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		float min_z = 0.0f, max_z = 0.0f, climb = 0.0f, max_incline = 0.0f;
		if (!Pathfinder::GetPathElevationStats(path, min_z, max_z, climb, max_incline))
		{
			return GPS_ERROR_INVALID_PATH;
		}

		if (!WriteRef(amx, params[2], min_z) ||
		    !WriteRef(amx, params[3], max_z) ||
		    !WriteRef(amx, params[4], climb) ||
		    !WriteRef(amx, params[5], max_incline))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL RegisterTunnelArea(AMX* amx, cell* params)
	{
		CHECK_PARAMS(7);
		const float minx = amx_ctof(params[1]);
		const float miny = amx_ctof(params[2]);
		const float minz = amx_ctof(params[3]);
		const float maxx = amx_ctof(params[4]);
		const float maxy = amx_ctof(params[5]);
		const float maxz = amx_ctof(params[6]);

		int id = Tunnels::RegisterTunnelArea(minx, miny, minz, maxx, maxy, maxz);

		if (!WriteRef(amx, params[7], id))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL UnregisterTunnelArea(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const int id = static_cast<int>(params[1]);
		return Tunnels::UnregisterTunnelArea(id) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL ClearCustomTunnelAreas(AMX* amx, cell* params)
	{
		Tunnels::ClearCustomTunnelAreas();
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsPointInTunnel(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);
		const float x = amx_ctof(params[1]);
		const float y = amx_ctof(params[2]);
		const float z = amx_ctof(params[3]);
		return Tunnels::IsPointInTunnel(x, y, z) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL IsMapNodeUnderground(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const auto id = static_cast<int>(params[1]);
		auto node = Container::Nodes::Find(id);
		return Tunnels::IsMapNodeUnderground(node) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL GetPathTunnelSegments(AMX* amx, cell* params)
	{
		CHECK_PARAMS(3);
		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		float entry = 0.0f, exit = 0.0f;
		bool has_tunnel = Tunnels::GetPathTunnelSegments(path, entry, exit);

		if (!WriteRef(amx, params[2], entry) ||
		    !WriteRef(amx, params[3], exit))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return has_tunnel ? 1 : 0;
	}


	cell AMX_NATIVE_CALL FindNearestPointByRoad(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(7);

		const float from_x = amx_ctof(params[1]);
		const float from_y = amx_ctof(params[2]);
		const float from_z = amx_ctof(params[3]);

		const int total_points = static_cast<int>(params[5]);
		if (total_points <= 0)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		cell* points_addr = PawnUtils::GetArray(amx, params[4], total_points);
		if (points_addr == nullptr)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		std::vector<std::tuple<float, float, float>> destinations;
		destinations.reserve(total_points);

		for (int i = 0; i < total_points; ++i)
		{
			cell* sub_addr = PawnUtils::GetPawnSubArray(amx, points_addr, i, 3);
			if (sub_addr == nullptr)
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			destinations.push_back({ amx_ctof(sub_addr[0]), amx_ctof(sub_addr[1]), amx_ctof(sub_addr[2]) });
		}

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (9 * 4))
		{
			if (!IsValidPathType(params[9], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[9]);
		}

		bool create_path = false;
		if (params[0] >= (8 * static_cast<int>(sizeof(cell))))
		{
			cell* path_addr = PawnUtils::GetAddress(amx, params[8]);
			if (path_addr != nullptr)
			{
				if (params[0] >= (9 * static_cast<int>(sizeof(cell))) && *path_addr == INVALID_PATH_ID)
				{
					create_path = false;
				}
				else
				{
					create_path = true;
				}
			}
		}

		int nearest_idx = -1;
		float road_dist = 0.0f;
		int path_id = INVALID_PATH_ID;

		int status = Pathfinder::FindNearestPointByRoad(from_x, from_y, from_z, destinations, nearest_idx, road_dist, path_id, type, amx, create_path);
		if (status != 0)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		WriteRef(amx, params[6], nearest_idx);
		WriteRef(amx, params[7], road_dist);

		if (params[0] >= (8 * static_cast<int>(sizeof(cell))))
		{
			WriteRef(amx, params[8], path_id);
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL RegisterRoadRadar(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(5);

		const int id = static_cast<int>(params[1]);
		const float x = amx_ctof(params[2]);
		const float y = amx_ctof(params[3]);
		const float z = amx_ctof(params[4]);
		const float speed_limit = amx_ctof(params[5]);
		float alert_radius = 300.0f;
		if (params[0] >= (6 * 4))
		{
			alert_radius = amx_ctof(params[6]);
		}

		Radars::RegisterRadar(id, x, y, z, speed_limit, alert_radius);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL UnregisterRoadRadar(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const int id = static_cast<int>(params[1]);
		return Radars::UnregisterRadar(id) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL ClearRoadRadars(AMX* amx, cell* params)
	{
		Radars::ClearRadars();
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetNextPathRadar(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const float current_dist = amx_ctof(params[2]);
		int radar_id = -1;
		float dist_to_radar = 0.0f;
		float speed_limit = 0.0f;

		bool found = Radars::GetNextPathRadar(path, current_dist, radar_id, dist_to_radar, speed_limit);

		if (!WriteRef(amx, params[3], radar_id) ||
		    !WriteRef(amx, params[4], dist_to_radar) ||
		    !WriteRef(amx, params[5], speed_limit))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return found ? 1 : 0;
	}


	cell AMX_NATIVE_CALL GetPathOffsetPosition(AMX* amx, cell* params)
	{
		CHECK_PARAMS(7);
		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const float dist = amx_ctof(params[2]);
		const float offset = amx_ctof(params[3]);

		float ox = 0.0f, oy = 0.0f, oz = 0.0f, oang = 0.0f;
		if (!Pathfinder::GetPathOffsetPosition(path, dist, offset, ox, oy, oz, oang))
		{
			return GPS_ERROR_INVALID_PATH;
		}

		if (!WriteRef(amx, params[4], ox) ||
		    !WriteRef(amx, params[5], oy) ||
		    !WriteRef(amx, params[6], oz) ||
		    !WriteRef(amx, params[7], oang))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsMapNodeDeadEnd(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const auto id = static_cast<int>(params[1]);
		auto node = Container::Nodes::Find(id);
		return Pathfinder::IsMapNodeDeadEnd(node) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL FindEvasivePath(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (4 * 4))
		{
			if (!IsValidPathType(params[4], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[4]);
		}

		const auto result = Pathfinder::FindEvasivePath(start, target, type, amx);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		if (!WriteRef(amx, params[3], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetConnectionToll(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(2);

		const auto id = static_cast<int>(params[1]);
		const bool has_toll = params[2] != 0;
		int cost = 0;
		if (params[0] >= (3 * 4))
		{
			cost = static_cast<int>(params[3]);
		}

		Container::Connections::SetToll(id, has_toll, cost);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionToll(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(1);

		const auto id = static_cast<int>(params[1]);
		int cost = 0;
		bool has_toll = Container::Connections::GetToll(id, cost);

		if (params[0] >= (2 * 4))
		{
			WriteRef(amx, params[2], cost);
		}

		return has_toll ? 1 : 0;
	}


	cell AMX_NATIVE_CALL FindPathAvoidTolls(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (4 * 4))
		{
			if (!IsValidPathType(params[4], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[4]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (5 * 4))
		{
			if (!IsValidRoutePref(params[5]))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			pref = static_cast<RoutePreference>(params[5]);
		}

		const auto result = Pathfinder::FindPathAvoidTolls(start, target, type, pref, amx);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		if (!WriteRef(amx, params[3], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathRoundaboutInfo(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const int index = static_cast<int>(params[2]);
		bool is_roundabout = false;
		int exit_number = 0;
		float radius = 0.0f;

		if (!Pathfinder::GetPathRoundaboutInfo(path, index, is_roundabout, exit_number, radius))
		{
			return GPS_ERROR_INVALID_PATH;
		}

		if (!WriteRef(amx, params[3], is_roundabout ? 1 : 0) ||
		    !WriteRef(amx, params[4], exit_number) ||
		    !WriteRef(amx, params[5], radius))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetSmoothPathPoints(AMX* amx, cell* params)
	{
		CHECK_PARAMS(7);
		const auto path_id = static_cast<int>(params[1]);
		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr) return GPS_ERROR_INVALID_PATH;

		const float spacing = amx_ctof(params[2]);
		const int max_points = static_cast<int>(params[6]);
		if (max_points <= 0)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		cell* out_x = PawnUtils::GetArray(amx, params[3], max_points);
		cell* out_y = PawnUtils::GetArray(amx, params[4], max_points);
		cell* out_z = PawnUtils::GetArray(amx, params[5], max_points);
		if (out_x == nullptr || out_y == nullptr || out_z == nullptr)
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		std::vector<std::tuple<float, float, float>> pts;
		if (!Pathfinder::GetSmoothPathPoints(path, spacing, pts, max_points))
		{
			return GPS_ERROR_INVALID_PATH;
		}

		const int written = std::min(static_cast<int>(pts.size()), max_points);
		for (int i = 0; i < written; ++i)
		{
			out_x[i] = PawnUtils::FloatToCell(std::get<0>(pts[i]));
			out_y[i] = PawnUtils::FloatToCell(std::get<1>(pts[i]));
			out_z[i] = PawnUtils::FloatToCell(std::get<2>(pts[i]));
		}

		if (!WriteRef(amx, params[7], written))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsConnectionPaved(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const int id = static_cast<int>(params[1]);
		return Container::Connections::IsPaved(id) ? 1 : 0;
	}


	cell AMX_NATIVE_CALL SetConnectionPaved(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);
		const int id = static_cast<int>(params[1]);
		const bool paved = params[2] != 0;
		Container::Connections::SetPaved(id, paved);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindPathPavedOnly(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(3);

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (4 * 4))
		{
			if (!IsValidPathType(params[4], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[4]);
		}

		const auto result = Pathfinder::FindPathPavedOnly(start, target, type, amx);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		if (!WriteRef(amx, params[3], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionRoadClass(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(1);

		const int id = static_cast<int>(params[1]);
		RoadClass rc = Container::Connections::GetRoadClass(id);

		if (params[0] >= (2 * 4))
		{
			float speed = Container::Connections::GetSpeedLimit(id);
			WriteRef(amx, params[2], speed);
		}

		return static_cast<cell>(rc);
	}


	cell AMX_NATIVE_CALL SetConnectionSpeedLimit(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);
		const int id = static_cast<int>(params[1]);
		const float speed = amx_ctof(params[2]);
		Container::Connections::SetSpeedLimit(id, speed);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionSpeedLimit(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const int id = static_cast<int>(params[1]);
		float speed = Container::Connections::GetSpeedLimit(id);
		return amx_ftoc(speed);
	}


	cell AMX_NATIVE_CALL GetPathRejoinPoint(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const auto old_id = static_cast<int>(params[1]);
		const auto old_path = Container::Paths::Find(old_id);
		if (old_path == nullptr) return GPS_ERROR_INVALID_PATH;

		const auto new_id = static_cast<int>(params[2]);
		const auto new_path = Container::Paths::Find(new_id);
		if (new_path == nullptr) return GPS_ERROR_INVALID_PATH;

		int diverge = -1, rejoin_old = -1, rejoin_new = -1;
		if (!Pathfinder::GetPathRejoinPoint(old_path, new_path, diverge, rejoin_old, rejoin_new))
		{
			return GPS_ERROR_INVALID_PATH;
		}

		if (!WriteRef(amx, params[3], diverge) ||
		    !WriteRef(amx, params[4], rejoin_old) ||
		    !WriteRef(amx, params[5], rejoin_new))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL SetConnectionMaxHeight(AMX* amx, cell* params)
	{
		CHECK_PARAMS(2);
		const int id = static_cast<int>(params[1]);
		const float height = amx_ctof(params[2]);
		Container::Connections::SetMaxHeight(id, height);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetConnectionMaxHeight(AMX* amx, cell* params)
	{
		CHECK_PARAMS(1);
		const int id = static_cast<int>(params[1]);
		float height = Container::Connections::GetMaxHeight(id);
		return amx_ftoc(height);
	}


	cell AMX_NATIVE_CALL SetAreaMaxHeight(AMX* amx, cell* params)
	{
		CHECK_PARAMS(5);
		const float minx = amx_ctof(params[1]);
		const float miny = amx_ctof(params[2]);
		const float maxx = amx_ctof(params[3]);
		const float maxy = amx_ctof(params[4]);
		const float height = amx_ctof(params[5]);
		Container::Connections::SetAreaMaxHeight(minx, miny, maxx, maxy, height);
		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindPathWithHeightLimit(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(4);

		const auto start_id = static_cast<int>(params[1]);
		const auto start = Container::Nodes::Find(start_id);
		if (start == nullptr) return GPS_ERROR_INVALID_NODE;

		const auto target_id = static_cast<int>(params[2]);
		const auto target = Container::Nodes::Find(target_id);
		if (target == nullptr) return GPS_ERROR_INVALID_NODE;

		const float vehicle_height = amx_ctof(params[3]);

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (5 * 4))
		{
			if (!IsValidPathType(params[5], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[5]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (6 * 4))
		{
			if (!IsValidRoutePref(params[6]))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			pref = static_cast<RoutePreference>(params[6]);
		}

		const auto result = Pathfinder::FindPathWithHeightLimit(start, target, vehicle_height, type, pref, amx);
		if (result == -1) return GPS_ERROR_INVALID_PATH;

		if (!WriteRef(amx, params[4], result))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL FindPathToPoint(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(9);

		const auto source_id = static_cast<int>(params[1]);
		const float x = amx_ctof(params[2]);
		const float y = amx_ctof(params[3]);
		const float z = amx_ctof(params[4]);
		const float max_road_dist = (params[0] >= (10 * 4)) ? amx_ctof(params[10]) : 150.0f;

		auto type = PATH_TYPE_VEHICLE;
		if (params[0] >= (11 * 4))
		{
			if (!IsValidPathType(params[11], true))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			type = static_cast<PathType>(params[11]);
		}

		auto pref = ROUTE_PREF_SHORTEST;
		if (params[0] >= (12 * 4))
		{
			if (!IsValidRoutePref(params[12]))
			{
				return GPS_ERROR_INVALID_PARAMS;
			}
			pref = static_cast<RoutePreference>(params[12]);
		}

		std::vector<Node*> best_nodes;
		float total_len = 0.0f;
		float stop_x = x, stop_y = y, stop_z = z;
		float dist_to_road = -1.0f;
		bool source_invalid = false;
		bool found = false;

		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			Node* source = Container::Nodes::Find(source_id);
			if (source == nullptr || source->isBlocked())
			{
				source_invalid = true;
			}
			else
			{
				const int src_comp = Container::Nodes::GetConnectedComponent(source, type);
				const auto candidates = Container::Connections::FindRoadCandidates(x, y, z, max_road_dist, type, src_comp);

				auto try_candidate = [&](const Container::Connections::RoadCandidate& cand, bool forbid_reverse) -> bool
				{
					Connection* edge = cand.conn;
					Node* s = edge->getSource();
					Node* t = edge->getTarget();

					const float vx = t->getX() - s->getX();
					const float vy = t->getY() - s->getY();
					const float edge_len = std::hypot(vx, vy);
					if (edge_len <= 0.0001f)
					{
						return false;
					}

					const float t_proj = std::clamp(
						((cand.qx - s->getX()) * vx + (cand.qy - s->getY()) * vy) / (edge_len * edge_len),
						0.0f, 1.0f);

					Connection* rev_edge = nullptr;
					if (forbid_reverse)
					{
						for (Connection* c : t->getConnections())
						{
							if (c != nullptr && c->getSource() == t && c->getTarget() == s)
							{
								rev_edge = c;
								break;
							}
						}
					}

					std::vector<Node*> prefix_nodes;
					float prefix_len = 0.0f;
					if (!Pathfinder::FindPathNodes(source, s, prefix_nodes, prefix_len, type, pref,
						VEHICLE_CAT_STANDARD, 0.0f, 0.0f, false, false, 0.0f, false, rev_edge))
					{
						return false;
					}

					prefix_nodes.push_back(t);
					best_nodes = std::move(prefix_nodes);
					total_len = prefix_len + t_proj * edge->getDistance();

					const float ux = vx / edge_len;
					const float uy = vy / edge_len;
					const float d_lane = edge->getLaneOffset(edge->getTravelLanes() - 1);
					stop_x = cand.qx + uy * d_lane;
					stop_y = cand.qy - ux * d_lane;
					stop_z = cand.qz;
					dist_to_road = std::sqrt(cand.dist2d_sq);
					return true;
				};

				const size_t max_tries = std::min<size_t>(candidates.size(), 12);
				for (size_t i = 0; i < max_tries && !found; ++i)
				{
					found = try_candidate(candidates[i], true);
				}
				if (!found)
				{
					for (size_t i = 0; i < max_tries && !found; ++i)
					{
						found = try_candidate(candidates[i], false);
					}
				}
			}
		}

		if (source_invalid)
		{
			WriteRef(amx, params[5], INVALID_PATH_ID);
			return GPS_ERROR_INVALID_NODE;
		}

		if (!found)
		{
			WriteRef(amx, params[5], INVALID_PATH_ID);
			return GPS_ERROR_INVALID_PATH;
		}

		Path* path = Container::Paths::Register(best_nodes, total_len, amx);
		if (path == nullptr)
		{
			WriteRef(amx, params[5], INVALID_PATH_ID);
			return GPS_ERROR_INTERNAL;
		}

		const Node* s = best_nodes[best_nodes.size() - 2];
		const Node* t = best_nodes[best_nodes.size() - 1];
		const float seg_dist = std::hypot(stop_x - s->getX(), stop_y - s->getY());
		(void)t;
		path->setStopPoint(stop_x, stop_y, stop_z, std::max(0.5f, seg_dist));

		if (!WriteRef(amx, params[5], path->getID()) ||
		    !WriteRef(amx, params[6], stop_x) ||
		    !WriteRef(amx, params[7], stop_y) ||
		    !WriteRef(amx, params[8], stop_z) ||
		    !WriteRef(amx, params[9], dist_to_road))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL IsPathStopReached(AMX* amx, cell* params)
	{
		CHECK_PARAMS(4);

		const int path_id = static_cast<int>(params[1]);
		const int path_index = static_cast<int>(params[2]);
		const float x = amx_ctof(params[3]);
		const float y = amx_ctof(params[4]);

		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return 0;
		}

		std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
		const auto& nodes = path->getNodes();
		const int count = static_cast<int>(nodes.size());
		if (count <= 0)
		{
			return 0;
		}

		if (count == 1)
		{
			float end_x = nodes[0]->getX(), end_y = nodes[0]->getY(), end_z = 0.0f, seg_d = 0.0f;
			if (path->hasStopPoint())
			{
				path->getStopPoint(end_x, end_y, end_z, seg_d);
			}
			return (std::hypot(x - end_x, y - end_y) <= 8.0f) ? 1 : 0;
		}

		const int num_segs = count - 1;
		const int clamped_idx = std::clamp(path_index, 0, num_segs - 1);
		const int build_start = std::max(0, clamped_idx - 1);

		std::vector<LaneSeg> segs;
		bool reaches_end = false;
		if (!BuildPathLaneSegments(path, build_start, segs, 60.0f, &reaches_end) || segs.empty())
		{
			return 0;
		}
		if (!reaches_end)
		{
			return 0;
		}

		const int max_search = std::min(num_segs - 1, clamped_idx + 8);
		int best_idx = 0;
		float best_t = 0.0f, best_qx = 0.0f, best_qy = 0.0f, lat_err = 0.0f;
		ProjectOntoLaneSegments(segs, build_start, x, y, clamped_idx, max_search, best_idx, best_t, best_qx, best_qy, lat_err);

		const float end_x = segs.back().ex;
		const float end_y = segs.back().ey;
		const float direct_dist = std::hypot(x - end_x, y - end_y);
		const float reach_radius = path->hasStopPoint() ? 6.0f : 10.0f;
		if (direct_dist <= reach_radius)
		{
			return 1;
		}

		const int last_idx = static_cast<int>(segs.size()) - 1;
		if (best_idx == last_idx && std::abs(lat_err) <= 12.0f)
		{
			const LaneSeg& last = segs.back();
			const float rem_len = std::hypot(last.ex - last.sx, last.ey - last.sy) * (1.0f - best_t);
			if (best_t >= 0.99f || rem_len <= 5.0f)
			{
				return 1;
			}
		}

		return 0;
	}


	static float ComputeCurveSpeedMs(
		const float ax, const float ay,
		const float bx, const float by,
		const float cx, const float cy,
		const float v_max_ms,
		const float min_curve_ms)
	{
		const float abx = bx - ax, aby = by - ay;
		const float bcx = cx - bx, bcy = cy - by;
		const float acx = cx - ax, acy = cy - ay;
		const float a = std::hypot(abx, aby);
		const float b = std::hypot(bcx, bcy);
		const float c = std::hypot(acx, acy);
		if (a < 0.5f || b < 0.5f)
		{
			return v_max_ms;
		}

		const float dot = abx * bcx + aby * bcy;
		if (dot < -0.5f * a * b)
		{
			return min_curve_ms;
		}

		const float cross = std::abs(abx * acy - aby * acx);
		const float kappa = (2.0f * cross) / std::max(a * b * c, 1e-4f);
		if (kappa > 0.002f)
		{
			return std::max(min_curve_ms, std::sqrt(CURVE_MAX_LATERAL_ACCEL / kappa));
		}
		return v_max_ms;
	}

	cell AMX_NATIVE_CALL GetPathSpeedAdvice(AMX* amx, cell* params)
	{
		CHECK_PARAMS(7);

		const int path_id = static_cast<int>(params[1]);
		const int path_index = static_cast<int>(params[2]);
		const float x = amx_ctof(params[3]);
		const float y = amx_ctof(params[4]);
		const float max_speed_kmh = std::max(0.0f, amx_ctof(params[5]));

		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		float out_target_kmh = max_speed_kmh;
		float out_constraint_dist = -1.0f;
		int err = GPS_ERROR_NONE;

		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			const auto& nodes = path->getNodes();
			const int count = static_cast<int>(nodes.size());
			if (count <= 0)
			{
				err = GPS_ERROR_INVALID_PATH;
			}
			else if (count > 1)
			{
				const int num_segs = count - 1;
				const int clamped_idx = std::clamp(path_index, 0, num_segs - 1);
				const int build_start = std::max(0, clamped_idx - 1);
				const float v_max_ms = max_speed_kmh / 3.6f;
				const float d_brake_horizon = std::max(40.0f, (v_max_ms * v_max_ms) / (2.0f * CURVE_MAX_BRAKE_DECEL) + 25.0f);

				std::vector<LaneSeg> segs;
				bool reaches_end = false;
				if (!BuildPathLaneSegments(path, build_start, segs, d_brake_horizon + CURVE_ARC_WINDOW + 30.0f, &reaches_end) || segs.empty())
				{
					err = GPS_ERROR_INTERNAL;
				}
				else
				{
					const int max_search = std::min(num_segs - 1, clamped_idx + 8);
					int best_idx = 0;
					float best_t = 0.0f, best_qx = 0.0f, best_qy = 0.0f, lat_err = 0.0f;
					ProjectOntoLaneSegments(segs, build_start, x, y, clamped_idx, max_search, best_idx, best_t, best_qx, best_qy, lat_err);

					struct PolyPt { float x, y, s; };
					std::vector<PolyPt> poly;
					float cum_s = 0.0f;

					const LaneSeg& start_seg = segs[best_idx];
					const float cur_x = start_seg.sx + best_t * (start_seg.ex - start_seg.sx);
					const float cur_y = start_seg.sy + best_t * (start_seg.ey - start_seg.sy);
					poly.push_back({ cur_x, cur_y, 0.0f });

					for (int idx = best_idx; idx < static_cast<int>(segs.size()); ++idx)
					{
						const LaneSeg& s = segs[idx];
						if (idx > best_idx)
						{
							const float gap = std::hypot(s.sx - poly.back().x, s.sy - poly.back().y);
							if (gap > 0.01f)
							{
								cum_s += gap;
								poly.push_back({ s.sx, s.sy, cum_s });
							}
						}

						const float seg_end_dist = std::hypot(s.ex - poly.back().x, s.ey - poly.back().y);
						if (seg_end_dist > 0.01f)
						{
							cum_s += seg_end_dist;
							poly.push_back({ s.ex, s.ey, cum_s });
						}

						if (cum_s >= d_brake_horizon + CURVE_ARC_WINDOW)
						{
							break;
						}
					}

					auto sample_at_s = [&poly](const float target_s, float& ox, float& oy)
					{
						if (target_s <= 0.0f || poly.size() == 1)
						{
							ox = poly.front().x;
							oy = poly.front().y;
							return;
						}
						for (size_t i = 0; i + 1 < poly.size(); ++i)
						{
							if (target_s <= poly[i + 1].s)
							{
								const float ds = poly[i + 1].s - poly[i].s;
								const float frac = (ds > 0.0001f) ? ((target_s - poly[i].s) / ds) : 0.0f;
								ox = poly[i].x + frac * (poly[i + 1].x - poly[i].x);
								oy = poly[i].y + frac * (poly[i + 1].y - poly[i].y);
								return;
							}
						}
						ox = poly.back().x;
						oy = poly.back().y;
					};

					float best_target_ms = v_max_ms;
					float best_constraint_dist = -1.0f;

					const float total_poly_len = poly.back().s;
					const float eval_limit = std::min(total_poly_len, d_brake_horizon);
					const float min_curve_ms = std::min(v_max_ms, CURVE_MIN_SPEED_KMH / 3.6f);

					std::vector<float> sample_distances;
					for (float s_val = 0.0f; s_val <= eval_limit; s_val += 3.0f)
					{
						sample_distances.push_back(s_val);
					}
					for (const auto& pt : poly)
					{
						if (pt.s > 0.0f && pt.s <= eval_limit)
						{
							sample_distances.push_back(pt.s);
						}
					}

					for (const float s_c : sample_distances)
					{
						const float s_prev = std::max(0.0f, s_c - CURVE_ARC_WINDOW);
						const float s_next = std::min(total_poly_len, s_c + CURVE_ARC_WINDOW);
						if ((s_c - s_prev) < 1.0f || (s_next - s_c) < 1.0f)
						{
							continue;
						}

						float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f, cx = 0.0f, cy = 0.0f;
						sample_at_s(s_prev, ax, ay);
						sample_at_s(s_c, bx, by);
						sample_at_s(s_next, cx, cy);

						const float v_curve_ms = ComputeCurveSpeedMs(ax, ay, bx, by, cx, cy, v_max_ms, min_curve_ms);

						if (v_curve_ms < v_max_ms)
						{
							const float v_allowed_ms = std::sqrt(v_curve_ms * v_curve_ms + 2.0f * CURVE_MAX_BRAKE_DECEL * s_c);
							if (v_allowed_ms < best_target_ms)
							{
								best_target_ms = v_allowed_ms;
								best_constraint_dist = s_c;
							}
						}
					}

					if (reaches_end && path->hasStopPoint() && total_poly_len <= d_brake_horizon)
					{
						const float stop_brake_dist = std::max(0.0f, total_poly_len - 2.0f);
						const float v_stop_ms = std::sqrt(2.0f * CURVE_MAX_BRAKE_DECEL * stop_brake_dist);
						if (v_stop_ms < best_target_ms)
						{
							best_target_ms = v_stop_ms;
							best_constraint_dist = total_poly_len;
						}
					}

					out_target_kmh = std::clamp(best_target_ms * 3.6f, 0.0f, max_speed_kmh);
					out_constraint_dist = best_constraint_dist;
				}
			}
		}

		if (err != GPS_ERROR_NONE)
		{
			return err;
		}

		if (!WriteRef(amx, params[6], out_target_kmh) ||
		    !WriteRef(amx, params[7], out_constraint_dist))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}


	cell AMX_NATIVE_CALL GetPathRoadProfile(AMX* amx, cell* params)
	{
		CHECK_PARAMS(7);

		const int path_id = static_cast<int>(params[1]);
		const int path_index = static_cast<int>(params[2]);
		const float x = amx_ctof(params[3]);
		const float y = amx_ctof(params[4]);
		const float ahead_dist = std::max(5.0f, amx_ctof(params[5]));

		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		float out_road_z = 0.0f;
		float out_grade_ahead = 0.0f;
		int err = GPS_ERROR_NONE;

		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			const auto& nodes = path->getNodes();
			const int count = static_cast<int>(nodes.size());
			if (count <= 0)
			{
				err = GPS_ERROR_INVALID_PATH;
			}
			else if (count == 1)
			{
				out_road_z = nodes[0]->getZ();
				out_grade_ahead = 0.0f;
			}
			else
			{
				const int num_segs = count - 1;
				const int clamped_idx = std::clamp(path_index, 0, num_segs - 1);
				const int build_start = std::max(0, clamped_idx - 1);

				std::vector<LaneSeg> segs;
				if (!BuildPathLaneSegments(path, build_start, segs, ahead_dist + 60.0f) || segs.empty())
				{
					err = GPS_ERROR_INTERNAL;
				}
				else
				{
					const int max_search = std::min(num_segs - 1, clamped_idx + 8);
					int best_idx = 0;
					float best_t = 0.0f, best_qx = 0.0f, best_qy = 0.0f, lat_err = 0.0f;
					ProjectOntoLaneSegments(segs, build_start, x, y, clamped_idx, max_search, best_idx, best_t, best_qx, best_qy, lat_err);

					const LaneSeg& cur_seg = segs[best_idx];
					out_road_z = cur_seg.sz + best_t * (cur_seg.ez - cur_seg.sz);

					float ax = 0.0f, ay = 0.0f, z_ahead = out_road_z, covered = 0.0f;
					WalkLanePolylineForward(segs, best_idx, best_t, ahead_dist, ax, ay, z_ahead, covered);

					if (covered > 1.0f)
					{
						out_grade_ahead = std::clamp((z_ahead - out_road_z) / covered, -0.35f, 0.35f);
					}
					else if (cur_seg.raw_len > 1.0f)
					{
						out_grade_ahead = std::clamp((cur_seg.ez - cur_seg.sz) / cur_seg.raw_len, -0.35f, 0.35f);
					}
				}
			}
		}

		if (err != GPS_ERROR_NONE)
		{
			return err;
		}

		if (!WriteRef(amx, params[6], out_road_z) ||
		    !WriteRef(amx, params[7], out_grade_ahead))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}

	cell AMX_NATIVE_CALL GetPathTimeEstimate(AMX* amx, cell* params)
	{
		CHECK_PARAMS_MIN(6);

		const int path_id = static_cast<int>(params[1]);
		const int path_index = static_cast<int>(params[2]);
		const float x = amx_ctof(params[3]);
		const float y = amx_ctof(params[4]);
		const float current_speed_kmh = (params[0] >= 7 * static_cast<cell>(sizeof(cell))) ? amx_ctof(params[7]) : 0.0f;
		const float max_speed_kmh = (params[0] >= 8 * static_cast<cell>(sizeof(cell))) ? amx_ctof(params[8]) : 0.0f;

		if (!(current_speed_kmh >= 0.0f) || !(max_speed_kmh >= 0.0f) ||
		    !std::isfinite(current_speed_kmh) || !std::isfinite(max_speed_kmh))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		const auto path = Container::Paths::Find(path_id);
		if (path == nullptr)
		{
			return GPS_ERROR_INVALID_PATH;
		}

		float out_seconds = 0.0f;
		float out_remaining_dist = 0.0f;
		int err = GPS_ERROR_NONE;

		{
			std::shared_lock<std::shared_mutex> lock(Container::GetGraphMutex());
			const auto& nodes = path->getNodes();
			const int count = static_cast<int>(nodes.size());
			if (count < 2)
			{
				err = GPS_ERROR_INVALID_PATH;
			}
			else
			{
				const int num_segs = count - 1;
				const int clamped_idx = std::clamp(path_index, 0, num_segs - 1);
				const int build_start = std::max(0, clamped_idx - 1);

				std::vector<LaneSeg> segs;
				bool reaches_end = false;
				if (!BuildPathLaneSegments(path, build_start, segs, -1.0f, &reaches_end) || segs.empty())
				{
					err = GPS_ERROR_INTERNAL;
				}
				else
				{
					const int max_search = std::min(num_segs - 1, clamped_idx + 8);
					int best_idx = 0;
					float best_t = 0.0f, best_qx = 0.0f, best_qy = 0.0f, lat_err = 0.0f;
					ProjectOntoLaneSegments(segs, build_start, x, y, clamped_idx, max_search, best_idx, best_t, best_qx, best_qy, lat_err);

					auto seg_limit_ms = [max_speed_kmh](const Connection* conn) -> float
					{
						if (max_speed_kmh > 0.0f)
						{
							return std::max(0.1f, max_speed_kmh / 3.6f);
						}
						const float kmh = (conn != nullptr) ? conn->getSpeedLimit() : 50.0f;
						return std::max(0.1f, kmh / 3.6f);
					};

					struct EtaPolyPt
					{
						float x;
						float y;
						float s;
						float lim_ms;
					};

					std::vector<EtaPolyPt> poly;
					poly.reserve(segs.size() - best_idx + 4);
					float cum_s = 0.0f;

					const LaneSeg& start_seg = segs[best_idx];
					const float cur_x = start_seg.sx + best_t * (start_seg.ex - start_seg.sx);
					const float cur_y = start_seg.sy + best_t * (start_seg.ey - start_seg.sy);
					poly.push_back({ cur_x, cur_y, 0.0f, seg_limit_ms(start_seg.conn) });

					for (int idx = best_idx; idx < static_cast<int>(segs.size()); ++idx)
					{
						const LaneSeg& s = segs[idx];
						const float lim_ms = seg_limit_ms(s.conn);
						if (idx > best_idx)
						{
							const float gap = std::hypot(s.sx - poly.back().x, s.sy - poly.back().y);
							if (gap > 0.01f)
							{
								cum_s += gap;
								poly.push_back({ s.sx, s.sy, cum_s, lim_ms });
							}
						}

						const float seg_end_dist = std::hypot(s.ex - poly.back().x, s.ey - poly.back().y);
						if (seg_end_dist > 0.01f)
						{
							cum_s += seg_end_dist;
							poly.push_back({ s.ex, s.ey, cum_s, lim_ms });
						}
					}

					const float total_poly_len = poly.back().s;
					out_remaining_dist = total_poly_len;

					if (poly.size() >= 2 && total_poly_len > 0.01f)
					{
						std::vector<float> s_vals;
						s_vals.reserve(static_cast<size_t>(total_poly_len / 3.0f) + poly.size() + 2);
						size_t pk = 0;
						float grid_s = 0.0f;
						while (grid_s <= total_poly_len || pk < poly.size())
						{
							const float next_grid = (grid_s <= total_poly_len) ? grid_s : std::numeric_limits<float>::infinity();
							const float next_poly = (pk < poly.size()) ? poly[pk].s : std::numeric_limits<float>::infinity();
							float chosen = 0.0f;
							if (next_grid <= next_poly)
							{
								chosen = next_grid;
								grid_s += 3.0f;
							}
							else
							{
								chosen = next_poly;
								++pk;
							}
							if (s_vals.empty() || (chosen - s_vals.back()) > 0.05f)
							{
								s_vals.push_back(chosen);
							}
						}
						if (total_poly_len - s_vals.back() > 1e-4f)
						{
							if (total_poly_len - s_vals.back() <= 0.05f)
							{
								s_vals.back() = total_poly_len;
							}
							else
							{
								s_vals.push_back(total_poly_len);
							}
						}

						auto sample_cursor = [&poly](size_t& cursor, const float target_s, float& ox, float& oy, float& out_lim_ms)
						{
							if (target_s <= 0.0f)
							{
								ox = poly.front().x;
								oy = poly.front().y;
								out_lim_ms = poly.front().lim_ms;
								return;
							}
							while (cursor + 2 < poly.size() && target_s > poly[cursor + 1].s)
							{
								++cursor;
							}
							if (target_s >= poly.back().s)
							{
								ox = poly.back().x;
								oy = poly.back().y;
								out_lim_ms = poly.back().lim_ms;
								return;
							}
							const float ds = poly[cursor + 1].s - poly[cursor].s;
							const float frac = (ds > 0.0001f) ? ((target_s - poly[cursor].s) / ds) : 0.0f;
							ox = poly[cursor].x + frac * (poly[cursor + 1].x - poly[cursor].x);
							oy = poly[cursor].y + frac * (poly[cursor + 1].y - poly[cursor].y);
							out_lim_ms = poly[cursor + 1].lim_ms;
						};

						const size_t n_samples = s_vals.size();
						std::vector<float> v_profile(n_samples);
						const float global_min_curve_ms = CURVE_MIN_SPEED_KMH / 3.6f;

						size_t cur_prev = 0;
						size_t cur_mid = 0;
						size_t cur_next = 0;

						for (size_t i = 0; i < n_samples; ++i)
						{
							const float s_c = s_vals[i];
							float bx = 0.0f, by = 0.0f, lim_ms = 0.0f;
							sample_cursor(cur_mid, s_c, bx, by, lim_ms);

							float v_allowed = lim_ms;
							const float s_prev = std::max(0.0f, s_c - CURVE_ARC_WINDOW);
							const float s_next = std::min(total_poly_len, s_c + CURVE_ARC_WINDOW);
							if ((s_c - s_prev) >= 1.0f && (s_next - s_c) >= 1.0f)
							{
								float ax = 0.0f, ay = 0.0f, lim_a = 0.0f;
								float cx = 0.0f, cy = 0.0f, lim_c = 0.0f;
								sample_cursor(cur_prev, s_prev, ax, ay, lim_a);
								sample_cursor(cur_next, s_next, cx, cy, lim_c);
								const float min_curve_ms = std::min(lim_ms, global_min_curve_ms);
								const float v_curve_ms = ComputeCurveSpeedMs(ax, ay, bx, by, cx, cy, lim_ms, min_curve_ms);
								v_allowed = std::min(v_allowed, v_curve_ms);
							}
							v_profile[i] = v_allowed;
						}

						const float current_speed_ms = current_speed_kmh / 3.6f;
						v_profile[0] = std::min(current_speed_ms, v_profile[0]);
						for (size_t i = 1; i < n_samples; ++i)
						{
							const float ds = s_vals[i] - s_vals[i - 1];
							const float v_accel = std::sqrt(v_profile[i - 1] * v_profile[i - 1] + 2.0f * PATH_ETA_ACCEL * ds);
							v_profile[i] = std::min(v_profile[i], v_accel);
						}

						if (reaches_end && path->hasStopPoint())
						{
							v_profile.back() = 0.0f;
						}
						for (size_t idx = n_samples - 1; idx > 0; --idx)
						{
							const size_t i = idx - 1;
							const float ds = s_vals[i + 1] - s_vals[i];
							const float v_brake = std::sqrt(v_profile[i + 1] * v_profile[i + 1] + 2.0f * CURVE_MAX_BRAKE_DECEL * ds);
							v_profile[i] = std::min(v_profile[i], v_brake);
						}

						double total_sec = 0.0;
						for (size_t i = 1; i < n_samples; ++i)
						{
							const float ds = s_vals[i] - s_vals[i - 1];
							const float v_avg = std::max(0.5f, 0.5f * (v_profile[i - 1] + v_profile[i]));
							total_sec += static_cast<double>(ds) / static_cast<double>(v_avg);
						}
						out_seconds = static_cast<float>(total_sec);
					}
				}
			}
		}

		if (err != GPS_ERROR_NONE)
		{
			return err;
		}

		if (!WriteRef(amx, params[5], out_seconds) ||
		    !WriteRef(amx, params[6], out_remaining_dist))
		{
			return GPS_ERROR_INVALID_PARAMS;
		}

		return GPS_ERROR_NONE;
	}
}

template <cell(AMX_NATIVE_CALL *Func)(AMX*, cell*), cell OnError = GPS_ERROR_INTERNAL>
cell SafeNativeCall(AMX* amx, cell* params, const char* name)
{
	try
	{
		return Func(amx, params);
	}
	catch (const std::exception& e)
	{
		GPSLogger::Log("[GPS-Plugin-Plus Exception] %s: %s", name, e.what());
		return OnError;
	}
	catch (...)
	{
		GPSLogger::Log("[GPS-Plugin-Plus Exception] %s: Unknown exception", name);
		return OnError;
	}
}

#define DEFINE_NATIVE_EX(pawn_name, cpp_name, on_error) \
	{ pawn_name, [](AMX* a, cell* p) -> cell { return SafeNativeCall<Natives::cpp_name, (on_error)>(a, p, pawn_name); } }
#define DEFINE_NATIVE(name) DEFINE_NATIVE_EX(#name, name, GPS_ERROR_INTERNAL)
#define DEFINE_NATIVE_BOOL(name) DEFINE_NATIVE_EX(#name, name, 0)

AMX_NATIVE_INFO PluginNatives[] =
{
	DEFINE_NATIVE(CreateMapNode),
	DEFINE_NATIVE(DestroyMapNode),
	DEFINE_NATIVE_BOOL(IsValidMapNode),
	DEFINE_NATIVE(GetMapNodePos),

	DEFINE_NATIVE(GetMapNodeType),
	DEFINE_NATIVE(GetMapNodeFlags),

	DEFINE_NATIVE(CreateConnection),
	DEFINE_NATIVE(DestroyConnection),
	DEFINE_NATIVE(GetConnectionSource),
	DEFINE_NATIVE(GetConnectionTarget),

	DEFINE_NATIVE(GetMapNodeConnectionCount),
	DEFINE_NATIVE(GetMapNodeConnection),
	DEFINE_NATIVE(GetConnectionBetweenMapNodes),

	DEFINE_NATIVE(GetConnectionLaneInfo),
	DEFINE_NATIVE(GetConnectionLanePos),
	DEFINE_NATIVE_BOOL(IsConnectionOneWay),
	DEFINE_NATIVE_BOOL(IsConnectionWrongWay),

	DEFINE_NATIVE(GetDistanceBetweenMapNodes),
	DEFINE_NATIVE(GetAngleBetweenMapNodes),
	DEFINE_NATIVE(GetMapNodeDistanceFromPoint),
	DEFINE_NATIVE(GetMapNodeAngleFromPoint),
	DEFINE_NATIVE(GetClosestMapNodeToPoint),
	DEFINE_NATIVE(GetClosestMapNodeToPointAngle),

	DEFINE_NATIVE(GetHighestMapNodeID),
	DEFINE_NATIVE(GetRandomMapNode),
	DEFINE_NATIVE(GetRandomMapNodeFromArea),
	DEFINE_NATIVE_EX("GetRandomMapNodeInArea", GetRandomMapNodeFromArea, GPS_ERROR_INTERNAL),

	DEFINE_NATIVE(FindPath),
	DEFINE_NATIVE(FindPathThreaded),
	DEFINE_NATIVE(FindPathThreadedEx),
	DEFINE_NATIVE(FindMultiStopPath),

	DEFINE_NATIVE(SaveMapNodesToFile),

	DEFINE_NATIVE_BOOL(IsValidPath),
	DEFINE_NATIVE(GetPathSize),
	DEFINE_NATIVE(GetPathLength),
	DEFINE_NATIVE(GetPathNode),
	DEFINE_NATIVE(GetPathNodeIndex),
	DEFINE_NATIVE(GetPathLaneNodePos),
	DEFINE_NATIVE(DestroyPath),

	DEFINE_NATIVE(SetMapNodeBlocked),
	DEFINE_NATIVE_BOOL(IsMapNodeBlocked),
	DEFINE_NATIVE(SetConnectionBlocked),
	DEFINE_NATIVE_BOOL(IsConnectionBlocked),

	DEFINE_NATIVE(GetClosestPointOnRoad),
	DEFINE_NATIVE(GetRoadPositionInfo),
	DEFINE_NATIVE_BOOL(IsPointInValidRoad),
	DEFINE_NATIVE(IsPointAgainstTraffic),

	DEFINE_NATIVE(GetPathTurnDirection),
	DEFINE_NATIVE(GetPathEstimatedTravelTime),

	DEFINE_NATIVE(GetPathProgress),
	DEFINE_NATIVE(GetPathRemainingInfo),
	DEFINE_NATIVE(GetPathNextManeuver),
	DEFINE_NATIVE(GetSimplifiedPathNodes),
	DEFINE_NATIVE_BOOL(IsMapNodeIntersection),
	DEFINE_NATIVE(GetMapNodeConnectionTarget),

	DEFINE_NATIVE(SetConnectionTrafficMultiplier),
	DEFINE_NATIVE(GetConnectionTrafficMultiplier),
	DEFINE_NATIVE(SetAreaTrafficMultiplier),
	DEFINE_NATIVE(ResetTrafficMultipliers),

	DEFINE_NATIVE(FindInterceptPath),
	DEFINE_NATIVE(GetPurePursuitSteering),

	DEFINE_NATIVE(GetReachableMapNodes),
	DEFINE_NATIVE(GetReachableBoundingBox),

	DEFINE_NATIVE(GetPointZoneName),
	DEFINE_NATIVE(GetMapNodeZoneName),
	DEFINE_NATIVE(GetPathZonesSummary),

	DEFINE_NATIVE(SetPathAutoDestroy),

	DEFINE_NATIVE(SavePathToFile),
	DEFINE_NATIVE(LoadPathFromFile),
	DEFINE_NATIVE(ExportPathToString),
	DEFINE_NATIVE(ImportPathFromString),

	DEFINE_NATIVE(FindPathForVehicleCat),

	DEFINE_NATIVE(SetMapNodeDanger),
	DEFINE_NATIVE(GetMapNodeDanger),
	DEFINE_NATIVE(SetAreaDangerWeight),
	DEFINE_NATIVE(ResetDangerWeights),
	DEFINE_NATIVE(FindSafePath),

	DEFINE_NATIVE(FindEcoPath),
	DEFINE_NATIVE(GetPathElevationStats),

	DEFINE_NATIVE(RegisterTunnelArea),
	DEFINE_NATIVE_BOOL(UnregisterTunnelArea),
	DEFINE_NATIVE(ClearCustomTunnelAreas),
	DEFINE_NATIVE_BOOL(IsPointInTunnel),
	DEFINE_NATIVE_BOOL(IsMapNodeUnderground),
	DEFINE_NATIVE_BOOL(GetPathTunnelSegments),

	DEFINE_NATIVE(FindNearestPointByRoad),

	DEFINE_NATIVE(RegisterRoadRadar),
	DEFINE_NATIVE_BOOL(UnregisterRoadRadar),
	DEFINE_NATIVE(ClearRoadRadars),
	DEFINE_NATIVE_BOOL(GetNextPathRadar),

	DEFINE_NATIVE(GetPathOffsetPosition),

	DEFINE_NATIVE_BOOL(IsMapNodeDeadEnd),
	DEFINE_NATIVE(FindEvasivePath),

	DEFINE_NATIVE(SetConnectionToll),
	DEFINE_NATIVE_BOOL(GetConnectionToll),
	DEFINE_NATIVE(FindPathAvoidTolls),

	DEFINE_NATIVE(GetPathRoundaboutInfo),
	DEFINE_NATIVE(GetSmoothPathPoints),

	DEFINE_NATIVE_BOOL(IsConnectionPaved),
	DEFINE_NATIVE(SetConnectionPaved),
	DEFINE_NATIVE(FindPathPavedOnly),

	DEFINE_NATIVE(GetConnectionRoadClass),
	DEFINE_NATIVE(SetConnectionSpeedLimit),
	DEFINE_NATIVE(GetConnectionSpeedLimit),

	DEFINE_NATIVE(GetPathRejoinPoint),

	DEFINE_NATIVE(SetConnectionMaxHeight),
	DEFINE_NATIVE(GetConnectionMaxHeight),
	DEFINE_NATIVE(SetAreaMaxHeight),
	DEFINE_NATIVE(FindPathWithHeightLimit),

	DEFINE_NATIVE(GetPathLaneTarget),

	DEFINE_NATIVE(FindPathToPoint),
	DEFINE_NATIVE_BOOL(IsPathStopReached),
	DEFINE_NATIVE(GetPathSpeedAdvice),
	DEFINE_NATIVE(GetPathRoadProfile),
	DEFINE_NATIVE(GetPathTimeEstimate),

	{0, 0}
};
