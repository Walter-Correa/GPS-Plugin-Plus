#!/usr/bin/env python3
"""
GTA San Andreas Node Extractor for GPS-Plugin-Plus
Extracts vehicle nodes, boat nodes, pedestrian nodes (sidewalks), real lane widths (5.4 m),
center-divider gaps (width_byte / 16.0), and edge-oriented lane counts from original GTA San Andreas
binary files (NODES0.DAT to NODES63.DAT) into GPS.dat format.

GPS.dat edge convention (directed source -> target):
  1 <source> <target> <direction> <road_width_m> <left_lanes> <right_lanes> <oneway>
  - right_lanes: lanes flowing in the source -> target direction (forward).
  - left_lanes:  lanes flowing in the target -> source direction (opposite).
  - oneway = 1:  wrong-way edge has right_lanes = 0, left_lanes > 0 (penalized by pathfinder).
  - two-way:     nodes lie on the road centerline (two-way divider); gap = wc = nw / 16.0.
  - one-way:     nodes lie on the carriageway centerline; road_width = n * 5.4 m.
"""

import os
import sys
import struct
import argparse
import time


PATH_TYPE_VEHICLE = 0
PATH_TYPE_PEDESTRIAN = 1
PATH_TYPE_BOAT = 2

LANE_WIDTH = 5.4

NODE_FLAG_NONE = 0
NODE_FLAG_HIGHWAY = 1 << 0
NODE_FLAG_EMERGENCY_ONLY = 1 << 1
NODE_FLAG_ROADBLOCK = 1 << 2


def extract_nodes(paths_dir, output_file):
    print(f"[Extractor] Reading GTA SA path files from: {paths_dir}")
    if not os.path.exists(paths_dir):
        print(f"[Error] Directory not found: {paths_dir}")
        return False

    t_start = time.time()
    area_data = {}
    num_veh_map = {}
    num_ped_map = {}
    veh_base = {}
    ped_base = {}

    curr_veh = 0
    curr_ped = 0

    # Step 1: Read all 64 area files
    for area_id in range(64):
        file_name = f"NODES{area_id}.DAT"
        file_path = os.path.join(paths_dir, file_name)

        if not os.path.exists(file_path):
            file_name = f"nodes{area_id}.dat"
            file_path = os.path.join(paths_dir, file_name)
            if not os.path.exists(file_path):
                continue

        with open(file_path, "rb") as f:
            header = f.read(20)
            if len(header) < 20:
                continue

            num_nodes, num_veh, num_ped, num_navi, num_links = struct.unpack("<5I", header)
            sec1 = f.read(num_nodes * 28)
            sec2 = f.read(num_navi * 14)
            sec3 = f.read(num_links * 4)
            sec4 = f.read(768)  # Filler
            sec5 = f.read(num_links * 2) if num_links > 0 else b""

            area_data[area_id] = {
                "num_nodes": num_nodes,
                "num_veh": num_veh,
                "num_ped": num_ped,
                "num_navi": num_navi,
                "num_links": num_links,
                "sec1": sec1,
                "sec2": sec2,
                "sec3": sec3,
                "sec5": sec5
            }

            num_veh_map[area_id] = num_veh
            num_ped_map[area_id] = num_ped
            veh_base[area_id] = curr_veh
            ped_base[area_id] = curr_ped

            curr_veh += num_veh
            curr_ped += num_ped

    total_veh_nodes = curr_veh  # Exactly 30,587 in vanilla GTA SA
    total_ped_nodes = curr_ped  # Exactly 37,650 in vanilla GTA SA
    print(f"[Extractor] Loaded {len(area_data)} areas. Vehicle nodes: {total_veh_nodes}, Ped nodes: {total_ped_nodes}")

    total_car = 0
    total_boat = 0
    total_ped = 0
    total_connections = 0
    node_positions = {}
    seen_edges = set()

    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)
    with open(output_file, "w", encoding="ascii") as out:
        # Header line (compatible with original GPS.dat header: 3 174 1)
        out.write("3 174 1\n")

        # Step 2: Output nodes
        # Vehicle & Boat nodes first (IDs 0 .. total_veh_nodes - 1)
        for area_id in sorted(area_data.keys()):
            data = area_data[area_id]
            num_veh = data["num_veh"]
            sec1 = data["sec1"]
            base_v = veh_base[area_id]

            for local_id in range(num_veh):
                raw = sec1[local_id * 28 : (local_id + 1) * 28]
                _, _, x, y, z, _, _, _, _, _, _, flags = struct.unpack("<IIhhhHHHHBBI", raw)

                world_x = x / 8.0
                world_y = y / 8.0
                world_z = z / 8.0

                is_boat = bool(flags & (1 << 7))
                is_emergency = bool(flags & (1 << 8))
                is_highway = bool(flags & (1 << 13))
                is_roadblock = bool(flags & (1 << 6))

                if is_boat:
                    node_type = PATH_TYPE_BOAT
                    total_boat += 1
                else:
                    node_type = PATH_TYPE_VEHICLE
                    total_car += 1

                node_flags = NODE_FLAG_NONE
                if is_highway:
                    node_flags |= NODE_FLAG_HIGHWAY
                if is_emergency:
                    node_flags |= NODE_FLAG_EMERGENCY_ONLY
                if is_roadblock:
                    node_flags |= NODE_FLAG_ROADBLOCK

                global_id = base_v + local_id
                node_positions[global_id] = (world_x, world_y)

                # Line format: 0 x y z ignore id type flags
                out.write(f"0 {world_x:.6f} {world_y:.6f} {world_z:.6f} -1 {global_id} {node_type} {node_flags}\n")

        # Pedestrian nodes second (IDs total_veh_nodes .. total_veh_nodes + total_ped_nodes - 1)
        for area_id in sorted(area_data.keys()):
            data = area_data[area_id]
            num_nodes = data["num_nodes"]
            num_veh = data["num_veh"]
            num_ped = data["num_ped"]
            sec1 = data["sec1"]
            base_p = ped_base[area_id]

            for p_idx in range(num_ped):
                local_id = num_veh + p_idx
                raw = sec1[local_id * 28 : (local_id + 1) * 28]
                _, _, x, y, z, _, _, _, _, _, _, flags = struct.unpack("<IIhhhHHHHBBI", raw)

                world_x = x / 8.0
                world_y = y / 8.0
                world_z = z / 8.0

                node_type = PATH_TYPE_PEDESTRIAN
                node_flags = NODE_FLAG_NONE
                total_ped += 1

                global_id = total_veh_nodes + base_p + p_idx
                node_positions[global_id] = (world_x, world_y)

                # Line format: 0 x y z ignore id type flags
                out.write(f"0 {world_x:.6f} {world_y:.6f} {world_z:.6f} -1 {global_id} {node_type} {node_flags}\n")

        # Step 3: Output connections
        for area_id in sorted(area_data.keys()):
            data = area_data[area_id]
            num_nodes = data["num_nodes"]
            num_veh = data["num_veh"]
            sec1 = data["sec1"]
            sec3 = data["sec3"]
            sec5 = data["sec5"]
            base_v = veh_base[area_id]
            base_p = ped_base[area_id]

            for local_id in range(num_nodes):
                raw = sec1[local_id * 28 : (local_id + 1) * 28]
                _, _, _, _, _, _, link_id, _, _, width_byte, _, flags = struct.unpack("<IIhhhHHHHBBI", raw)

                num_links = flags & 0x0F
                is_ped = (local_id >= num_veh)
                is_boat = bool(flags & (1 << 7))

                if is_ped:
                    source_global_id = total_veh_nodes + base_p + (local_id - num_veh)
                else:
                    source_global_id = base_v + local_id

                for l_idx in range(num_links):
                    link_pos = (link_id + l_idx) * 4
                    if link_pos + 4 > len(sec3):
                        continue

                    target_area, target_local = struct.unpack("<HH", sec3[link_pos : link_pos + 4])
                    if target_area not in num_veh_map:
                        continue

                    target_nv = num_veh_map[target_area]
                    if target_local < target_nv:
                        target_global_id = veh_base[target_area] + target_local
                    else:
                        target_global_id = total_veh_nodes + ped_base[target_area] + (target_local - target_nv)

                    edge_key = (source_global_id, target_global_id)
                    if edge_key in seen_edges:
                        continue
                    seen_edges.add(edge_key)

                    # Determine road geometry and lanes
                    if is_ped:
                        road_width = 2.5
                        left_lanes = 1
                        right_lanes = 1
                        oneway = 0
                    elif is_boat:
                        road_width = 15.0
                        left_lanes = 1
                        right_lanes = 1
                        oneway = 0
                    else:
                        # Vehicle lane extraction via Section 5 & Section 2 (Navi Nodes)
                        wc = width_byte / 16.0
                        left_lanes = 1
                        right_lanes = 1
                        oneway = 0
                        road_width = 2.0 * wc + 2.0 * LANE_WIDTH

                        src_pos = node_positions.get(source_global_id, (0.0, 0.0))
                        tgt_pos = node_positions.get(target_global_id, (0.0, 0.0))
                        ex = tgt_pos[0] - src_pos[0]
                        ey = tgt_pos[1] - src_pos[1]

                        navi_pos = (link_id + l_idx) * 2
                        if len(sec5) >= navi_pos + 2:
                            navi_ref, = struct.unpack("<H", sec5[navi_pos : navi_pos + 2])
                            if navi_ref != 0xFFFF:
                                navi_id = navi_ref & 0x3FF
                                navi_area = (navi_ref >> 10) & 0x3F
                                if navi_area in area_data and navi_id < area_data[navi_area]["num_navi"]:
                                    raw_navi = area_data[navi_area]["sec2"][navi_id * 14 : (navi_id + 1) * 14]
                                    if len(raw_navi) >= 14:
                                        _, _, navi_area_tgt, navi_node_tgt, navi_dx, navi_dy, navi_flags = struct.unpack("<hhHHbbI", raw_navi)
                                        nl = (navi_flags >> 8) & 0x07   # lanes along navi vector
                                        nr = (navi_flags >> 11) & 0x07  # lanes in opposite direction
                                        nw = navi_flags & 0xFF

                                        if nw > 0:
                                            wc = nw / 16.0

                                        navi_tgt = None
                                        if navi_area_tgt in num_veh_map:
                                            nv_ = num_veh_map[navi_area_tgt]
                                            navi_tgt = (veh_base[navi_area_tgt] + navi_node_tgt) if navi_node_tgt < nv_ else (total_veh_nodes + ped_base[navi_area_tgt] + (navi_node_tgt - nv_))
                                        if navi_tgt == target_global_id:
                                            along = True
                                        elif navi_tgt == source_global_id:
                                            along = False
                                        else:
                                            along = (ex * navi_dx + ey * navi_dy) > 0

                                        fwd = nl if along else nr   # lanes along source -> target
                                        back = nr if along else nl  # lanes in opposite direction

                                        if fwd > 0 or back > 0:
                                            right_lanes = fwd
                                            left_lanes = back
                                            oneway = 1 if (fwd == 0 or back == 0) else 0

                                        if oneway:
                                            road_width = (left_lanes + right_lanes) * LANE_WIDTH
                                        else:
                                            road_width = 2.0 * wc + (left_lanes + right_lanes) * LANE_WIDTH

                    # Line format: 1 id id2 direction width left_lanes right_lanes oneway
                    out.write(f"1 {source_global_id} {target_global_id} 0 {road_width:.2f} {left_lanes} {right_lanes} {oneway}\n")
                    total_connections += 1

    elapsed = time.time() - t_start
    print(f"[Extractor] Finished in {elapsed:.2f}s!")
    print(f"  Total Vehicle Nodes: {total_car}")
    print(f"  Total Boat Nodes:    {total_boat}")
    print(f"  Total Ped Nodes:     {total_ped}")
    print(f"  Total Nodes Output:  {total_car + total_boat + total_ped}")
    print(f"  Total Connections:   {total_connections}")
    print(f"  Output saved to:     {output_file}")
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract GTA SA Path Nodes into GPS.dat")
    parser.add_argument("--paths", default=r"D:\JOGOS\GTA San Andreas - Sem Mods\data\paths", help="Path to GTA SA data/paths directory")
    parser.add_argument("--output", default="GPS.dat", help="Output file path")
    args = parser.parse_args()

    extract_nodes(args.paths, args.output)
