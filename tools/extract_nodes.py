#!/usr/bin/env python3
"""
GTA San Andreas Node Extractor for GPS-Plugin-Plus
Extracts vehicle nodes, boat nodes, pedestrian nodes (sidewalks), road widths, and lane counts
from original GTA San Andreas binary files (NODES0.DAT to NODES63.DAT) into GPS.dat format.
"""

import os
import sys
import struct
import argparse
import time


PATH_TYPE_VEHICLE = 0
PATH_TYPE_PEDESTRIAN = 1
PATH_TYPE_BOAT = 2

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
                        road_width = max(3.5, float(width_byte)) if width_byte > 0 else 8.0
                        left_lanes = 1
                        right_lanes = 1
                        oneway = 0

                        navi_pos = (link_id + l_idx) * 2
                        if len(sec5) >= navi_pos + 2:
                            navi_ref, = struct.unpack("<H", sec5[navi_pos : navi_pos + 2])
                            if navi_ref != 0xFFFF:
                                navi_id = navi_ref & 0x3FF
                                navi_area = (navi_ref >> 10) & 0x3F
                                if navi_area in area_data and navi_id < area_data[navi_area]["num_navi"]:
                                    raw_navi = area_data[navi_area]["sec2"][navi_id * 14 : (navi_id + 1) * 14]
                                    if len(raw_navi) >= 14:
                                        _, _, _, _, _, _, navi_flags = struct.unpack("<hhHHbbI", raw_navi)
                                        nl = (navi_flags >> 8) & 0x07
                                        nr = (navi_flags >> 11) & 0x07
                                        nw = navi_flags & 0xFF

                                        if nl > 0 or nr > 0:
                                            left_lanes = nl
                                            right_lanes = nr
                                            if nl == 0 and nr > 0:
                                                oneway = 1
                                            elif nr == 0 and nl > 0:
                                                oneway = 1

                                        if nw > 0:
                                            road_width = float(nw)
                                        elif width_byte > 0:
                                            road_width = float(width_byte)
                                        else:
                                            total_l = max(1, left_lanes + right_lanes)
                                            road_width = total_l * 3.5

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
