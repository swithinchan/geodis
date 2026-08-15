#!/usr/bin/env python3
"""
generate_district_data.py — Scatter 30 officers and 200 fieldwork sites across
Hong Kong's 18 districts, snapped to real pedestrian-network nodes.

Street names come from the government Road Centreline dataset
(Transportation_RoadCentreline_*.geojson), grouped by district via
build_street_districts.py → data/street_names_by_district.json.
Addresses use real street names only (no estate/tower names).

Usage:
    python3 scripts/generate_district_data.py [--graph data/graph.bin]

Outputs (overwrites):
    data/test_officers_30.csv
    data/test_sites_200.csv
"""

import struct
import random
import argparse
import json
import os
import sys

# ── 18 districts: (display_name, name_tc, lon_min, lat_min, lon_max, lat_max) ──
DISTRICTS = [
    ("Central and Western", "中西區", 114.100, 22.265, 114.165, 22.305),
    ("Eastern", "東區", 114.195, 22.255, 114.265, 22.305),
    ("Southern", "南區", 114.100, 22.195, 114.255, 22.265),
    ("Wan Chai", "灣仔區", 114.155, 22.265, 114.195, 22.290),
    ("Kowloon City", "九龍城區", 114.165, 22.300, 114.215, 22.350),
    ("Yau Tsim Mong", "油尖旺區", 114.155, 22.290, 114.185, 22.325),
    ("Sham Shui Po", "深水埗區", 114.140, 22.315, 114.175, 22.350),
    ("Wong Tai Sin", "黃大仙區", 114.185, 22.325, 114.225, 22.370),
    ("Kwun Tong", "觀塘區", 114.210, 22.300, 114.265, 22.335),
    ("Kwai Tsing", "葵青區", 114.080, 22.335, 114.155, 22.385),
    ("Tsuen Wan", "荃灣區", 114.080, 22.355, 114.145, 22.405),
    ("Tuen Mun", "屯門區", 113.925, 22.360, 113.995, 22.425),
    ("Yuen Long", "元朗區", 113.995, 22.420, 114.065, 22.475),
    ("North", "北區", 114.100, 22.470, 114.185, 22.555),
    ("Tai Po", "大埔區", 114.130, 22.415, 114.205, 22.485),
    ("Sha Tin", "沙田區", 114.145, 22.345, 114.225, 22.425),
    ("Sai Kung", "西貢區", 114.220, 22.295, 114.330, 22.405),
    ("Islands", "離島區", 113.845, 22.150, 114.055, 22.305),
]

def load_street_names(root):
    """Load {district: [street names]} from the government-derived JSON."""
    path = os.path.join(root, 'data', 'street_names_by_district.json')
    if not os.path.exists(path):
        print(f'Warning: {path} not found — run build_street_districts.py first', file=sys.stderr)
        return {}
    with open(path, encoding='utf-8') as f:
        return json.load(f)

def read_nodes(graph_path):
    """Read all node coordinates from graph.bin → list of (lon, lat, z)."""
    nodes = []
    with open(graph_path, 'rb') as f:
        magic = f.read(8)
        assert magic == b'GEODISG\x02', f'bad magic {magic}'
        f.read(4)  # version
        num_nodes = struct.unpack('<Q', f.read(8))[0]
        f.read(8)  # num_edges
        f.read(4)  # reserved
        for _ in range(num_nodes):
            lon, lat, z, _flags = struct.unpack('<dddI', f.read(28))
            nodes.append((lon, lat, z))
    return nodes

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--graph', default='data/graph.bin')
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    graph_path = os.path.join(root, args.graph) if not os.path.isabs(args.graph) else args.graph

    rng = random.Random(args.seed)
    street_names = load_street_names(root)

    print(f'Reading network nodes from {graph_path}...', file=sys.stderr)
    nodes = read_nodes(graph_path)
    print(f'  {len(nodes):,} nodes', file=sys.stderr)

    # Pre-index nodes by district bbox
    district_nodes = []
    for dist in DISTRICTS:
        name, tc, lon_min, lat_min, lon_max, lat_max = dist
        in_dist = [n for n in nodes if lon_min <= n[0] <= lon_max and lat_min <= n[1] <= lat_max]
        district_nodes.append(in_dist)
        nstreets = len(street_names.get(name, []))
        print(f'  {name}: {len(in_dist):,} nodes, {nstreets} street names', file=sys.stderr)

    # ── 30 officers (1-2 per district, round-robin) ─────────────────────────
    officer_plan = [i % len(DISTRICTS) for i in range(30)]
    with open(os.path.join(root, 'data', 'test_officers_30.csv'), 'w') as f:
        f.write('name,address,lon,lat,z\n')
        for i, didx in enumerate(officer_plan):
            name, tc, *_ = DISTRICTS[didx]
            cands = district_nodes[didx]
            streets = street_names.get(name, [])
            if not cands:
                didx = next(d for d in range(len(DISTRICTS)) if district_nodes[d])
                name = DISTRICTS[didx][0]
                cands = district_nodes[didx]
                streets = street_names.get(name, [])
            lon, lat, z = rng.choice(cands)
            street = rng.choice(streets) if streets else name
            addr = f"{street}, {name}"
            f.write(f'Officer_{i+1:02d},"{addr}",{lon:.8f},{lat:.8f},{z:.1f}\n')
    print('Wrote data/test_officers_30.csv', file=sys.stderr)

    # ── 200 sites (~11 per district, round-robin) ────────────────────────────
    site_plan = [i % len(DISTRICTS) for i in range(200)]
    with open(os.path.join(root, 'data', 'test_sites_200.csv'), 'w') as f:
        f.write('name,address,lon,lat,z\n')
        for i, didx in enumerate(site_plan):
            name, tc, *_ = DISTRICTS[didx]
            cands = district_nodes[didx]
            streets = street_names.get(name, [])
            if not cands:
                didx = next(d for d in range(len(DISTRICTS)) if district_nodes[d])
                name = DISTRICTS[didx][0]
                cands = district_nodes[didx]
                streets = street_names.get(name, [])
            lon, lat, z = rng.choice(cands)
            street = rng.choice(streets) if streets else name
            addr = f"{street}, {name}"
            f.write(f'TPU-{rng.randint(100,999)}-Site-{i+1:03d},"{addr}",{lon:.8f},{lat:.8f},{z:.1f}\n')
    print('Wrote data/test_sites_200.csv', file=sys.stderr)
    print('Done!', file=sys.stderr)

if __name__ == '__main__':
    main()
