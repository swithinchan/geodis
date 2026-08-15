#!/usr/bin/env python3
"""
generate_district_data.py — Scatter 30 officers and 200 fieldwork sites across
Hong Kong's 18 districts, snapped to real pedestrian-network nodes.

Usage:
    python3 scripts/generate_district_data.py [--graph data/graph.bin]

Outputs (overwrites):
    data/test_officers_30.csv
    data/test_sites_200.csv

Each generated coordinate is an actual node of the LandsD 3D Pedestrian
Network, so every officer/site is reachable and routes follow the network.
"""

import struct
import random
import argparse
import os
import sys

# ── Hong Kong's 18 districts: (name_en, name_tc, lon_min, lat_min, lon_max, lat_max, streets) ──
DISTRICTS = [
    ("Central and Western", "中西區", 114.100, 22.265, 114.165, 22.305,
     ["Connaught Road Central", "Des Voeux Road West", "Queen's Road West",
      "Belcher's Street", "Praya Kennedy Town", "Hollywood Road"]),
    ("Eastern", "東區", 114.195, 22.255, 114.265, 22.305,
     ["King's Road", "Java Road", "Electric Road", "Shau Kei Wan Road", "Chai Wan Road"]),
    ("Southern", "南區", 114.100, 22.195, 114.255, 22.265,
     ["Aberdeen Praya Road", "Wong Chuk Hang Road", "Ap Lei Chau Bridge Road",
      "Stanley Main Street", "Pok Fu Lam Road"]),
    ("Wan Chai", "灣仔區", 114.155, 22.265, 114.195, 22.290,
     ["Hennessy Road", "Lockhart Road", "Queen's Road East", "Gloucester Road"]),
    ("Kowloon City", "九龍城區", 114.165, 22.300, 114.215, 22.350,
     ["Argyle Street", "Prince Edward Road West", "To Kwa Wan Road", "Sung Wong Toi Road"]),
    ("Yau Tsim Mong", "油尖旺區", 114.155, 22.290, 114.185, 22.325,
     ["Nathan Road", "Canton Road", "Austin Road", "Jordan Road", "Kimberley Road"]),
    ("Sham Shui Po", "深水埗區", 114.140, 22.315, 114.175, 22.350,
     ["Cheung Sha Wan Road", "Lai Chi Kok Road", "Tai Po Road", "Pei Ho Street"]),
    ("Wong Tai Sin", "黃大仙區", 114.185, 22.325, 114.225, 22.370,
     ["Lung Cheung Road", "Choi Hung Road", "Fung Tak Road", "Po Kong Village Road"]),
    ("Kwun Tong", "觀塘區", 114.210, 22.300, 114.265, 22.335,
     ["Kwun Tong Road", "Hip Wo Street", "Ngau Tau Kok Road", "Lei Yue Mun Road"]),
    ("Kwai Tsing", "葵青區", 114.080, 22.335, 114.155, 22.385,
     ["Kwai Chung Road", "Tsing Yi Road", "Cheung Hong Street", "Hing Fong Road"]),
    ("Tsuen Wan", "荃灣區", 114.080, 22.355, 114.145, 22.405,
     ["Castle Peak Road", "Tai Ho Road", "Sha Tsui Road", "Yeung Uk Road"]),
    ("Tuen Mun", "屯門區", 113.925, 22.360, 113.995, 22.425,
     ["Tuen Mun Heung Sze Wui Road", "Tsing Wun Road", "Wu Chui Road", "Pui To Road"]),
    ("Yuen Long", "元朗區", 113.995, 22.420, 114.065, 22.475,
     ["Yuen Long Main Road", "Kau Yuk Road", "Tai Tong Road", "Ma Tin Road"]),
    ("North", "北區", 114.100, 22.470, 114.185, 22.555,
     ["Sha Tau Kok Road", "San Fung Avenue", "Luen On Street", "Fanling Station Road"]),
    ("Tai Po", "大埔區", 114.130, 22.415, 114.205, 22.485,
     ["Kwong Fuk Road", "Tai Po Tai Wo Road", "On Chee Road", "Ting Kok Road"]),
    ("Sha Tin", "沙田區", 114.145, 22.345, 114.225, 22.425,
     ["Sha Tin Centre Street", "Tai Po Road", "Tin Sam Street", "Kong Pui Street"]),
    ("Sai Kung", "西貢區", 114.220, 22.295, 114.330, 22.405,
     ["Po Tung Road", "Hiram's Highway", "Fuk Man Road", "Man Nin Street"]),
    ("Islands", "離島區", 113.845, 22.150, 114.055, 22.305,
     ["Tung Chung Road", "Cheung Chau Praya Road", "Peng Chau Wing On Street", "Mui Wo Ferry Pier Road"]),
]

ESTATES = [
    "Belcher Gardens", "Metro Harbour View", "City One Shatin", "Kingswood Villas",
    "South Horizons", "Laguna City", "Mei Foo Sun Chuen", "Tuen Mun Town Plaza",
    "The Pavilia Hill", "Maritime Bay", "Festival City", "Sunshine City",
    "Park Island", "Tai Po Centre", "New Town Plaza", "Waldorf Garden",
]

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

def pick_node_in_district(nodes, dist, rng):
    """Pick a random network node inside a district bbox."""
    name, tc, lon_min, lat_min, lon_max, lat_max, streets = dist
    candidates = [n for n in nodes
                  if lon_min <= n[0] <= lon_max and lat_min <= n[1] <= lat_max]
    if not candidates:
        # fall back: nearest node to bbox centre
        clon = (lon_min + lon_max) / 2.0
        clat = (lat_min + lat_max) / 2.0
        candidates = [min(nodes, key=lambda n: (n[0]-clon)**2 + (n[1]-clat)**2)]
        candidates = [candidates]
    return rng.choice(candidates), streets

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--graph', default='data/graph.bin')
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    graph_path = os.path.join(root, args.graph) if not os.path.isabs(args.graph) else args.graph

    rng = random.Random(args.seed)
    print(f'Reading network nodes from {graph_path}...', file=sys.stderr)
    nodes = read_nodes(graph_path)
    print(f'  {len(nodes):,} nodes', file=sys.stderr)

    # Pre-index nodes by district (once, for speed)
    district_nodes = []
    for dist in DISTRICTS:
        name, tc, lon_min, lat_min, lon_max, lat_max, streets = dist
        in_dist = [n for n in nodes if lon_min <= n[0] <= lon_max and lat_min <= n[1] <= lat_max]
        district_nodes.append(in_dist)
        print(f'  {name}: {len(in_dist):,} nodes', file=sys.stderr)

    # ── Distribute 30 officers (1-2 per district, round-robin) ─────────────
    officer_plan = []
    for i in range(30):
        officer_plan.append(i % len(DISTRICTS))

    with open(os.path.join(root, 'data', 'test_officers_30.csv'), 'w') as f:
        f.write('name,address,lon,lat,z\n')
        for i, didx in enumerate(officer_plan):
            dist = DISTRICTS[didx]
            cands = district_nodes[didx]
            if not cands:
                # skip district with no nodes; re-assign to a district with nodes
                didx = next(d for d in range(len(DISTRICTS)) if district_nodes[d])
                dist = DISTRICTS[didx]
                cands = district_nodes[didx]
            lon, lat, z = rng.choice(cands)
            street = rng.choice(dist[6])
            estate = rng.choice(ESTATES)
            flat = rng.randint(1, 40)
            floor = rng.randint(1, 35)
            addr = f"Flat {flat}, Floor {floor}, {estate}, {street}, {dist[0]}"
            f.write(f'Officer_{i+1:02d},"{addr}",{lon:.8f},{lat:.8f},{z:.1f}\n')
    print('Wrote data/test_officers_30.csv', file=sys.stderr)

    # ── Distribute 200 sites (~11 per district, round-robin) ───────────────
    site_plan = [i % len(DISTRICTS) for i in range(200)]

    with open(os.path.join(root, 'data', 'test_sites_200.csv'), 'w') as f:
        f.write('name,address,lon,lat,z\n')
        for i, didx in enumerate(site_plan):
            dist = DISTRICTS[didx]
            cands = district_nodes[didx]
            if not cands:
                didx = next(d for d in range(len(DISTRICTS)) if district_nodes[d])
                dist = DISTRICTS[didx]
                cands = district_nodes[didx]
            lon, lat, z = rng.choice(cands)
            street = rng.choice(dist[6])
            estate = rng.choice(ESTATES)
            block = rng.choice('ABCDEFGH')
            floor = rng.randint(1, 30)
            addr = f"Block {block}, Floor {floor}, {estate}, {street}, {dist[0]}"
            f.write(f'TPU-{rng.randint(100,999)}-Site-{i+1:03d},"{addr}",{lon:.8f},{lat:.8f},{z:.1f}\n')
    print('Wrote data/test_sites_200.csv', file=sys.stderr)
    print('Done!', file=sys.stderr)

if __name__ == '__main__':
    main()
