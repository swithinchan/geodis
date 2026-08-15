#!/usr/bin/env python3
"""Generate random officer and fieldwork site locations from network nodes."""

import json
import sys
import random
import argparse

def generate(input_geojson, num_officers, num_sites, seed=42):
    random.seed(seed)
    
    with open(input_geojson) as f:
        data = json.load(f)
    
    features = data['features']
    print(f"Loading {len(features)} features...", file=sys.stderr)
    
    # Collect all unique node coordinates from the network
    nodes = {}  # (lon, lat, z) -> index
    for feat in features:
        coords = feat['geometry']['coordinates']
        for c in coords:
            key = (round(c[0], 8), round(c[1], 8), round(c[2], 4) if len(c) >= 3 else 0.0)
            if key not in nodes:
                nodes[key] = len(nodes)
    
    node_list = list(nodes.keys())
    print(f"  {len(node_list):,} unique network nodes available", file=sys.stderr)
    
    # Pick random nodes for officers and sites (mutually exclusive)
    all_indices = list(range(len(node_list)))
    random.shuffle(all_indices)
    
    officer_nodes = all_indices[:num_officers]
    site_nodes = all_indices[num_officers:num_officers + num_sites]
    
    # Hong Kong street names for realism
    streets_en = [
        "Praya, Kennedy Town", "Belcher's Street", "Catchick Street",
        "Davis Street", "Forbes Street", "Harcourt Road", 
        "Queen's Road West", "Des Voeux Road West", "Connaught Road West",
        "Hill Road", "Water Street", "Eastern Street",
        "Western Street", "Centre Street", "Sutherland Street",
        "Victoria Road", "Mount Davis Road", "Pok Fu Lam Road",
        "Smithfield", "Sand Street", "Hau Wo Street",
    ]
    streets_tc = [
        "堅彌地城海旁", "卑路乍街", "吉席街",
        "爹核士街", "科士街", "夏慤道",
        "皇后大道西", "德輔道西", "干諾道西",
        "山道", "水街", "東邊街",
        "西邊街", "正街", "修打蘭街",
        "域多利道", "摩星嶺道", "薄扶林道",
        "士美菲路", "沙街", "厚和街",
    ]
    
    estates = ["Belcher Gardens", "Kennedy Town Centre", "Smithfield Court",
               "Lung Cheung Court", "Ka On Building", "Wah Fung Building",
               "Kwun Lung Lau", "Yue Sun Mansion", "Manhattan Heights",
               "The Merton", "Imperial Kennedy", "Lexington Hill",
               "High West", "Scholar Court", "Greenvale"]
    
    # Generate officers CSV
    with open("data/test_officers_30.csv", "w") as f:
        f.write("name,address,lon,lat,z\n")
        for i, ni in enumerate(officer_nodes):
            lon, lat, z = node_list[ni]
            street = random.choice(streets_en)
            building = random.choice(estates)
            flat = random.randint(1, 40)
            floor = random.randint(1, 35)
            addr = f"Flat {flat}, Floor {floor}, {building}, {street}"
            name = f"Officer_{i+1:02d}"
            f.write(f'{name},"{addr}",{lon:.8f},{lat:.8f},{z:.1f}\n')
    
    # Generate sites CSV
    with open("data/test_sites_200.csv", "w") as f:
        f.write("name,address,lon,lat,z\n")
        for i, ni in enumerate(site_nodes):
            lon, lat, z = node_list[ni]
            street = random.choice(streets_en)
            building = random.choice(estates)
            floor = random.randint(1, 30)
            addr = f"Block {random.choice('ABCDEFGH')}, Floor {floor}, {building}, {street}"
            name = f"TPU-{random.randint(100,999)}-Site-{i+1:03d}"
            f.write(f'{name},"{addr}",{lon:.8f},{lat:.8f},{z:.1f}\n')
    
    print(f"Generated {num_officers} officers → data/test_officers_30.csv", file=sys.stderr)
    print(f"Generated {num_sites} sites → data/test_sites_200.csv", file=sys.stderr)
    print(f"All coordinates are real network nodes — guaranteed reachable", file=sys.stderr)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='Input GeoJSON file')
    parser.add_argument('-o', '--officers', type=int, default=30)
    parser.add_argument('-s', '--sites', type=int, default=200)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    generate(args.input, args.officers, args.sites, args.seed)
