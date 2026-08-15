#!/usr/bin/env python3
"""
preprocess.py — Convert HK 3D Pedestrian Network GeoJSON to binary graph format.

Usage:
    python preprocess.py pedestrian_route.json -o graph.bin

The 3D Pedestrian Network data can be downloaded from:
  https://portal.csdi.gov.hk/csdi-webpage/file-api?dataset_id=landsd_rcd_1637222018065_52265&format=geojson&layer_name=PedestrianRoute

Or via ArcGIS REST API:
  https://portal.csdi.gov.hk/server/rest/services/common/landsd_rcd_1637222018065_52265/MapServer/0/query?where=1%3D1&outFields=*&returnGeometry=true&f=geojson

Binary format produced (little-endian):
  Header:  magic[8] ("GEODISG\x02") + version[4] + num_nodes[8] + num_edges[8] + flags[4]
  Nodes:  x[8] + y[8] + z[8] + flags[4]    per node
  Edges:  from[8] + to[8] + len_2d[8] + len_3d[8] + ascent[8] + descent[8] + grade[8] + flags[4]  per edge

The script:
  1. Reads GeoJSON LineString features
  2. Extracts unique endpoints as nodes
  3. Creates directed edges (both directions for "Both Ways")
  4. Computes 3D length and ascent/descent from 3D coordinates
  5. Writes the binary graph
"""

import json
import struct
import sys
import argparse
import os
from collections import defaultdict

MAGIC = b"GEODISG\x02"
VERSION = 2

# ─── FeatureType codes (from ArcGIS domain) ──────────────────────────────────
FEATURE_TYPE = {
    "Footway": 0,
    "Footbridge": 1,
    "Subway": 2,
    "Stairs": 3,
    "Lift": 4,
    "Escalator": 5,
    "Crossing": 6,
    "Generalized Walkway inside Park": 7,
    "Generalized Walkway inside Site": 8,
    "Walkway inside Building": 9,
    "Cycletrack": 10,
}

DIRECTION = {
    "Both Ways": 0,
    "One Way (Digitizing Direction)": 1,
    "One Way (Reverse of Digitizing Direction)": 2,
}

LOCATION = {
    "Outdoor": 1,
    "Indoor": 2,
}

# Node/edge flag bits
NODE_FLAG_JUNCTION    = 0
NODE_FLAG_STAIR_UP    = 1 << 0
NODE_FLAG_STAIR_DOWN  = 1 << 1
NODE_FLAG_LIFT        = 1 << 2
NODE_FLAG_CROSSING    = 1 << 3
NODE_FLAG_ENTRANCE    = 1 << 4
NODE_FLAG_INDOOR      = 1 << 5
NODE_FLAG_BARRIER     = 1 << 6

EDGE_FLAG_STAIRS      = 1 << 0
EDGE_FLAG_LIFT        = 1 << 1
EDGE_FLAG_ESCALATOR   = 1 << 2
EDGE_FLAG_FOOTBRIDGE  = 1 << 3
EDGE_FLAG_SUBWAY      = 1 << 4
EDGE_FLAG_CROSSING    = 1 << 5
EDGE_FLAG_INDOOR      = 1 << 6
EDGE_FLAG_COVERED     = 1 << 7
EDGE_FLAG_STEEP       = 1 << 8
EDGE_FLAG_BARRIER     = 1 << 9


def distance_3d(p1, p2):
    """Euclidean 3D distance in meters (coordinates are in degrees but we approximate)."""
    import math
    dx = (p2[0] - p1[0]) * 111320.0 * math.cos(math.radians((p1[1] + p2[1]) / 2))
    dy = (p2[1] - p1[1]) * 110540.0
    z1 = p1[2] if len(p1) >= 3 else 0.0
    z2 = p2[2] if len(p2) >= 3 else 0.0
    dz = z2 - z1
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def process_geojson(input_path, output_path, progress_interval=50000):
    """Convert GeoJSON pedestrian network to binary graph."""
    print(f"Reading {input_path}...")

    with open(input_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    features = data.get('features', [])
    n_total = len(features)
    print(f"  {n_total:,} features")

    # ── Pass 1: collect all unique coordinates as nodes ──────────────────────
    # Use (x, y, z) tuple as key
    coord_to_node = {}
    nodes = []  # list of (x, y, z, flags)

    def get_or_create_node(x, y, z):
        key = (round(x, 10), round(y, 10), round(z, 4))
        if key not in coord_to_node:
            idx = len(nodes)
            coord_to_node[key] = idx
            nodes.append((x, y, z, NODE_FLAG_JUNCTION))
        return coord_to_node[key]

    # ── Pass 2: build edges ─────────────────────────────────────────────────
    edges = []  # (from_node, to_node, len_2d, len_3d, ascent, descent, grade, flags)
    skipped = 0

    for i, feat in enumerate(features):
        if i > 0 and i % progress_interval == 0:
            print(f"  Processing feature {i:,}/{n_total:,} ({100*i/n_total:.1f}%)...")

        props = feat.get('properties', {})
        geom = feat.get('geometry', {})
        geom_type = geom.get('type', 'LineString')
        raw_coords = geom.get('coordinates', [])

        # Normalise LineString / MultiLineString into a list of lines
        if geom_type == 'MultiLineString':
            lines = [c for c in raw_coords if len(c) >= 2]
        else:
            lines = [raw_coords] if len(raw_coords) >= 2 else []

        if not lines:
            skipped += 1
            continue

        # Determine edge flags
        ft = props.get('FeatureType', 'Footway')
        loc = props.get('Location', 'Outdoor')
        wp = props.get('WeatherProof', 'NonCovered')
        wb = props.get('WheelchairBarrier', 'False')
        direction = props.get('Direction', 'Both Ways')
        gradient = props.get('Gradient', 0) or 0

        edge_flags = 0
        ft_l = ft.lower()
        if 'stair' in ft_l:
            edge_flags |= EDGE_FLAG_STAIRS
        elif 'lift' in ft_l:
            edge_flags |= EDGE_FLAG_LIFT
        elif 'escalator' in ft_l or 'travelator' in ft_l:
            edge_flags |= EDGE_FLAG_ESCALATOR
        elif 'footbridge' in ft_l:
            edge_flags |= EDGE_FLAG_FOOTBRIDGE
        elif 'subway' in ft_l:
            edge_flags |= EDGE_FLAG_SUBWAY
        elif 'crossing' in ft_l:
            edge_flags |= EDGE_FLAG_CROSSING

        if loc == 'Indoor':
            edge_flags |= EDGE_FLAG_INDOOR
        if wp == 'Covered':
            edge_flags |= EDGE_FLAG_COVERED
        if wb == 'True':
            edge_flags |= EDGE_FLAG_BARRIER
        if gradient and gradient > 0.1:  # >10% grade
            edge_flags |= EDGE_FLAG_STEEP

        # Process each line, then each segment within it
        shape_len_2d = props.get('Shape_Length', 0) or 0

        # Direction: 0=Both Ways, 1=One Way (digitizing), 2=One Way (reverse)
        is_both = (direction == 0 or direction == 'Both Ways' or direction == '0')
        is_reverse_only = (direction == 2 or direction == 'One Way (Reverse of Digitizing Direction)' or direction == '2')

        for coords in lines:
            # 2D length allocation: if a single line, use Shape_Length directly
            seg_count = len(coords) - 1
            for j in range(seg_count):
                p1, p2 = coords[j], coords[j + 1]

                # Get z values (default to 0)
                z1 = p1[2] if len(p1) >= 3 else 0.0
                z2 = p2[2] if len(p2) >= 3 else 0.0

                n1 = get_or_create_node(p1[0], p1[1], z1)
                n2 = get_or_create_node(p2[0], p2[1], z2)

                # 3D distance
                d3d = distance_3d(p1, p2)

                # Ascent/descent
                dz = z2 - z1
                ascent = max(0.0, dz)
                descent = max(0.0, -dz)

                # 2D length (proportional allocation from Shape_Length)
                # Use the computed 3D length as fallback if Shape_Length is 0
                d2d = d3d if shape_len_2d <= 0 else (d3d / (d3d + 0.001)) * (shape_len_2d / seg_count)

                # Forward direction
                edges.append((n1, n2, d2d, d3d, ascent, descent, gradient, edge_flags))

                if is_both:
                    rev_ascent = descent
                    rev_descent = ascent
                    edges.append((n2, n1, d2d, d3d, rev_ascent, rev_descent, gradient, edge_flags))
                elif is_reverse_only:
                    edges.append((n2, n1, d2d, d3d, descent, ascent, gradient, edge_flags))

    print(f"  Nodes: {len(nodes):,}")
    print(f"  Edges: {len(edges):,}")
    print(f"  Skipped features: {skipped}")

    # ── Write binary ─────────────────────────────────────────────────────────
    print(f"Writing {output_path}...")
    with open(output_path, 'wb') as f:
        # Header
        f.write(MAGIC)
        f.write(struct.pack('<I', VERSION))
        f.write(struct.pack('<Q', len(nodes)))
        f.write(struct.pack('<Q', len(edges)))
        f.write(struct.pack('<I', 0))  # reserved flags

        # Nodes
        for x, y, z, flags in nodes:
            f.write(struct.pack('<dddI', x, y, z, flags))

        # Edges
        for from_n, to_n, d2d, d3d, asc, desc, grade, flags in edges:
            f.write(struct.pack('<QQdddddI', from_n, to_n, d2d, d3d, asc, desc, grade, flags))

    file_size = os.path.getsize(output_path)
    print(f"  Written: {file_size:,} bytes ({file_size/1024/1024:.1f} MB)")
    print("Done!")


def main():
    parser = argparse.ArgumentParser(description="Convert 3D Pedestrian Network GeoJSON to binary graph")
    parser.add_argument('input', help='Input GeoJSON file (pedestrian_route.json)')
    parser.add_argument('-o', '--output', default='graph.bin', help='Output binary file')
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Error: Input file not found: {args.input}")
        sys.exit(1)

    process_geojson(args.input, args.output)


if __name__ == '__main__':
    main()
