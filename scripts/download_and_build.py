#!/usr/bin/env python3
"""
download_and_build.py — Download HK 3D Pedestrian Network from ArcGIS REST API
and build the binary graph file for geodis C++ application.

Downloads in chunks of 2000 features (total ~465K features = ~233 requests).
Each request is about 1-2 MB. Expect the full download to take a few minutes.

Usage:
    python download_and_build.py -o graph.bin

Requirements:
    pip install requests
"""

import json
import struct
import sys
import os
import time
import argparse
import math

try:
    import requests
except ImportError:
    print("Error: requests library required. Install with: pip install requests")
    sys.exit(1)

# ─── Constants ────────────────────────────────────────────────────────────────
ARCGIS_URL = (
    "https://portal.csdi.gov.hk/server/rest/services/common/"
    "landsd_rcd_1637222018065_52265/MapServer/0/query"
)

MAGIC = b"GEODISG\x02"
VERSION = 2
CHUNK_SIZE = 2000  # features per request

# ─── Feature type flags ──────────────────────────────────────────────────────
EDGE_FLAG_STAIRS     = 1 << 0
EDGE_FLAG_LIFT       = 1 << 1
EDGE_FLAG_ESCALATOR  = 1 << 2
EDGE_FLAG_FOOTBRIDGE = 1 << 3
EDGE_FLAG_SUBWAY     = 1 << 4
EDGE_FLAG_CROSSING   = 1 << 5
EDGE_FLAG_INDOOR     = 1 << 6
EDGE_FLAG_COVERED    = 1 << 7
EDGE_FLAG_STEEP      = 1 << 8
EDGE_FLAG_BARRIER    = 1 << 9

FTYPE_MAP = {
    "Footway": 0,
    "Footbridge": 1,
    "Subway": 2,
    "Stairs": 3,
    "Lift": 4,
    "Escalator": 5,
    "Crossing": 6,
}

DIRECTION = {
    "Both Ways": 0,
    "One Way (Digitizing Direction)": 1,
    "One Way (Reverse of Digitizing Direction)": 2,
}


def distance_3d_m(p1, p2):
    """Approximate 3D distance in meters between two WGS84 points."""
    lat_mid = math.radians((p1[1] + p2[1]) / 2.0)
    dx = (p2[0] - p1[0]) * 111320.0 * math.cos(lat_mid)
    dy = (p2[1] - p1[1]) * 110540.0
    dz = (p2[2] if len(p2) > 2 else 0) - (p1[2] if len(p1) > 2 else 0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def feature_type_to_flags(ft, location, weatherproof, wheelchair_barrier, gradient):
    """Convert feature type to edge flags. Handles both integer codes and strings."""
    flags = 0

    # FeatureType codes: 1=Footway, 2=Footbridge, 3=Subway, 4=Stairs, 5=Lift,
    #                    6=Escalator, 7=Crossing
    ft_int = None
    if isinstance(ft, (int, float)):
        ft_int = int(ft)
    elif isinstance(ft, str):
        s = ft.lower()
        if "stairs" in s:      ft_int = 4
        elif "lift" in s:      ft_int = 5
        elif "escalator" in s: ft_int = 6
        elif "footbridge" in s: ft_int = 2
        elif "subway" in s:    ft_int = 3
        elif "crossing" in s:  ft_int = 7
        else:                  ft_int = 1

    if ft_int == 4:      flags |= EDGE_FLAG_STAIRS
    elif ft_int == 5:    flags |= EDGE_FLAG_LIFT
    elif ft_int == 6:    flags |= EDGE_FLAG_ESCALATOR
    elif ft_int == 2:    flags |= EDGE_FLAG_FOOTBRIDGE
    elif ft_int == 3:    flags |= EDGE_FLAG_SUBWAY
    elif ft_int == 7:    flags |= EDGE_FLAG_CROSSING

    # Location: 1=Outdoor, 2=Indoor
    loc_int = location
    if isinstance(location, str):
        loc_int = 2 if location.lower() == "indoor" else 1
    if loc_int == 2:
        flags |= EDGE_FLAG_INDOOR

    # WeatherProof: 1=Covered, 2=NonCovered
    wp_int = weatherproof
    if isinstance(weatherproof, str):
        wp_int = 1 if weatherproof.lower() == "covered" else 2
    if wp_int == 1:
        flags |= EDGE_FLAG_COVERED

    # WheelchairBarrier: 1=True, 2=False
    wb_int = wheelchair_barrier
    if isinstance(wheelchair_barrier, str):
        wb_int = 1 if wheelchair_barrier.lower() == "true" else 2
    if wb_int == 1:
        flags |= EDGE_FLAG_BARRIER

    try:
        g = float(gradient) if gradient is not None else 0.0
        if g > 0.1:
            flags |= EDGE_FLAG_STEEP
    except (ValueError, TypeError):
        pass

    return flags


def build_graph(output_path, resume_from=0):
    """Download the full dataset and build the binary graph."""
    print(f"Downloading 3D Pedestrian Network from ArcGIS REST API...")
    print(f"  Chunk size: {CHUNK_SIZE} features")
    if resume_from > 0:
        print(f"  Resuming from offset {resume_from}")

    coord_to_node = {}
    nodes = []  # list of (lon, lat, z, flags)
    edges = []  # (from_node, to_node, len_2d, len_3d, ascent, descent, grade, flags)

    offset = resume_from
    total_features = resume_from
    chunk_num = resume_from // CHUNK_SIZE
    start_time = time.time()

    while True:
        params = {
            "where": "1=1",
            "outFields": "*",
            "returnGeometry": "true",
            "geometryType": "esriGeometryPolyline",
            "returnZ": "true",
            "f": "geojson",
            "resultRecordCount": CHUNK_SIZE,
            "resultOffset": offset,
        }

        max_retries = 5
        retry_delay = 2.0
        for attempt in range(max_retries):
            try:
                resp = requests.get(ARCGIS_URL, params=params, timeout=120)

                if resp.status_code == 403:
                    if attempt < max_retries - 1:
                        wait = retry_delay * (2 ** attempt)
                        print(f"  Rate limited (403), retrying in {wait:.0f}s...")
                        time.sleep(wait)
                        continue
                    else:
                        resp.raise_for_status()

                resp.raise_for_status()
                break
            except requests.exceptions.RequestException as e:
                if attempt < max_retries - 1:
                    wait = retry_delay * (2 ** attempt)
                    print(f"  Error: {e}, retrying in {wait:.0f}s...")
                    time.sleep(wait)
                else:
                    raise

        data = resp.json()
        features = data.get("features", [])

        if not features:
            break

        chunk_num += 1
        total_features += len(features)

        # Process features
        for feat in features:
            props = feat.get("properties", {})
            coords = feat.get("geometry", {}).get("coordinates", [])

            if len(coords) < 2:
                continue

            ft = props.get("FeatureType", "Footway")
            loc = props.get("Location", "Outdoor")
            wp = props.get("WeatherProof", "NonCovered")
            wb = props.get("WheelchairBarrier", "False")
            direction = props.get("Direction", "Both Ways")
            gradient = props.get("Gradient", 0)
            shape_len = props.get("Shape_Length", 0) or 0

            edge_flags = feature_type_to_flags(ft, loc, wp, wb, gradient)

            # For each consecutive pair of coordinates, create nodes and edges
            n_segments = len(coords) - 1

            for j in range(n_segments):
                p1, p2 = coords[j], coords[j + 1]
                z1 = p1[2] if len(p1) >= 3 else 0.0
                z2 = p2[2] if len(p2) >= 3 else 0.0

                key1 = (round(p1[0], 10), round(p1[1], 10), round(z1, 4))
                key2 = (round(p2[0], 10), round(p2[1], 10), round(z2, 4))

                if key1 not in coord_to_node:
                    idx = len(nodes)
                    coord_to_node[key1] = idx
                    nodes.append((p1[0], p1[1], z1, 0))
                if key2 not in coord_to_node:
                    idx = len(nodes)
                    coord_to_node[key2] = idx
                    nodes.append((p2[0], p2[1], z2, 0))

                n1 = coord_to_node[key1]
                n2 = coord_to_node[key2]

                d3d = distance_3d_m(p1, p2)
                d2d = d3d  # 2D fallback

                # Use Shape_Length proportionally if available
                if shape_len > 0 and n_segments > 0:
                    d2d = shape_len / n_segments

                dz = z2 - z1
                ascent = max(0.0, dz)
                descent = max(0.0, -dz)

                try:
                    grade = float(gradient) if gradient else 0.0
                except (ValueError, TypeError):
                    grade = 0.0

                # Forward
                edges.append((n1, n2, d2d, d3d, ascent, descent, grade, edge_flags))

                # Direction: 0=Both Ways, 1=One Way (digitizing), 2=One Way (reverse)
                is_both = (direction == 0 or direction == "Both Ways" or direction == "0")
                is_reverse = (direction == 2 or direction == "One Way (Reverse of Digitizing Direction)" or direction == "2")
                if is_both or is_reverse:
                    if is_both:
                        rev_ascent = descent
                        rev_descent = ascent
                    else:
                        rev_ascent = ascent
                        rev_descent = descent
                    edges.append((n2, n1, d2d, d3d, rev_ascent, rev_descent, grade, edge_flags))

        elapsed = time.time() - start_time
        rate = total_features / elapsed if elapsed > 0 else 0
        print(f"  Chunk {chunk_num}: {total_features:,} features, "
              f"{len(nodes):,} nodes, {len(edges):,} edges "
              f"({rate:.0f} feat/s)")

        offset += len(features)

        # If fewer than requested, we're done
        if len(features) < CHUNK_SIZE:
            break

        # Rate limiting - slow down to avoid 403s
        time.sleep(0.5)

    elapsed = time.time() - start_time
    print(f"\nDownload complete: {total_features:,} features in {elapsed:.1f}s")
    print(f"  Nodes: {len(nodes):,}")
    print(f"  Edges: {len(edges):,}")

    # ── Write binary ─────────────────────────────────────────────────────────
    print(f"\nWriting {output_path}...")
    with open(output_path, 'wb') as f:
        # Header
        f.write(MAGIC)
        f.write(struct.pack('<I', VERSION))
        f.write(struct.pack('<Q', len(nodes)))
        f.write(struct.pack('<Q', len(edges)))
        f.write(struct.pack('<I', 0))  # reserved

        # Nodes: lon[8] lat[8] z[8] flags[4]
        for lon, lat, z, flags in nodes:
            f.write(struct.pack('<dddI', lon, lat, z, flags))

        # Edges: from[8] to[8] len2d[8] len3d[8] ascent[8] descent[8] grade[8] flags[4]
        for from_n, to_n, d2d, d3d, asc, desc, grade, flags in edges:
            f.write(struct.pack('<QQdddddI', from_n, to_n, d2d, d3d, asc, desc, grade, flags))

    file_size = os.path.getsize(output_path)
    print(f"  Written: {file_size:,} bytes ({file_size/1024/1024:.1f} MB)")
    print("Done!")


def main():
    parser = argparse.ArgumentParser(
        description="Download HK 3D Pedestrian Network and build binary graph")
    parser.add_argument('-o', '--output', default='graph.bin',
                        help='Output binary graph file')
    args = parser.parse_args()

    build_graph(args.output)


if __name__ == '__main__':
    main()
