#!/usr/bin/env python3
"""
download_checkpoint.py — Download 3D Pedestrian Network with checkpoint/resume.

Saves partial progress every 20 chunks so a failed download can be resumed.
The final binary graph is assembled from checkpoint files.

Usage:
    python download_checkpoint.py -o graph.bin           # fresh download
    python download_checkpoint.py -o graph.bin --resume  # resume from checkpoint
"""

import json
import struct
import sys
import os
import time
import argparse
import math
import glob
import pickle

try:
    import requests
except ImportError:
    print("pip install requests")
    sys.exit(1)

ARCGIS_URL = (
    "https://portal.csdi.gov.hk/server/rest/services/common/"
    "landsd_rcd_1637222018065_52265/MapServer/0/query"
)
MAGIC = b"GEODISG\x02"
VERSION = 2
CHUNK_SIZE = 2000
CHECKPOINT_EVERY = 20  # save every 20 chunks (40K features)

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


def distance_3d_m(p1, p2):
    lat_mid = math.radians((p1[1] + p2[1]) / 2.0)
    dx = (p2[0] - p1[0]) * 111320.0 * math.cos(lat_mid)
    dy = (p2[1] - p1[1]) * 110540.0
    z1 = p1[2] if len(p1) > 2 else 0.0
    z2 = p2[2] if len(p2) > 2 else 0.0
    dz = z2 - z1
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def feature_flags(ft, loc, wp, wb, gradient):
    """Determine edge flags from feature properties.
    
    Handles both integer codes (ArcGIS REST API) and string values (FILE_API).
    """
    flags = 0
    
    # FeatureType: 1=Footway, 2=Footbridge, 3=Subway, 4=Stairs, 5=Lift, 
    #              6=Escalator, 7=Crossing, 8+=Generalized Walkway
    ft_int = None
    if isinstance(ft, (int, float)):
        ft_int = int(ft)
    elif isinstance(ft, str):
        ft_s = ft.lower()
        if "stairs" in ft_s:      ft_int = 4
        elif "lift" in ft_s:      ft_int = 5
        elif "escalator" in ft_s: ft_int = 6
        elif "footbridge" in ft_s: ft_int = 2
        elif "subway" in ft_s:    ft_int = 3
        elif "crossing" in ft_s:  ft_int = 7
        else:                     ft_int = 1  # default footway
    
    if ft_int == 4:      flags |= EDGE_FLAG_STAIRS
    elif ft_int == 5:    flags |= EDGE_FLAG_LIFT
    elif ft_int == 6:    flags |= EDGE_FLAG_ESCALATOR
    elif ft_int == 2:    flags |= EDGE_FLAG_FOOTBRIDGE
    elif ft_int == 3:    flags |= EDGE_FLAG_SUBWAY
    elif ft_int == 7:    flags |= EDGE_FLAG_CROSSING
    
    # Location: 1=Outdoor, 2=Indoor
    loc_int = loc
    if isinstance(loc, str):
        loc_int = 2 if loc.lower() == "indoor" else 1
    if loc_int == 2:
        flags |= EDGE_FLAG_INDOOR
    
    # WeatherProof: 1=Covered, 2=NonCovered
    wp_int = wp
    if isinstance(wp, str):
        wp_int = 1 if wp.lower() == "covered" else 2
    if wp_int == 1:
        flags |= EDGE_FLAG_COVERED
    
    # WheelchairBarrier: 1=True, 2=False (reversed: barrier flag set when True)
    wb_int = wb
    if isinstance(wb, str):
        wb_int = 1 if wb.lower() == "true" else 2
    if wb_int == 1:
        flags |= EDGE_FLAG_BARRIER
    
    # Gradient: steep if > 0.1 (10% grade)
    try:
        g = float(gradient) if gradient is not None else 0.0
        if g > 0.1:
            flags |= EDGE_FLAG_STEEP
    except (ValueError, TypeError):
        pass
    
    return flags


def save_checkpoint(checkpoint_dir, chunk_idx, nodes, edges, coord_to_node, total_features):
    """Save partial graph to checkpoint files."""
    os.makedirs(checkpoint_dir, exist_ok=True)
    base = os.path.join(checkpoint_dir, f"chunk_{chunk_idx:04d}")

    # Save nodes (appended format)
    with open(base + "_nodes.bin", 'wb') as f:
        for lon, lat, z, flags in nodes:
            f.write(struct.pack('<dddI', lon, lat, z, flags))

    # Save edges
    with open(base + "_edges.bin", 'wb') as f:
        for from_n, to_n, d2d, d3d, asc, desc, grade, flags in edges:
            f.write(struct.pack('<QQdddddI', from_n, to_n, d2d, d3d, asc, desc, grade, flags))

    # Save coord_to_node mapping
    with open(base + "_map.pkl", 'wb') as f:
        pickle.dump(coord_to_node, f)

    # Save metadata
    with open(os.path.join(checkpoint_dir, "meta.json"), 'w') as f:
        json.dump({
            "chunk_idx": chunk_idx,
            "total_features": total_features,
            "total_nodes": sum(1 for _ in glob.iglob(os.path.join(checkpoint_dir, "*_nodes.bin"))),
            "total_edges": sum(1 for _ in glob.iglob(os.path.join(checkpoint_dir, "*_edges.bin"))),
        }, f)

    print(f"  [checkpoint saved: chunk {chunk_idx}, {total_features:,} features]")


def load_checkpoint(checkpoint_dir):
    """Load the latest checkpoint state."""
    meta_path = os.path.join(checkpoint_dir, "meta.json")
    if not os.path.exists(meta_path):
        return None, [], [], {}, 0

    with open(meta_path) as f:
        meta = json.load(f)

    # Find latest map file
    map_files = sorted(glob.glob(os.path.join(checkpoint_dir, "chunk_*_map.pkl")))
    if not map_files:
        return None, [], [], {}, 0

    latest_map = map_files[-1]
    with open(latest_map, 'rb') as f:
        coord_to_node = pickle.load(f)

    # Load all nodes
    nodes = []
    for nf in sorted(glob.glob(os.path.join(checkpoint_dir, "chunk_*_nodes.bin"))):
        with open(nf, 'rb') as f:
            while True:
                data = f.read(28)  # 3 doubles + 1 uint32
                if len(data) < 28:
                    break
                lon, lat, z, flags = struct.unpack('<dddI', data)
                nodes.append((lon, lat, z, flags))

    # Load all edges
    edges = []
    for ef in sorted(glob.glob(os.path.join(checkpoint_dir, "chunk_*_edges.bin"))):
        with open(ef, 'rb') as f:
            while True:
                data = f.read(60)  # 2 uint64 + 6 doubles + 1 uint32
                if len(data) < 60:
                    break
                from_n, to_n, d2d, d3d, asc, desc, grade, flags = struct.unpack('<QQdddddI', data)
                edges.append((from_n, to_n, d2d, d3d, asc, desc, grade, flags))

    return coord_to_node, nodes, edges, meta["total_features"]


def download(output_path, checkpoint_dir=".checkpoints", resume=False):
    os.makedirs(checkpoint_dir, exist_ok=True)

    if resume:
        coord_to_node, nodes, edges, total_features = load_checkpoint(checkpoint_dir)
        if coord_to_node is None:
            print("No checkpoint found, starting fresh.")
            coord_to_node = {}
            nodes = []
            edges = []
            total_features = 0
        else:
            print(f"Resumed from checkpoint: {total_features:,} features, "
                  f"{len(nodes):,} nodes, {len(edges):,} edges")
    else:
        coord_to_node = {}
        nodes = []
        edges = []
        total_features = 0

    offset = total_features
    chunk_num = total_features // CHUNK_SIZE
    checkpoint_idx = chunk_num
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

        # Retry with backoff
        max_retries = 6
        for attempt in range(max_retries):
            try:
                resp = requests.get(ARCGIS_URL, params=params, timeout=120)
                if resp.status_code == 403 and attempt < max_retries - 1:
                    wait = 5.0 * (2 ** attempt)
                    print(f"  Rate limited (403), waiting {wait:.0f}s...")
                    time.sleep(wait)
                    continue
                if resp.status_code == 200:
                    break
                resp.raise_for_status()
            except requests.exceptions.RequestException as e:
                if attempt < max_retries - 1:
                    wait = 5.0 * (2 ** attempt)
                    print(f"  {e}, retrying in {wait:.0f}s...")
                    time.sleep(wait)
                else:
                    raise

        data = resp.json()
        features = data.get("features", [])
        if not features:
            break

        chunk_num += 1
        total_features += len(features)

        # Process
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
            edge_flags = feature_flags(ft, loc, wp, wb, gradient)

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
                d2d = d3d
                if shape_len > 0 and n_segments > 0:
                    d2d = shape_len / n_segments

                dz = z2 - z1
                ascent = max(0.0, dz)
                descent = max(0.0, -dz)
                try:
                    grade = float(gradient) if gradient else 0.0
                except (ValueError, TypeError):
                    grade = 0.0

                edges.append((n1, n2, d2d, d3d, ascent, descent, grade, edge_flags))
                # Direction: 0=Both Ways, 1=One Way (digitizing), 2=One Way (reverse)
                is_both = (direction == 0 or direction == "Both Ways" or direction == "0")
                is_reverse = (direction == 2 or direction == "One Way (Reverse of Digitizing Direction)" or direction == "2")
                if is_both:
                    edges.append((n2, n1, d2d, d3d, descent, ascent, grade, edge_flags))
                elif is_reverse:
                    edges.append((n2, n1, d2d, d3d, ascent, descent, grade, edge_flags))

        elapsed = time.time() - start_time
        rate = total_features / elapsed if elapsed > 0 else 0
        print(f"  Chunk {chunk_num}: {total_features:,} features, "
              f"{len(nodes):,} nodes, {len(edges):,} edges "
              f"({rate:.0f} f/s)")

        # Checkpoint
        if chunk_num % CHECKPOINT_EVERY == 0:
            save_checkpoint(checkpoint_dir, chunk_num, nodes, edges, coord_to_node, total_features)
            # Clear old checkpoints to save space (keep only this one's data)
            for old in glob.glob(os.path.join(checkpoint_dir, f"chunk_*")):
                cidx = int(os.path.basename(old).split('_')[1])
                if cidx != chunk_num:
                    for ext in ['_nodes.bin', '_edges.bin', '_map.pkl']:
                        fpath = old.replace(old.split('_')[0] + '_' + old.split('_')[1], f"chunk_{cidx:04d}")
                        # Actually just keep the latest map
                        pass

        offset += len(features)
        if len(features) < CHUNK_SIZE:
            break

        time.sleep(0.5)

    elapsed = time.time() - start_time
    print(f"\nDownload complete: {total_features:,} features in {elapsed:.1f}s")
    print(f"  Nodes: {len(nodes):,}")
    print(f"  Edges: {len(edges):,}")

    # ── Write final binary ──────────────────────────────────────────────────
    print(f"Writing {output_path}...")
    with open(output_path, 'wb') as f:
        f.write(MAGIC)
        f.write(struct.pack('<I', VERSION))
        f.write(struct.pack('<Q', len(nodes)))
        f.write(struct.pack('<Q', len(edges)))
        f.write(struct.pack('<I', 0))

        for lon, lat, z, flags in nodes:
            f.write(struct.pack('<dddI', lon, lat, z, flags))
        for from_n, to_n, d2d, d3d, asc, desc, grade, flags in edges:
            f.write(struct.pack('<QQdddddI', from_n, to_n, d2d, d3d, asc, desc, grade, flags))

    file_size = os.path.getsize(output_path)
    print(f"  Written: {file_size:,} bytes ({file_size/1024/1024:.1f} MB)")
    print("Done!")


def main():
    parser = argparse.ArgumentParser(description="Download HK 3D Pedestrian Network with checkpoint support")
    parser.add_argument('-o', '--output', default='graph.bin', help='Output binary file')
    parser.add_argument('--resume', action='store_true', help='Resume from checkpoint')
    parser.add_argument('--checkpoint-dir', default='.checkpoints', help='Checkpoint directory')
    args = parser.parse_args()

    download(args.output, args.checkpoint_dir, args.resume)


if __name__ == '__main__':
    main()
