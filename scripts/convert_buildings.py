#!/usr/bin/env python3
"""
convert_buildings.py — Parse HK 3D-BIT00 FBX building files, normalize coordinates,
swap Y/Z axes for 3D engine (Y-up), and output a compact binary file.

Usage:
    python3 convert_buildings.py [--fbx-dir DIR] [-o OUTPUT.bin] [--max-files N]

Input:  FBX files from HK CSDI 3D-BIT00 dataset (HK80 grid, centimeters)
Output: Binary file with normalized, axis-swapped geometry (meters)

Binary format (little-endian):
  HEADER (32 bytes)
    magic[8]     = "GEOBLDG1"
    version[4]   = uint32 = 1
    center_x[8]  = float64 (HK80 easting of scene center, meters)
    center_y[8]  = float64 (HK80 northing of scene center, meters)
    num_buildings[4] = uint32
  BUILDING (variable per building)
    id_len[2]    = uint16
    id[id_len]   = UTF-8 bytes
    base_pd[4]   = float32 (ground elevation, meters)
    roof_pd[4]   = float32 (roof elevation, meters)
    num_verts[4] = uint32
    num_idx[4]   = uint32
    vertices[num_verts * 12]  = float32[3] each (X, Y, Z)  — Y=up, Z=into-screen
    indices[num_idx]          = uint32 each (triangle list)
"""

import struct
import sys
import os
import glob
import subprocess
import json
import argparse
import math
from concurrent.futures import ProcessPoolExecutor, as_completed

MAGIC = b"GEOBLDG1"
VERSION = 1


# ── Batch FBX parsing (subprocess for ufbx isolation) ──────────────────────

# ufbx segfaults when loading >1 file per process, so we use one subprocess per file.
# We write a helper script and invoke it per file for speed (avoids -c overhead).

_FBX_HELPER = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '_fbx_parse.py')

def _write_helper():
    """Write the single-file FBX parser helper script."""
    with open(_FBX_HELPER, 'w') as f:
        f.write('''\
import ufbx, json, sys, os

fbx_path = sys.argv[1]
fname = os.path.basename(fbx_path)

try:
    scene = ufbx.load_file(fbx_path)
    for node in scene.nodes:
        if node.mesh is None:
            continue
        mesh = node.mesh
        m = node.node_to_world

        verts = []
        for i in range(mesh.num_vertices):
            v = mesh.vertices[i]
            wx = m.c0.x*v.x + m.c1.x*v.y + m.c2.x*v.z + m.c3.x
            wy = m.c0.y*v.x + m.c1.y*v.y + m.c2.y*v.z + m.c3.y
            wz = m.c0.z*v.x + m.c1.z*v.y + m.c2.z*v.z + m.c3.z
            verts.extend([wx / 100.0, wy / 100.0, wz / 100.0])

        # Sanity check: HK80 coordinates for HK should be > 100000 meters
        if verts[0] < 50000 or verts[1] < 50000:
            print(json.dumps({"ok": False, "error": f"coords out of HK80 range: ({verts[0]:.0f},{verts[1]:.0f})"}))
            sys.exit(0)

        # Skip degenerate geometry (buildings with near-zero footprint)
        xmin = float('inf'); xmax = float('-inf')
        ymin = float('inf'); ymax = float('-inf')
        for i in range(0, len(verts), 3):
            if verts[i] < xmin: xmin = verts[i]
            if verts[i] > xmax: xmax = verts[i]
            if verts[i+1] < ymin: ymin = verts[i+1]
            if verts[i+1] > ymax: ymax = verts[i+1]
        if (xmax - xmin) < 0.5 and (ymax - ymin) < 0.5:
            print(json.dumps({"ok": False, "error": f"degenerate footprint: {xmax-xmin:.2f}x{ymax-ymin:.2f}m"}))
            sys.exit(0)

        idx = [int(mesh.vertex_indices[i]) for i in range(mesh.num_indices)]

        zmin = float('inf'); zmax = float('-inf')
        for i in range(0, len(verts), 3):
            z = verts[i+2]
            if z < zmin: zmin = z
            if z > zmax: zmax = z

        print(json.dumps({
            "ok": True, "v": verts, "i": idx,
            "base_pd": zmin, "roof_pd": zmax
        }))
        sys.exit(0)

    print(json.dumps({"ok": False, "error": "no mesh node"}))
except Exception as e:
    print(json.dumps({"ok": False, "error": str(e)}))
''')

_write_helper()


def parse_fbx_file(fbx_path):
    """
    Parse a single FBX file via subprocess (ufbx isolation).
    Returns dict with world-space geometry in meters, or None on failure.
    """
    try:
        r = subprocess.run(
            ['python3', _FBX_HELPER, fbx_path],
            capture_output=True, text=True, timeout=30
        )
        if r.stdout:
            data = json.loads(r.stdout)
            if data.get('ok'):
                return data
            # else: error
    except subprocess.TimeoutExpired:
        pass
    except Exception:
        pass
    return None


def process_all_fbx(fbx_dir, max_files=None, include_terrain=False):
    """
    Process all FBX files in fbx_dir, return:
      buildings: list of dicts with id, base_pd, roof_pd, vertices, indices
      global_bounds: (xmin, xmax, ymin, ymax, zmin, zmax) in meters
    """
    files = sorted(glob.glob(os.path.join(fbx_dir, "*.fbx")))

    # Skip terrain tiles (T_*) — they use a different coordinate scale (tile-local)
    if not include_terrain:
        files = [f for f in files
                 if not os.path.basename(f).split('_')[1].startswith('T')]

    if max_files:
        files = files[:max_files]

    print(f"Processing {len(files)} FBX files...", file=sys.stderr)

    buildings = []
    xmin, xmax = float('inf'), float('-inf')
    ymin, ymax = float('inf'), float('-inf')
    zmin, zmax = float('inf'), float('-inf')
    errors = 0

    # Process files in parallel subprocesses (each subprocess loads 1 file to avoid ufbx segfault)
    MAX_WORKERS = min(8, len(files))
    print(f"  Using {MAX_WORKERS} parallel workers", file=sys.stderr)

    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
        future_to_path = {executor.submit(parse_fbx_file, p): p for p in files}

        for i, future in enumerate(as_completed(future_to_path)):
            fbx_path = future_to_path[future]
            fname = os.path.basename(fbx_path)
            parts = fname.replace('.fbx', '').split('_', 1)
            building_id = parts[1] if len(parts) > 1 else fname

            try:
                data = future.result(timeout=30)
            except Exception:
                data = None

            if data is None:
                errors += 1
                if (i + 1) % 100 == 0 or i < 5:
                    print(f"  [{i+1}/{len(files)}] SKIP {fname}", file=sys.stderr)
                continue

            verts = data['v']
            indices = data['i']

            if len(verts) < 3 or len(indices) < 3:
                errors += 1
                continue

            # Update global bounds
            for j in range(0, len(verts), 3):
                x, y, z = verts[j], verts[j+1], verts[j+2]
                if x < xmin: xmin = x
                if x > xmax: xmax = x
                if y < ymin: ymin = y
                if y > ymax: ymax = y
                if z < zmin: zmin = z
                if z > zmax: zmax = z

            buildings.append({
                'id': building_id,
                'base_pd': data['base_pd'],
                'roof_pd': data['roof_pd'],
                'vertices': verts,
                'indices': indices,
            })

            if (i + 1) % 100 == 0:
                print(f"  [{i+1}/{len(files)}] {len(buildings)} buildings, "
                      f"bounds X:[{xmin:.0f},{xmax:.0f}] Y:[{ymin:.0f},{ymax:.0f}]",
                      file=sys.stderr)

    print(f"\nDone: {len(buildings)} buildings, {errors} errors/skipped", file=sys.stderr)
    print(f"World bounds (HK80 meters):", file=sys.stderr)
    print(f"  X: [{xmin:.1f}, {xmax:.1f}]  span={xmax-xmin:.1f}", file=sys.stderr)
    print(f"  Y: [{ymin:.1f}, {ymax:.1f}]  span={ymax-ymin:.1f}", file=sys.stderr)
    print(f"  Z: [{zmin:.2f}, {zmax:.2f}]  span={zmax-zmin:.2f}", file=sys.stderr)

    return buildings, (xmin, xmax, ymin, ymax, zmin, zmax)


def normalize_and_swap(buildings, bounds):
    """
    Apply coordinate normalization and Y/Z axis swap.

    GIS convention:  X=easting, Y=northing, Z=elevation
    3D engine:       X=right,   Y=up,        Z=into-screen

    Transform:
      out_X =  gis_X - center_X
      out_Y =  gis_Z                   (elevation → up)
      out_Z = -(gis_Y - center_Y)      (northing → into-screen, negated)
    """
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    center_x = (xmin + xmax) / 2.0
    center_y = (ymin + ymax) / 2.0

    print(f"\nNormalizing: center=({center_x:.1f}, {center_y:.1f})", file=sys.stderr)

    for b in buildings:
        v = b['vertices']
        new_v = []
        for i in range(0, len(v), 3):
            gx, gy, gz = v[i], v[i+1], v[i+2]
            ox = gx - center_x
            oy = gz                     # elevation → up
            oz = -(gy - center_y)       # northing → -Z
            new_v.extend([ox, oy, oz])
        b['vertices'] = new_v

    return center_x, center_y


def write_binary(buildings, center_x, center_y, output_path):
    """Write the normalized binary file."""
    with open(output_path, 'wb') as f:
        # Header
        f.write(MAGIC)
        f.write(struct.pack('<I', VERSION))
        f.write(struct.pack('<d', center_x))
        f.write(struct.pack('<d', center_y))
        f.write(struct.pack('<I', len(buildings)))

        for b in buildings:
            bid = b['id'].encode('utf-8')
            # Building record
            f.write(struct.pack('<H', len(bid)))
            f.write(bid)
            f.write(struct.pack('<f', float(b['base_pd'])))
            f.write(struct.pack('<f', float(b['roof_pd'])))
            f.write(struct.pack('<I', len(b['vertices']) // 3))
            f.write(struct.pack('<I', len(b['indices'])))

            # Vertices as float32 triplets
            for v in b['vertices']:
                f.write(struct.pack('<f', float(v)))

            # Indices as uint32
            for idx in b['indices']:
                f.write(struct.pack('<I', int(idx)))

    size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"\nWrote: {output_path} ({size_mb:.1f} MB)", file=sys.stderr)
    print(f"  {len(buildings)} buildings", file=sys.stderr)
    print(f"  Center: ({center_x:.1f}, {center_y:.1f}) HK80", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description="Convert HK 3D-BIT00 FBX buildings to normalized binary")
    parser.add_argument('--fbx-dir',
                        default='scripts/buildings/fbx',
                        help='Directory containing FBX files')
    parser.add_argument('-o', '--output',
                        default='scripts/buildings.bin',
                        help='Output binary file')
    parser.add_argument('--max-files', type=int, default=None,
                        help='Limit number of FBX files to process')
    parser.add_argument('--include-terrain', action='store_true',
                        help='Also process terrain tiles (T_*.fbx)')
    args = parser.parse_args()

    # Resolve paths relative to project root
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    fbx_dir = os.path.join(project_root, args.fbx_dir) if not os.path.isabs(args.fbx_dir) else args.fbx_dir
    output_path = os.path.join(project_root, args.output) if not os.path.isabs(args.output) else args.output

    if not os.path.isdir(fbx_dir):
        print(f"Error: FBX directory not found: {fbx_dir}", file=sys.stderr)
        sys.exit(1)

    # Phase 1: Parse all FBX files, collect world-space geometry
    buildings, bounds = process_all_fbx(fbx_dir, args.max_files, args.include_terrain)

    if not buildings:
        print("Error: no buildings parsed", file=sys.stderr)
        sys.exit(1)

    # Phase 2: Normalize coordinates and swap Y/Z axes
    center_x, center_y = normalize_and_swap(buildings, bounds)

    # Phase 3: Write binary output
    write_binary(buildings, center_x, center_y, output_path)

    # Quick stats
    total_v = sum(len(b['vertices']) // 3 for b in buildings)
    total_t = sum(len(b['indices']) // 3 for b in buildings)
    print(f"  {total_v:,} total vertices, {total_t:,} total triangles", file=sys.stderr)
    print("Done!", file=sys.stderr)


if __name__ == '__main__':
    main()
