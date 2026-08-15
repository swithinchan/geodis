#!/usr/bin/env python3
"""
generate_network_data.py — Convert the full HK 3D Pedestrian Network GeoJSON
(Lands Department) into a compact binary file + street-name label points.

Outputs (in scripts/ by default):
  1. network.bin           — line geometry, feature-type code, street-name index
  2. street_labels.geojson — one point per unique street name

network.bin layout (little-endian):
  HEADER (16 bytes)
    magic[8]       = "GEONET01"
    version[4]     = uint32 = 1
    num_features[4]= uint32
    num_streets[2] = uint16
    reserved[2]    = uint16
  STREET TABLE (variable)
    for each street:
      name_len[2]  = uint16 (UTF-8 byte length)
      name[...]    = UTF-8 bytes
  FEATURES
    for each feature:
      ft[1]        = uint8  (feature-type code)
      street_idx[2]= uint16 (0xFFFF = none)
      num_points[2]= uint16
      for each point:
        lon[4] = float32
        lat[4] = float32
        z[4]   = float32
"""

import json
import sys
import os
import struct
import argparse

MAGIC = b"GEONET01"
VERSION = 1

# FeatureType → compact numeric code (for data-driven styling in the viewer)
FTYPE_CODE = {
    "Footway": 0,
    "Footpath": 1,
    "Generalized Walkway inside Park": 2,
    "Staircase": 3,
    "Stairlift": 3,
    "Lift": 4,
    "Escalator": 5,
    "Travelator": 5,
    "Footbridge": 6,
    "Subway": 7,
    "Crossing - Signalized": 8,
    "Crossing - Cautionary": 8,
    "Crossing - Zebra": 8,
    "Crossing - Others": 8,
    "Ramp": 9,
    "RunIn": 9,
    "Service Lane": 9,
    "Traffic Island": 9,
    "Village": 9,
    "Track": 9,
    "Other": 9,
}

def iter_line_strings(geom):
    """Yield (list_of_[lon,lat,z]) for each line in a LineString/MultiLineString."""
    gt = geom.get('type')
    coords = geom.get('coordinates', [])
    if gt == 'MultiLineString':
        for seg in coords:
            yield seg
    else:
        yield coords

def iter_points(coords):
    for c in coords:
        if isinstance(c, (list, tuple)) and len(c) >= 2:
            yield c

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', nargs='?', default='data/pedestrian_route.json')
    parser.add_argument('-o', '--outdir', default='scripts')
    args = parser.parse_args()

    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    input_path = os.path.join(project_root, args.input) if not os.path.isabs(args.input) else args.input
    outdir = os.path.join(project_root, args.outdir) if not os.path.isabs(args.outdir) else args.outdir

    if not os.path.exists(input_path):
        print(f"Error: {input_path} not found", file=sys.stderr)
        sys.exit(1)

    print(f"Reading {input_path}...", file=sys.stderr)
    with open(input_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    features = data.get('features', [])
    print(f"  {len(features):,} features", file=sys.stderr)

    # ── Build street table and binary feature buffers ───────────────────────
    street_to_idx = {}
    street_names = []  # list of UTF-8 bytes
    street_points = {}  # idx -> [sum_lon, sum_lat, count, tc]

    feat_buffers = []
    num_features = 0

    for feat in features:
        props = feat.get('properties', {})
        geom = feat.get('geometry', {})

        ft = props.get('FeatureType') or 'Footway'
        code = FTYPE_CODE.get(ft, 9)

        en = props.get('StreetNameEN')
        tc = props.get('StreetNameTC')

        street_idx = 0xFFFF
        sp = None
        if en:
            if en not in street_to_idx:
                street_to_idx[en] = len(street_names)
                street_names.append(en.encode('utf-8'))
                street_points[street_to_idx[en]] = [0.0, 0.0, 0, tc]
            street_idx = street_to_idx[en]
            sp = street_points[street_idx]

        for line in iter_line_strings(geom):
            pts = []
            for c in iter_points(line):
                lon = c[0]
                lat = c[1]
                z = c[2] if len(c) >= 3 else 0.0
                pts.append((lon, lat, z))
                if sp is not None:
                    sp[0] += lon
                    sp[1] += lat
                    sp[2] += 1
            if len(pts) < 2:
                continue

            buf = bytearray()
            buf += struct.pack('<B', code)
            buf += struct.pack('<H', street_idx)
            buf += struct.pack('<H', len(pts))
            for lon, lat, z in pts:
                buf += struct.pack('<fff', lon, lat, z)
            feat_buffers.append(bytes(buf))
            num_features += 1

    print(f"  {num_features:,} line features", file=sys.stderr)
    print(f"  {len(street_names):,} unique street names", file=sys.stderr)

    # ── Write binary network ────────────────────────────────────────────────
    network_path = os.path.join(outdir, 'network.bin')
    print(f"Writing {network_path}...", file=sys.stderr)
    with open(network_path, 'wb') as f:
        f.write(MAGIC)
        f.write(struct.pack('<I', VERSION))
        f.write(struct.pack('<I', num_features))
        f.write(struct.pack('<H', len(street_names)))
        f.write(struct.pack('<H', 0))  # reserved

        for name_bytes in street_names:
            f.write(struct.pack('<H', len(name_bytes)))
            f.write(name_bytes)

        for buf in feat_buffers:
            f.write(buf)

    size_mb = os.path.getsize(network_path) / (1024 * 1024)
    print(f"  {size_mb:.1f} MB", file=sys.stderr)

    # ── Write street labels (GeoJSON) ───────────────────────────────────────
    label_features = []
    for idx, name_bytes in enumerate(street_names):
        sum_lon, sum_lat, count, tc = street_points[idx]
        if count == 0:
            continue
        center = [round(sum_lon / count, 6), round(sum_lat / count, 6)]
        lprops = {'name': name_bytes.decode('utf-8')}
        if tc:
            lprops['name_tc'] = tc
        label_features.append({
            'type': 'Feature',
            'geometry': {'type': 'Point', 'coordinates': center},
            'properties': lprops,
        })

    labels_path = os.path.join(outdir, 'street_labels.geojson')
    print(f"Writing {labels_path}...", file=sys.stderr)
    with open(labels_path, 'w', encoding='utf-8') as f:
        json.dump({'type': 'FeatureCollection', 'features': label_features}, f, separators=(',', ':'))

    labels_mb = os.path.getsize(labels_path) / (1024 * 1024)
    print(f"  {len(label_features):,} street labels ({labels_mb:.1f} MB)", file=sys.stderr)

    print("Done!", file=sys.stderr)

if __name__ == '__main__':
    main()
