#!/usr/bin/env python3
"""
build_street_districts.py — Group the government Road Centreline street names
by Hong Kong's 18 districts.

The converted Road Centreline GeoJSON has street names + geometry but no
district column, so we spatially join each road segment's centroid to the
18 district boundaries (OSM admin_level=6) and group unique street names.

Input:
  /home/switc/Downloads/Transportation_RoadCentreline_20260803_gdb_RoadCentreLine_converted.geojson
Output:
  data/street_names_by_district.json
"""

import json, os, sys, urllib.parse, urllib.request
from shapely.geometry import Point, LineString, Polygon, MultiPolygon
from shapely.ops import polygonize, linemerge

ROADS = os.path.expanduser(
    '~/Downloads/Transportation_RoadCentreline_20260803_gdb_RoadCentreLine_converted.geojson')
OVERPASS = 'https://overpass.kumi.systems/api/interpreter'

# OSM district name -> our display name
DISTRICT_MAP = {
    'Central and Western District': 'Central and Western',
    'Eastern District': 'Eastern',
    'Southern District': 'Southern',
    'Wan Chai District': 'Wan Chai',
    'Kowloon City District': 'Kowloon City',
    'Yau Tsim Mong District': 'Yau Tsim Mong',
    'Sham Shui Po District': 'Sham Shui Po',
    'Wong Tai Sin District': 'Wong Tai Sin',
    'Kwun Tong District': 'Kwun Tong',
    'Kwai Tsing District': 'Kwai Tsing',
    'Tsuen Wan District': 'Tsuen Wan',
    'Tuen Mun District': 'Tuen Mun',
    'Yuen Long District': 'Yuen Long',
    'North District': 'North',
    'Tai Po District': 'Tai Po',
    'Sha Tin District': 'Sha Tin',
    'Sai Kung District': 'Sai Kung',
    'Islands District': 'Islands',
}

def fetch_district_polygons():
    """Return list of (display_name, shapely_polygon)."""
    q = ('[out:json][timeout:180];'
         'area["name:en"="Hong Kong"]->.hk;'
         '(relation["admin_level"="6"]["boundary"="administrative"](area.hk););'
         'out geom;')
    data = urllib.parse.urlencode({'data': q}).encode()
    req = urllib.request.Request(OVERPASS, data=data, headers={'User-Agent':'geodis/1.0'})
    with urllib.request.urlopen(req, timeout=200) as r:
        d = json.loads(r.read().decode())

    polygons = []
    for rel in d.get('elements', []):
        if rel.get('type') != 'relation':
            continue
        name = rel.get('tags', {}).get('name:en')
        if name not in DISTRICT_MAP:
            continue
        outer_lines = []
        for m in rel.get('members', []):
            if m.get('type') == 'way' and m.get('role') == 'outer' and 'geometry' in m:
                coords = [(pt['lon'], pt['lat']) for pt in m['geometry'] if 'lon' in pt]
                if len(coords) >= 2:
                    outer_lines.append(LineString(coords))
        if not outer_lines:
            continue
        polys = list(polygonize(outer_lines))
        if polys:
            poly = MultiPolygon([p for p in polys if p.is_valid]) if len(polys) > 1 else polys[0]
            polygons.append((DISTRICT_MAP[name], poly))
    return polygons

def main():
    out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'data', 'street_names_by_district.json')
    print('Fetching 18 district boundaries...', file=sys.stderr, flush=True)
    polygons = fetch_district_polygons()
    print(f'  got {len(polygons)} district polygons', file=sys.stderr, flush=True)

    print('Loading road centreline...', file=sys.stderr, flush=True)
    with open(ROADS, encoding='utf-8') as f:
        roads = json.load(f)['features']
    print(f'  {len(roads)} road segments', file=sys.stderr, flush=True)

    # Assign each named road segment to a district via centroid (STRtree index)
    from collections import defaultdict
    from shapely.strtree import STRtree
    district_streets = defaultdict(set)
    dnames = [d for d, _ in polygons]
    polys = [p for _, p in polygons]
    tree = STRtree(polys)
    matched = 0
    unmatched = 0
    for f in roads:
        name = f['properties'].get('ENGLISHSTREETNAME')
        if not name:
            continue
        coords = f['geometry']['coordinates']
        if not coords:
            continue
        lons = [c[0] for c in coords]
        lats = [c[1] for c in coords]
        pt = Point(sum(lons)/len(lons), sum(lats)/len(lats))
        assigned = False
        # query bbox candidates, then precise contains
        for cand_idx in tree.query(pt):
            poly = polys[cand_idx]
            if poly.contains(pt):
                district_streets[dnames[cand_idx]].add(name)
                assigned = True
                break
        if assigned:
            matched += 1
        else:
            unmatched += 1
    print(f'  matched={matched}, unmatched={unmatched}', file=sys.stderr, flush=True)

    result = {d: sorted(district_streets.get(d, [])) for d in DISTRICT_MAP.values()}
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=1)
    for d in result:
        print(f'  {d}: {len(result[d])} streets', file=sys.stderr, flush=True)
    print('WROTE', out_path, file=sys.stderr, flush=True)

if __name__ == '__main__':
    main()
