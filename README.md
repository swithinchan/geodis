# geodis — 3D Pedestrian Network Fieldwork Assignment Engine

**C++ application that computes 3D walking distances on Hong Kong's 8,300 km
pedestrian network and optimizes fieldwork officer-to-site assignments.**

Built on the [HK Lands Department 3D Pedestrian Network](https://portal.csdi.gov.hk/geoportal/?datasetId=landsd_rcd_1637222018065_52265)
(465,475 route segments covering all built-up areas, including footways,
footbridges, subways, stairs, lifts, escalators, and indoor passages).

## Why C++?

The existing Python workflow takes **half a day** to sort by geographical
distance. This C++ engine:

- Loads the full 8,300km network as a memory-efficient CSR graph
- Snaps addresses to the nearest network node via R-tree spatial index
- Runs Dijkstra shortest-path with 3D walking cost (slope, stairs, lift penalties)
- Solves the officer→site assignment problem (Hungarian or greedy)
- **Completes in seconds, not hours**

## Quick Start

### 1. Build

```bash
# Requires: cmake, ninja, g++ (C++20), Boost, TBB (optional)
mkdir build && cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
ninja -j$(nproc)
```

### 2. Download the pedestrian network data

```bash
# Full download with checkpoint/resume (recommended — takes ~10-15 min)
python3 scripts/download_checkpoint.py -o data/graph.bin

# If rate-limited, resume:
python3 scripts/download_checkpoint.py -o data/graph.bin --resume

# Alternative: download GeoJSON manually from CSDI Portal then convert:
python3 scripts/preprocess.py pedestrian_route.json -o data/graph.bin
```

Data source: [CSDI Portal — 3D Pedestrian Network](https://portal.csdi.gov.hk/csdi-webpage/file-api?dataset_id=landsd_rcd_1637222018065_52265&format=geojson&layer_name=PedestrianRoute)

### 3. Run assignment

```bash
./build/geodis \
  --graph data/graph.bin \
  --officers officers.csv \
  --sites sites.csv \
  --mode hungarian \
  --output assignments.csv
```

**Input CSV format** (header required):
```csv
name,address,lon,lat,z
Officer_A,"Flat 1, Kennedy Town",114.1305,22.2847,4.0
Site_1,"Block A, Sai Ying Pun",114.1420,22.2850,30.0
```

**Output CSV:**
```csv
officer_name,officer_address,site_name,site_address,walking_cost,walking_distance_m,straight_line_m
Officer_A,"Flat 1, Kennedy Town",Site_1,"Block A, Sai Ying Pun",1245.3,1180.2,1050.0
```

## Architecture

```
geodis/
├── CMakeLists.txt              # CMake + Ninja build (cross-platform)
├── src/
│   ├── main.cpp                # CLI entry point
│   ├── spatial/
│   │   ├── point3d.hpp         # 3D point, haversine, walking cost model
│   │   └── rtree_index.hpp/cpp # Boost.Geometry R-tree for snapping
│   ├── graph/
│   │   └── network_graph.hpp/cpp  # CSR graph, binary loader
│   ├── routing/
│   │   └── dijkstra.hpp/cpp    # Dijkstra with 3D walking cost weights
│   └── assignment/
│       └── assignment_engine.hpp/cpp  # Hungarian + greedy assignment
├── scripts/
│   ├── download_checkpoint.py  # Full download with checkpoint/resume
│   ├── download_and_build.py   # Download + build in one pass
│   └── preprocess.py           # Convert GeoJSON → binary graph
└── data/
    └── graph.bin               # Binary graph (generated)
```

## Binary Graph Format

Little-endian binary with CSR adjacency:

```
Header:   magic[8]="GEODISG\x02" version[4] num_nodes[8] num_edges[8] reserved[4]
Node:     lon[8] lat[8] z[8] flags[4]  (× num_nodes, 28 bytes each)
Edge:     from[8] to[8] len2d[8] len3d[8] ascent[8] descent[8] grade[8] flags[4]
          (× num_edges, 60 bytes each)
```

## Walking Cost Model

The Dijkstra edge weight is the **effective walking cost** in equivalent flat meters:

| Feature      | Cost multiplier              |
|-------------|-----------------------------|
| Flat walk   | 1.0× (distance in meters)   |
| Uphill      | +2.0× ascent                |
| Stairs      | +3.0× ascent (on top)       |
| Escalator   | 0.5× distance               |
| Lift        | 10m flat cost + 0.5× ascent |

## Assignment Modes

- **`hungarian`** — Optimal 1-to-1 assignment (O(n³), best for ≤500 officers/sites)
- **`greedy`** — Nearest-neighbor (O(n²), fast for large N)

## Windows Build (winlib)

```powershell
# Using vcpkg for dependencies
vcpkg install boost-system tbb
mkdir build && cd build
cmake -G "Visual Studio 17 2022" -DCMAKE_TOOLCHAIN_FILE=<vcpkg_root>/scripts/buildsystems/vcpkg.cmake ..
cmake --build . --config Release
```

## Data Attribution

3D Pedestrian Network © Lands Department, HKSAR Government.
Licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
Accessed via the [CSDI Portal](https://portal.csdi.gov.hk).

## Requirements

- **C++20** (GCC 12+ or Clang 16+ or MSVC 2022+)
- **Boost** ≥1.75 (system component)
- **TBB** (optional, for parallel Dijkstra)
- **CMake** ≥3.21 + **Ninja**
- **Python 3.9+** (for data preprocessing scripts, `pip install requests`)
