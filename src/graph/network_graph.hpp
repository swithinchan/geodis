#pragma once

#include "spatial/point3d.hpp"
#include "spatial/rtree_index.hpp"

#include <cstdint>
#include <vector>
#include <string>
#include <span>
#include <optional>
#include <string_view>

namespace geodis {

/// ─── Binary-loaded 3D Pedestrian Network ──────────────────────────────────
///
/// Loads the preprocessed binary graph (from scripts/preprocess.py).
/// Stores nodes and edges in flat arrays with CSR adjacency for
/// cache-friendly shortest-path search.
///
/// Binary format (little-endian):
///   Header:  magic[8]="GEODISG\x02" version[4] nodes[8] edges[8] reserved[4]
///   Node:    lon[8] lat[8] z[8] flags[4]  (x N)
///   Edge:    from[8] to[8] len2d[8] len3d[8] ascent[8] descent[8] grade[8] flags[4] (x M)
class NetworkGraph {
public:
    NetworkGraph() = default;

    NetworkGraph(NetworkGraph&&) noexcept = default;
    NetworkGraph& operator=(NetworkGraph&&) noexcept = default;
    NetworkGraph(const NetworkGraph&) = delete;
    NetworkGraph& operator=(const NetworkGraph&) = delete;

    /// Load from preprocessed binary file.
    /// Throws std::runtime_error on failure.
    static NetworkGraph load(const std::string& path);

    /// ─── Accessors ───────────────────────────────────────────────────────
    [[nodiscard]] size_t node_count() const noexcept { return m_nodes.size(); }
    [[nodiscard]] size_t edge_count() const noexcept { return m_edges.size(); }

    [[nodiscard]] const NetworkNode& node(size_t idx) const { return m_nodes[idx]; }
    [[nodiscard]] const NetworkEdge& edge(size_t idx) const { return m_edges[idx]; }

    /// Span of edge indices for node_idx's outgoing edges.
    [[nodiscard]] std::span<const uint64_t> out_edges(size_t node_idx) const {
        size_t start = m_offsets[node_idx];
        size_t end   = m_offsets[node_idx + 1];
        return {m_adj.data() + start, end - start};
    }

    /// Snap a WGS84 point to the nearest network node.
    /// Returns {node_index, distance_meters} or nullopt.
    [[nodiscard]] std::optional<std::pair<uint64_t, double>>
    snap(const Point3D& pt) const {
        return m_spatial.query_nearest(pt);
    }

    /// Find k nearest nodes.
    [[nodiscard]] std::vector<std::pair<uint64_t, double>>
    snap_knn(const Point3D& pt, size_t k = 3) const {
        return m_spatial.query_knn(pt, k);
    }

    /// ─── Stats ───────────────────────────────────────────────────────────
    struct Stats {
        size_t num_nodes, num_edges;
        double total_2d_km, total_3d_km;
        size_t num_stairs, num_lifts, num_footbridges, num_subways;
        size_t num_indoor, num_barrier_free;
        double min_z, max_z;
    };
    [[nodiscard]] Stats stats() const;

    [[nodiscard]] bool empty() const noexcept { return m_nodes.empty(); }

private:
    void build_adjacency();

    std::vector<NetworkNode> m_nodes;
    std::vector<NetworkEdge> m_edges;

    // CSR: m_offsets[i] = start in m_adj for node i's outgoing edge indices
    std::vector<size_t>   m_offsets;
    std::vector<uint64_t> m_adj;  // edge indices

    SpatialIndex m_spatial;
};

} // namespace geodis
