#pragma once

#include "graph/network_graph.hpp"

#include <vector>
#include <queue>
#include <limits>
#include <cstdint>
#include <span>
#include <optional>

namespace geodis {

/// Result of a shortest-path query.
struct ShortestPath {
    std::vector<uint64_t> nodes;       // node indices from source to target
    std::vector<uint64_t> edges;       // edge indices along the path
    std::vector<uint32_t> edge_flags;  // flags for each edge (for type coloring)
    std::vector<Point3D>  geometry;    // coordinates along the path (for visualization)
    double total_cost{0.0};            // effective walking cost
    double total_length_3d{0.0};       // actual 3D distance (meters)
    double total_ascent{0.0};
    double total_descent{0.0};
    bool   reached{false};
};

/// One-to-all Dijkstra result.
struct DijkstraResult {
    std::vector<double>   dist;
    std::vector<uint64_t> parent_node;
    std::vector<uint64_t> parent_edge;
    uint64_t source{0};

    [[nodiscard]] ShortestPath path_to(uint64_t target) const;
};

/// ─── Dijkstra Routing Engine ──────────────────────────────────────────────
///
/// Computes shortest paths on the 3D pedestrian network using edge
/// walking_cost() as weight. Uses lazy-decrease-key binary heap.
class DijkstraEngine {
public:
    explicit DijkstraEngine(const NetworkGraph& g) : m_graph(&g) {}

    /// One-to-all Dijkstra. Optionally stops early at target.
    [[nodiscard]] DijkstraResult
    compute(uint64_t source, std::optional<uint64_t> target = {}) const;

    /// Single shortest path, source→target (node indices).
    [[nodiscard]] ShortestPath
    shortest_path(uint64_t source, uint64_t target) const;

    /// Shortest path from one snapped point to another.
    [[nodiscard]] ShortestPath
    shortest_path_point(const Point3D& from, const Point3D& to) const;

    /// Build geometry (Point3D list) for a path's node sequence.
    /// Also populates edge_flags from the graph.
    void fill_path_details(ShortestPath& sp) const;

    /// Distances from source to many targets (one Dijkstra run).
    [[nodiscard]] std::vector<double>
    distances_to(uint64_t source, std::span<const uint64_t> targets) const;

private:
    const NetworkGraph* m_graph;

    struct HeapEntry {
        double cost;
        uint64_t node;
        bool operator<(const HeapEntry& o) const { return cost > o.cost; }
    };
};

} // namespace geodis
