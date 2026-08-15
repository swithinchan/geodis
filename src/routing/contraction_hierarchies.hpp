#pragma once

#include "graph/network_graph.hpp"
#include "dijkstra.hpp"

#include <vector>
#include <cstdint>
#include <limits>

namespace geodis {

/// ─── Contraction Hierarchies ─────────────────────────────────────────────────
///
/// Preprocesses the pedestrian network to accelerate many-to-many shortest-path
/// queries. Nodes are ordered by "importance" and contracted: each contracted
/// node is removed from the graph, and shortcut edges are added to preserve
/// shortest-path distances.
///
/// After preprocessing, point-to-point queries run bidirectional Dijkstra
/// only on the overlay graph, skipping low-importance nodes. This reduces
/// query time from O(E log V) to O(√E log V) in practice.
///
/// Memory: roughly 2–3× the original graph (shortcut edges).
/// Build time: O(N log N) for the 8,300 km HK network (a few minutes).
class ContractionHierarchies {
public:
    explicit ContractionHierarchies(const NetworkGraph& base_graph);

    /// Run the preprocessing (node ordering + contraction).
    /// progress_callback(percent, step_description) called periodically.
    void build(std::function<void(int, std::string_view)> progress_cb = {});

    /// Query shortest path using bidirectional CH Dijkstra.
    [[nodiscard]] ShortestPath query(uint64_t source, uint64_t target) const;

    /// Query distance only (faster than full path).
    [[nodiscard]] double query_distance(uint64_t source, uint64_t target) const;

    /// Batch query: compute distances from one source to many targets.
    [[nodiscard]] std::vector<double>
    query_many_to_one(std::span<const uint64_t> sources, uint64_t target) const;

    /// Whether preprocessing has completed.
    [[nodiscard]] bool is_ready() const noexcept { return m_ready; }

    /// ─── Serialization ────────────────────────────────────────────────────
    void save(const std::string& path) const;
    static ContractionHierarchies load(const std::string& path,
                                       const NetworkGraph& base_graph);

private:
    const NetworkGraph* m_base;
    bool m_ready{false};

    // Node ordering: rank[node] = position in contraction order
    // Lower rank = contracted earlier = less important
    std::vector<uint32_t> m_rank;

    // Shortcut edges added during contraction
    struct ShortcutEdge {
        uint64_t from, to;
        double cost;
        uint64_t middle_node; // the contracted node this shortcut bypasses
    };
    std::vector<ShortcutEdge> m_shortcuts;

    // Forward/backward adjacency for the augmented graph
    std::vector<std::vector<std::pair<uint64_t, double>>> m_fwd_adj;
    std::vector<std::vector<std::pair<uint64_t, double>>> m_bwd_adj;

    // Internal: edge difference heuristic for node ordering
    [[nodiscard]] int compute_edge_difference(uint64_t node) const;

    // Internal: contract a single node
    void contract_node(uint64_t node, uint32_t rank);

    // Internal: run a local Dijkstra limited by rank
    struct LocalDijkstraResult {
        std::vector<double> dist;
        std::vector<uint64_t> parent;
    };
    [[nodiscard]] LocalDijkstraResult
    local_dijkstra(uint64_t source, uint32_t max_rank,
                   const std::vector<std::vector<std::pair<uint64_t, double>>>& adj) const;
};

} // namespace geodis
