#pragma once

#include "graph/network_graph.hpp"
#include "routing/dijkstra.hpp"

#include <vector>
#include <string>
#include <cstdint>
#include <span>

namespace geodis {

/// ─── A fieldwork site or officer location ─────────────────────────────────
struct Location {
    std::string name;        // identifier (e.g., TPU code, address)
    std::string address;     // human-readable address
    Point3D     pos;         // WGS84 coordinates
    uint64_t    snapped_node{0}; // nearest network node (filled by engine)
    double      snap_dist{0.0};  // distance from original point to network
};

/// ─── An assignment: one officer → one site ────────────────────────────────
struct Assignment {
    size_t officer_idx;
    size_t site_idx;
    double walking_cost;        // effective walking cost
    double walking_distance_m;  // 3D network distance
    double straight_line_m;     // haversine distance
    std::string officer_name;
    std::string site_name;
    std::string officer_address;
    std::string site_address;
    std::vector<Point3D>  path_geometry;   // walking route coordinates
    std::vector<uint32_t> path_edge_flags; // edge type per segment (for coloring)
    double path_ascent{0};
    double path_descent{0};
};

/// ─── Assignment Engine ─────────────────────────────────────────────────────
///
/// Takes a list of officers (with home locations) and fieldwork sites,
/// snaps them to the pedestrian network, computes all-pairs 3D walking
/// distances, and produces optimized assignments.
///
/// Supports three modes:
///   - HUNGARIAN: optimal 1-to-1 assignment (minimizes total walking cost)
///   - GREEDY:     fast nearest-neighbor assignment (good for large N)
///   - CLUSTER:    assign each site to its nearest officer (many-to-1)
class AssignmentEngine {
public:
    enum class Mode { HUNGARIAN, GREEDY, CLUSTER };

    explicit AssignmentEngine(const NetworkGraph& graph,
                              Mode mode = Mode::HUNGARIAN)
        : m_graph(&graph), m_dijkstra(graph), m_mode(mode) {}

    /// Run the full pipeline:
    ///   1. Snap all officer and site locations to the network
    ///   2. Compute all-pairs walking distances (officers × sites)
    ///   3. Optimize assignments
    ///   4. Return sorted list of assignments
    [[nodiscard]] std::vector<Assignment>
    assign(std::span<Location> officers,
           std::span<Location> sites);

    /// Access the distance matrix after assign().
    [[nodiscard]] const std::vector<std::vector<double>>&
    distance_matrix() const { return m_dist_matrix; }

    /// Set TBB thread count (if compiled with TBB). 0 = auto.
    void set_num_threads(int n) { m_num_threads = n; }

    /// Compute path geometry for assignments (requires extra Dijkstra runs).
    /// Set compute_geometry=true before calling assign().
    void set_compute_geometry(bool v) { m_compute_geometry = v; }

    /// Set the clustering load-balancing penalty (CLUSTER mode only).
    /// A site already assigned to an officer adds this relative penalty to
    /// that officer's score for the next site, so loads stay balanced.
    /// Example: 0.15 → each existing site adds a 15% distance penalty.
    void set_cluster_load_penalty(double p) { m_cluster_load_penalty = p; }

private:
    const NetworkGraph* m_graph;
    DijkstraEngine      m_dijkstra;
    Mode                m_mode;
    int                 m_num_threads{0};
    bool                m_compute_geometry{false};
    double              m_cluster_load_penalty{0.15};

    std::vector<std::vector<double>> m_dist_matrix;

    void snap_locations(std::span<Location> locs);
    void compute_distance_matrix(std::span<const Location> officers,
                                  std::span<const Location> sites);
    std::vector<Assignment> solve_hungarian(std::span<const Location> officers,
                                             std::span<const Location> sites);
    std::vector<Assignment> solve_greedy(std::span<const Location> officers,
                                          std::span<const Location> sites);
    std::vector<Assignment> solve_cluster(std::span<const Location> officers,
                                           std::span<const Location> sites);
};

} // namespace geodis
