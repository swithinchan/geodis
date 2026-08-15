#include "assignment_engine.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <iostream>
#include <cmath>

#ifdef GEODIS_HAS_TBB
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#endif

namespace geodis {

// ─── Snapping ────────────────────────────────────────────────────────────────

void AssignmentEngine::snap_locations(std::span<Location> locs) {
    for (auto& loc : locs) {
        auto s = m_graph->snap(loc.pos);
        if (s) {
            loc.snapped_node = s->first;
            loc.snap_dist = s->second;
        } else {
            loc.snapped_node = UINT64_MAX;
            loc.snap_dist = 1e18;
        }
    }
}

// ─── Distance Matrix ─────────────────────────────────────────────────────────

void AssignmentEngine::compute_distance_matrix(
    std::span<const Location> officers,
    std::span<const Location> sites)
{
    size_t O = officers.size();
    size_t S = sites.size();

    m_dist_matrix.assign(O, std::vector<double>(S, 1e18));

    // For each officer, run one Dijkstra and extract distances to all sites
#ifdef GEODIS_HAS_TBB
    tbb::parallel_for(tbb::blocked_range<size_t>(0, O),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t o = r.begin(); o != r.end(); ++o) {
#else
    for (size_t o = 0; o < O; ++o) {
#endif
                if (officers[o].snapped_node == UINT64_MAX) continue;

                // Collect unique target nodes
                std::vector<uint64_t> targets;
                targets.reserve(S);
                for (size_t s = 0; s < S; ++s) {
                    if (sites[s].snapped_node != UINT64_MAX)
                        targets.push_back(sites[s].snapped_node);
                }

                auto dists = m_dijkstra.distances_to(officers[o].snapped_node, targets);

                // Map back to site indices
                size_t ti = 0;
                for (size_t s = 0; s < S; ++s) {
                    if (sites[s].snapped_node != UINT64_MAX && ti < dists.size()) {
                        m_dist_matrix[o][s] = dists[ti];
                        ti++;
                    }
                }
#ifdef GEODIS_HAS_TBB
            }
        });
#else
    }
#endif
}

// ─── Hungarian Algorithm (O(n³), optimal for n ≤ ~500) ──────────────────────

std::vector<Assignment> AssignmentEngine::solve_hungarian(
    std::span<const Location> officers,
    std::span<const Location> sites)
{
    size_t O = officers.size();
    size_t S = sites.size();
    size_t N = std::max(O, S);

    // Pad to square matrix
    std::vector<std::vector<double>> cost(N, std::vector<double>(N, 1e18));
    for (size_t i = 0; i < O; ++i)
        for (size_t j = 0; j < S; ++j)
            cost[i][j] = m_dist_matrix[i][j];

    // Hungarian algorithm implementation
    std::vector<double> u(N + 1, 0), v(N + 1, 0);
    std::vector<int> p(N + 1, 0), way(N + 1, 0);

    for (size_t i = 1; i <= N; ++i) {
        p[0] = static_cast<int>(i);
        int j0 = 0;
        std::vector<double> minv(N + 1, 1e18);
        std::vector<char> used(N + 1, 0);

        do {
            used[j0] = 1;
            int i0 = p[j0];
            double delta = 1e18;
            int j1 = 0;

            for (size_t j = 1; j <= N; ++j) {
                if (!used[j]) {
                    double cur = cost[i0 - 1][j - 1] - u[i0] - v[static_cast<int>(j)];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = static_cast<int>(j);
                    }
                }
            }

            for (int j = 0; j <= static_cast<int>(N); ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    // Build assignments
    std::vector<Assignment> result;
    result.reserve(O);
    for (size_t j = 1; j <= N; ++j) {
        int i = p[j];
        if (i > 0 && static_cast<size_t>(i - 1) < O && static_cast<size_t>(j - 1) < S) {
            size_t oi = static_cast<size_t>(i - 1);
            size_t si = static_cast<size_t>(j - 1);

            Assignment a;
            a.officer_idx = oi;
            a.site_idx = si;
            a.walking_cost = m_dist_matrix[oi][si];
            a.walking_distance_m = m_dist_matrix[oi][si];
            a.straight_line_m = haversine_m(officers[oi].pos.lon, officers[oi].pos.lat,
                                             sites[si].pos.lon, sites[si].pos.lat);
            a.officer_name = officers[oi].name;
            a.site_name = sites[si].name;
            a.officer_address = officers[oi].address;
            a.site_address = sites[si].address;

            // Compute path geometry if requested
            if (m_compute_geometry && a.walking_cost < 1e17) {
                auto sp = m_dijkstra.shortest_path(
                    officers[oi].snapped_node, sites[si].snapped_node);
                m_dijkstra.fill_path_details(sp);
                a.path_geometry = std::move(sp.geometry);
                a.path_edge_flags = std::move(sp.edge_flags);
                a.path_ascent = sp.total_ascent;
                a.path_descent = sp.total_descent;
            }
            result.push_back(a);
        }
    }

    // Sort by walking cost
    std::ranges::sort(result, {}, &Assignment::walking_cost);
    return result;
}

// ─── Greedy Assignment ───────────────────────────────────────────────────────

std::vector<Assignment> AssignmentEngine::solve_greedy(
    std::span<const Location> officers,
    std::span<const Location> sites)
{
    size_t O = officers.size();
    size_t S = sites.size();

    std::vector<Assignment> result;
    result.reserve(O);

    std::vector<char> site_assigned(S, 0);

    // For each officer, pick the closest unassigned site
    for (size_t o = 0; o < O; ++o) {
        double best_cost = 1e18;
        size_t best_s = 0;

        for (size_t s = 0; s < S; ++s) {
            if (!site_assigned[s] && m_dist_matrix[o][s] < best_cost) {
                best_cost = m_dist_matrix[o][s];
                best_s = s;
            }
        }

        if (best_cost < 1e17) {
            Assignment a;
            a.officer_idx = o;
            a.site_idx = best_s;
            a.walking_cost = best_cost;
            a.walking_distance_m = best_cost;
            a.straight_line_m = haversine_m(officers[o].pos.lon, officers[o].pos.lat,
                                             sites[best_s].pos.lon, sites[best_s].pos.lat);
            a.officer_name = officers[o].name;
            a.site_name = sites[best_s].name;
            a.officer_address = officers[o].address;
            a.site_address = sites[best_s].address;

            if (m_compute_geometry) {
                auto sp = m_dijkstra.shortest_path(
                    officers[o].snapped_node, sites[best_s].snapped_node);
                m_dijkstra.fill_path_details(sp);
                a.path_geometry = std::move(sp.geometry);
                a.path_edge_flags = std::move(sp.edge_flags);
                a.path_ascent = sp.total_ascent;
                a.path_descent = sp.total_descent;
            }
            result.push_back(a);
            site_assigned[best_s] = 1;
        }
    }

    std::ranges::sort(result, {}, &Assignment::walking_cost);
    return result;
}

// ─── Main pipeline ───────────────────────────────────────────────────────────

std::vector<Assignment> AssignmentEngine::assign(
    std::span<Location> officers,
    std::span<Location> sites)
{
    if (officers.empty() || sites.empty())
        throw std::runtime_error("Need at least one officer and one site");

    std::cerr << "Snapping " << officers.size() << " officer locations..." << std::endl;
    snap_locations(officers);

    std::cerr << "Snapping " << sites.size() << " site locations..." << std::endl;
    snap_locations(sites);

    std::cerr << "Computing " << officers.size() << "×" << sites.size()
              << " distance matrix..." << std::endl;
    compute_distance_matrix(officers, sites);

    std::cerr << "Solving assignments ("
              << (m_mode == Mode::HUNGARIAN ? "Hungarian" :
                  m_mode == Mode::CLUSTER   ? "Cluster (nearest officer)" : "Greedy")
              << ")..." << std::endl;

    if (m_mode == Mode::HUNGARIAN) {
        return solve_hungarian(officers, sites);
    } else if (m_mode == Mode::CLUSTER) {
        return solve_cluster(officers, sites);
    } else {
        return solve_greedy(officers, sites);
    }
}

// ── Cluster solver (each site → nearest officer) ─────────────────────────

std::vector<Assignment> AssignmentEngine::solve_cluster(
    std::span<const Location> officers,
    std::span<const Location> sites)
{
    size_t O = officers.size();
    size_t S = sites.size();

    // Balanced many-to-1 assignment: assign each site to its nearest officer,
    // but apply a load-balancing penalty so a few officers don't get overloaded.
    std::vector<std::vector<Assignment>> officer_assignments(O);
    std::vector<size_t> load(O, 0);  // sites currently assigned per officer

    for (size_t s = 0; s < S; ++s) {
        double best_score = 1e18;
        size_t best_o = 0;

        for (size_t o = 0; o < O; ++o) {
            // Relative load penalty: each existing assignment adds
            // m_cluster_load_penalty to this officer's effective distance.
            double score = m_dist_matrix[o][s] * (1.0 + m_cluster_load_penalty * load[o]);
            if (score < best_score) {
                best_score = score;
                best_o = o;
            }
        }

        if (best_score < 1e17) {
            Assignment a;
            a.officer_idx = best_o;
            a.site_idx = s;
            a.walking_cost = m_dist_matrix[best_o][s];
            a.walking_distance_m = a.walking_cost;
            a.straight_line_m = haversine_m(officers[best_o].pos.lon, officers[best_o].pos.lat,
                                             sites[s].pos.lon, sites[s].pos.lat);
            a.officer_name = officers[best_o].name;
            a.site_name = sites[s].name;
            a.officer_address = officers[best_o].address;
            a.site_address = sites[s].address;

            if (m_compute_geometry) {
                auto sp = m_dijkstra.shortest_path(
                    officers[best_o].snapped_node, sites[s].snapped_node);
                m_dijkstra.fill_path_details(sp);
                a.path_geometry = std::move(sp.geometry);
                a.path_edge_flags = std::move(sp.edge_flags);
                a.path_ascent = sp.total_ascent;
                a.path_descent = sp.total_descent;
            }
            officer_assignments[best_o].push_back(a);
            load[best_o]++;
        }
    }

    // Sort each officer's assignments by walking cost, then flatten
    std::vector<Assignment> result;
    for (auto& oa : officer_assignments) {
        std::ranges::sort(oa, {}, &Assignment::walking_cost);
        for (auto& a : oa) result.push_back(std::move(a));
    }
    return result;
}

} // namespace geodis
