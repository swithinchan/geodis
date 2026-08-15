#include "dijkstra.hpp"
#include <stdexcept>

namespace geodis {

ShortestPath DijkstraResult::path_to(uint64_t target) const {
    ShortestPath sp;
    if (target >= dist.size() || dist[target] >= 1e18) {
        sp.reached = false;
        return sp;
    }
    sp.reached = true;
    sp.total_cost = dist[target];

    uint64_t cur = target;
    while (cur != source) {
        sp.nodes.push_back(cur);
        sp.edges.push_back(parent_edge[cur]);
        cur = parent_node[cur];
    }
    sp.nodes.push_back(source);

    std::ranges::reverse(sp.nodes);
    std::ranges::reverse(sp.edges);
    return sp;
}

// ─── DijkstraEngine ──────────────────────────────────────────────────────────

void DijkstraEngine::fill_path_details(ShortestPath& sp) const {
    if (!sp.reached || sp.nodes.empty()) return;

    // Geometry from node positions
    sp.geometry.reserve(sp.nodes.size());
    for (uint64_t nid : sp.nodes) {
        const auto& nd = m_graph->node(nid);
        sp.geometry.emplace_back(Point3D{nd.lon, nd.lat, nd.z});
    }

    // Edge flags from edge data
    sp.edge_flags.reserve(sp.edges.size());
    double total_3d = 0, total_asc = 0, total_desc = 0;
    for (uint64_t ei : sp.edges) {
        const auto& e = m_graph->edge(ei);
        sp.edge_flags.push_back(e.flags);
        total_3d += e.length_3d;
        total_asc += e.ascent;
        total_desc += e.descent;
    }
    sp.total_length_3d = total_3d;
    sp.total_ascent = total_asc;
    sp.total_descent = total_desc;
}

DijkstraResult DijkstraEngine::compute(
    uint64_t source, std::optional<uint64_t> target) const
{
    size_t n = m_graph->node_count();
    if (source >= n) throw std::runtime_error("Source node out of range");

    constexpr double kInf = 1e18;
    DijkstraResult r;
    r.dist.resize(n, kInf);
    r.parent_node.resize(n, UINT64_MAX);
    r.parent_edge.resize(n, UINT64_MAX);
    r.source = source;

    std::priority_queue<HeapEntry> pq;
    r.dist[source] = 0.0;
    pq.push({0.0, source});

    while (!pq.empty()) {
        auto [cost, u] = pq.top(); pq.pop();
        if (cost > r.dist[u]) continue;  // stale
        if (target && u == *target) break;

        for (uint64_t ei : m_graph->out_edges(u)) {
            const auto& e = m_graph->edge(ei);
            uint64_t v = e.to_node;
            if (v >= n) continue;
            double w = walking_cost(e.length_3d, e.ascent, e.flags);
            double nd = cost + w;
            if (nd < r.dist[v]) {
                r.dist[v] = nd;
                r.parent_node[v] = u;
                r.parent_edge[v] = ei;
                pq.push({nd, v});
            }
        }
    }
    return r;
}

ShortestPath DijkstraEngine::shortest_path(uint64_t source, uint64_t target) const {
    auto r = compute(source, target);
    return r.path_to(target);
}

ShortestPath DijkstraEngine::shortest_path_point(
    const Point3D& from, const Point3D& to) const
{
    auto s1 = m_graph->snap(from);
    auto s2 = m_graph->snap(to);
    ShortestPath sp;
    if (!s1 || !s2) { sp.reached = false; return sp; }
    return shortest_path(s1->first, s2->first);
}

std::vector<double> DijkstraEngine::distances_to(
    uint64_t source, std::span<const uint64_t> targets) const
{
    auto r = compute(source);
    std::vector<double> out;
    out.reserve(targets.size());
    for (auto t : targets) {
        out.push_back(t < r.dist.size() ? r.dist[t] : 1e18);
    }
    return out;
}

} // namespace geodis
