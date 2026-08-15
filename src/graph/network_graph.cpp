#include "network_graph.hpp"

#include <fstream>
#include <stdexcept>
#include <cstring>
#include <algorithm>

namespace geodis {

static constexpr char MAGIC[8] = {'G','E','O','D','I','S','G','\x02'};
static constexpr uint32_t VERSION = 2;

NetworkGraph NetworkGraph::load(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        throw std::runtime_error("Cannot open: " + path);

    // Header
    char magic[8]; uint32_t ver; uint64_t nc, ec; uint32_t reserved;
    in.read(magic, 8);
    in.read(reinterpret_cast<char*>(&ver), 4);
    in.read(reinterpret_cast<char*>(&nc), 8);
    in.read(reinterpret_cast<char*>(&ec), 8);
    in.read(reinterpret_cast<char*>(&reserved), 4);

    if (std::memcmp(magic, MAGIC, 8) != 0)
        throw std::runtime_error("Bad magic bytes");
    if (ver != VERSION)
        throw std::runtime_error("Unsupported version: " + std::to_string(ver));

    NetworkGraph g;
    g.m_nodes.resize(nc);
    g.m_edges.resize(ec);

    // Read nodes: lon[8] lat[8] z[8] flags[4]
    for (auto& n : g.m_nodes) {
        in.read(reinterpret_cast<char*>(&n.lon), 8);
        in.read(reinterpret_cast<char*>(&n.lat), 8);
        in.read(reinterpret_cast<char*>(&n.z), 8);
        in.read(reinterpret_cast<char*>(&n.flags), 4);
    }

    // Read edges: from[8] to[8] len2d[8] len3d[8] ascent[8] descent[8] grade[8] flags[4]
    for (auto& e : g.m_edges) {
        in.read(reinterpret_cast<char*>(&e.from_node), 8);
        in.read(reinterpret_cast<char*>(&e.to_node), 8);
        in.read(reinterpret_cast<char*>(&e.length_2d), 8);
        in.read(reinterpret_cast<char*>(&e.length_3d), 8);
        in.read(reinterpret_cast<char*>(&e.ascent), 8);
        in.read(reinterpret_cast<char*>(&e.descent), 8);
        in.read(reinterpret_cast<char*>(&e.grade), 8);
        in.read(reinterpret_cast<char*>(&e.flags), 4);
    }

    if (!in)
        throw std::runtime_error("Truncated file: " + path);

    g.build_adjacency();
    return g;
}

void NetworkGraph::build_adjacency() {
    size_t n = m_nodes.size();

    // Count out-degree
    std::vector<size_t> degree(n, 0);
    for (const auto& e : m_edges) {
        if (e.from_node < n) degree[e.from_node]++;
    }

    // Prefix sum → offsets
    m_offsets.resize(n + 1, 0);
    for (size_t i = 0; i < n; ++i)
        m_offsets[i + 1] = m_offsets[i] + degree[i];

    // Fill adjacency
    m_adj.resize(m_edges.size());
    auto cursor = m_offsets;
    for (size_t ei = 0; ei < m_edges.size(); ++ei) {
        size_t from = m_edges[ei].from_node;
        if (from < n) {
            m_adj[cursor[from]++] = ei;
        }
    }

    // Build spatial index
    std::vector<std::pair<uint64_t, Point3D>> pts;
    pts.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const auto& nd = m_nodes[i];
        pts.emplace_back(i, Point3D{nd.lon, nd.lat, nd.z});
    }
    m_spatial.build(pts);
}

NetworkGraph::Stats NetworkGraph::stats() const {
    Stats s{};
    s.num_nodes = m_nodes.size();
    s.num_edges = m_edges.size();
    s.min_z = 1e18;
    s.max_z = -1e18;

    for (const auto& n : m_nodes) {
        if (n.z < s.min_z) s.min_z = n.z;
        if (n.z > s.max_z) s.max_z = n.z;
    }

    for (const auto& e : m_edges) {
        s.total_2d_km += e.length_2d;
        s.total_3d_km += e.length_3d;
        if (e.flags & EDGE_STAIRS)     s.num_stairs++;
        if (e.flags & EDGE_LIFT)       s.num_lifts++;
        if (e.flags & EDGE_FOOTBRIDGE) s.num_footbridges++;
        if (e.flags & EDGE_SUBWAY)     s.num_subways++;
        if (e.flags & EDGE_INDOOR)     s.num_indoor++;
        if (!(e.flags & EDGE_BARRIER)) s.num_barrier_free++;
    }

    s.total_2d_km /= 1000.0;
    s.total_3d_km /= 1000.0;
    return s;
}

} // namespace geodis
