#include "rtree_index.hpp"
#include <algorithm>
#include <cmath>

namespace geodis {

void SpatialIndex::build(const std::vector<std::pair<uint64_t, Point3D>>& nodes) {
    m_rtree.clear();
    m_count = nodes.size();

    std::vector<RTreeValue> values;
    values.reserve(nodes.size());
    for (const auto& [id, pt] : nodes) {
        values.emplace_back(to_meters(pt), id);
    }
    m_rtree = RTree(values.begin(), values.end());
}

std::vector<std::pair<uint64_t, double>>
SpatialIndex::query_knn(const Point3D& query, size_t k) const {
    if (empty()) return {};

    BgPoint qpt = to_meters(query);
    std::vector<RTreeValue> results;
    m_rtree.query(bgi::nearest(qpt, static_cast<unsigned>(k)),
                  std::back_inserter(results));

    std::vector<std::pair<uint64_t, double>> out;
    out.reserve(results.size());
    for (const auto& [pt, id] : results) {
        double dx = qpt.get<0>() - pt.get<0>();
        double dy = qpt.get<1>() - pt.get<1>();
        out.emplace_back(id, std::sqrt(dx * dx + dy * dy));
    }
    return out;
}

std::optional<std::pair<uint64_t, double>>
SpatialIndex::query_nearest(const Point3D& query) const {
    auto knn = query_knn(query, 1);
    if (knn.empty()) return std::nullopt;
    return knn.front();
}

} // namespace geodis
