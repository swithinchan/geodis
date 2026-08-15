#pragma once

#include "point3d.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <cstdint>
#include <vector>
#include <utility>
#include <optional>

namespace geodis {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using BgPoint = bg::model::point<double, 2, bg::cs::cartesian>;
using BgBox   = bg::model::box<BgPoint>;
using RTreeValue = std::pair<BgPoint, uint64_t>;
using RTree = bgi::rtree<RTreeValue, bgi::quadratic<32>>;

/// Spatial index for network nodes.
/// Coordinates are converted to approximate meters (relative to HK) for
/// accurate Euclidean distance queries in the R-tree.
class SpatialIndex {
public:
    SpatialIndex() = default;

    void build(const std::vector<std::pair<uint64_t, Point3D>>& nodes);

    [[nodiscard]] std::vector<std::pair<uint64_t, double>>
    query_knn(const Point3D& query, size_t k = 1) const;

    [[nodiscard]] std::optional<std::pair<uint64_t, double>>
    query_nearest(const Point3D& query) const;

    [[nodiscard]] size_t size() const noexcept { return m_count; }
    [[nodiscard]] bool empty() const noexcept { return m_count == 0; }

private:
    // Convert WGS84 to approximate meters for R-tree indexing
    static BgPoint to_meters(const Point3D& p) noexcept {
        // Reference: HK center ~22.3°N 114.1°E
        double x = (p.lon - 114.1) * 111320.0 * 0.925; // cos(22.3°)
        double y = (p.lat - 22.3) * 110540.0;
        return BgPoint(x, y);
    }

    RTree  m_rtree;
    size_t m_count{0};
};

} // namespace geodis
