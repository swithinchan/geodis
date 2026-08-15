#pragma once

#include <cstdint>
#include <cmath>
#include <compare>
#include <numbers>

namespace geodis {

/// 3D point in WGS84 decimal degrees + elevation in meters (HK Principal Datum).
struct Point3D {
    double lon{0.0};  // longitude (WGS84)
    double lat{0.0};  // latitude  (WGS84)
    double z{0.0};    // elevation in meters (HKPD)

    constexpr Point3D() noexcept = default;
    constexpr Point3D(double lo, double la, double el = 0.0) noexcept
        : lon(lo), lat(la), z(el) {}

    auto operator<=>(const Point3D&) const = default;
};

/// ─── Haversine distance (meters) ──────────────────────────────────────────
inline constexpr double EARTH_R = 6'371'000.0;
inline constexpr double D2R = std::numbers::pi / 180.0;

inline double haversine_m(double lon1, double lat1,
                          double lon2, double lat2) noexcept {
    double dlat = (lat2 - lat1) * D2R;
    double dlon = (lon2 - lon1) * D2R;
    double a = std::sin(dlat * 0.5) * std::sin(dlat * 0.5)
             + std::cos(lat1 * D2R) * std::cos(lat2 * D2R)
             * std::sin(dlon * 0.5) * std::sin(dlon * 0.5);
    return 2.0 * EARTH_R * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

/// Approximate WGS84→meters conversion near HK (lat ~22.3°).
inline double deg_to_m_lon(double dlon, double lat) noexcept {
    return dlon * EARTH_R * D2R * std::cos(lat * D2R);
}
inline double deg_to_m_lat(double dlat) noexcept {
    return dlat * EARTH_R * D2R;
}

/// 3D Euclidean distance in approximate meters.
inline double distance_3d_m(const Point3D& a, const Point3D& b) noexcept {
    double dx = deg_to_m_lon(b.lon - a.lon, (a.lat + b.lat) * 0.5);
    double dy = deg_to_m_lat(b.lat - a.lat);
    double dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// ─── Node in the network (as stored in binary) ────────────────────────────
struct NetworkNode {
    double lon, lat, z;
    uint32_t flags;
};

/// Node flags
enum NodeFlag : uint32_t {
    NODE_JUNCTION   = 0,
    NODE_STAIR_UP   = 1 << 0,
    NODE_STAIR_DOWN = 1 << 1,
    NODE_LIFT       = 1 << 2,
    NODE_CROSSING   = 1 << 3,
    NODE_ENTRANCE   = 1 << 4,
    NODE_INDOOR     = 1 << 5,
    NODE_BARRIER    = 1 << 6,
};

/// ─── Edge in the network ──────────────────────────────────────────────────
struct NetworkEdge {
    uint64_t from_node;
    uint64_t to_node;
    double length_2d;   // meters (2D)
    double length_3d;   // meters (3D, accounting for slope)
    double ascent;      // meters up
    double descent;     // meters down
    double grade;       // slope as fraction (0 = flat)
    uint32_t flags;
};

/// Edge flags
enum EdgeFlag : uint32_t {
    EDGE_WALK       = 0,
    EDGE_STAIRS     = 1 << 0,
    EDGE_LIFT       = 1 << 1,
    EDGE_ESCALATOR  = 1 << 2,
    EDGE_FOOTBRIDGE = 1 << 3,
    EDGE_SUBWAY     = 1 << 4,
    EDGE_CROSSING   = 1 << 5,
    EDGE_INDOOR     = 1 << 6,
    EDGE_COVERED    = 1 << 7,
    EDGE_STEEP      = 1 << 8,
    EDGE_BARRIER    = 1 << 9,
};

/// Walking cost in equivalent flat meters.
///
/// Cost model (tuned for urban fieldwork):
///   - flat walking: 1x the 3D length
///   - walking UPHILL on a footway: strong penalty (avoid walking up the hill)
///   - stairs (footsteps): extra penalty per metre of ascent
///   - steep footpath (>10% grade): heavy penalty (avoid walking on hills)
///   - lift / escalator: assisted, so cheaper than climbing stairs
inline double walking_cost(double len_3d, double ascent, uint32_t flags) noexcept {
    double cost = len_3d;

    // Uphill penalty — walking up a hill is far more strenuous than flat.
    cost += ascent * 5.0;

    // Footsteps / stairs penalty — climbing stairs is expensive.
    if (flags & EDGE_STAIRS)    cost += ascent * 8.0;

    // Assisted vertical movement is cheap.
    if (flags & EDGE_LIFT)      cost = 10.0 + ascent * 0.5;  // waiting + lift
    if (flags & EDGE_ESCALATOR) cost *= 0.5;                 // assisted

    // Steep hillside footpath — avoid walking on the hill.
    if (flags & EDGE_STEEP)     cost *= 8.0;

    return cost;
}

} // namespace geodis
