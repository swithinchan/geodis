#pragma once

#include "point3d.hpp"

#include <cmath>
#include <numbers>
#include <utility>

namespace geodis {

/// ─── Earth constants ────────────────────────────────────────────────────────
inline constexpr double EARTH_RADIUS_M = 6'371'000.0;
inline constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;
inline constexpr double RAD_TO_DEG = 180.0 / std::numbers::pi;

/// ─── Haversine distance (meters) between two WGS84 lat/lon points ──────────
/// Accurate to ~0.5% for distances up to thousands of km.
[[nodiscard]] inline double haversine_meters(double lon1, double lat1,
                                              double lon2, double lat2) noexcept {
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double a = std::sin(dlat * 0.5) * std::sin(dlat * 0.5)
             + std::cos(lat1 * DEG_TO_RAD) * std::cos(lat2 * DEG_TO_RAD)
             * std::sin(dlon * 0.5) * std::sin(dlon * 0.5);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return EARTH_RADIUS_M * c;
}

[[nodiscard]] inline double haversine_meters(const Point3D& a,
                                              const Point3D& b) noexcept {
    return haversine_meters(a.x, a.y, b.x, b.y);
}

/// ─── Azimuth (bearing) from a to b in degrees [0, 360) ─────────────────────
[[nodiscard]] inline double bearing_degrees(double lon1, double lat1,
                                             double lon2, double lat2) noexcept {
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double y = std::sin(dlon) * std::cos(lat2 * DEG_TO_RAD);
    double x = std::cos(lat1 * DEG_TO_RAD) * std::sin(lat2 * DEG_TO_RAD)
             - std::sin(lat1 * DEG_TO_RAD) * std::cos(lat2 * DEG_TO_RAD)
             * std::cos(dlon);
    double bearing = std::atan2(y, x) * RAD_TO_DEG;
    return std::fmod(bearing + 360.0, 360.0);
}

/// ─── WGS84 → HK1980 Grid approximate conversion ───────────────────────────
/// HK1980 (EPSG:2326) uses Transverse Mercator projection.
/// Hong Kong is roughly at 22.3°N 114.1°E.
/// This is a simplified conversion; for production use, link proj or GDAL.
[[nodiscard]] inline std::pair<double, double>
wgs84_to_hk1980_approx(double lon, double lat) noexcept {
    // HK1980 origin: 22°18'20"N, 114°10'20"E (approximate)
    // False Easting: 836,694.05m, False Northing: 819,069.80m
    // Scale factor at central meridian: 1.0

    constexpr double origin_lat = 22.3122;   // ~22°18'20"
    constexpr double origin_lon = 114.1783;  // ~114°10'42" (HK1980 central meridian)
    constexpr double false_easting = 836694.05;
    constexpr double false_northing = 819069.80;
    constexpr double scale = 1.0;

    // Simplified TM projection (works within ~100km of origin)
    double dlon_rad = (lon - origin_lon) * DEG_TO_RAD;
    double lat_rad = lat * DEG_TO_RAD;
    double origin_lat_rad = origin_lat * DEG_TO_RAD;

    double nu = EARTH_RADIUS_M / std::sqrt(1.0 - 0.00669438 * std::sin(lat_rad) * std::sin(lat_rad));
    double easting  = false_easting + scale * nu * dlon_rad * std::cos(lat_rad);
    double northing = false_northing + scale * EARTH_RADIUS_M * (lat_rad - origin_lat_rad);

    return {easting, northing};
}

[[nodiscard]] inline Point3D wgs84_to_hk1980_approx(const Point3D& p) noexcept {
    auto [e, n] = wgs84_to_hk1980_approx(p.x, p.y);
    return {e, n, p.z, CoordSystem::HK1980};
}

/// ─── 3D walking cost model ──────────────────────────────────────────────────
/// Combines horizontal distance with vertical penalty and step-count estimate.
struct WalkingCost {
    double distance_3d_m;       // 3D path length (meters)
    double ascent_m;            // total ascent (meters)
    double descent_m;           // total descent (meters)
    double cost;                // effective cost (equivalent flat meters)
    double estimated_seconds;   // estimated walking time

    /// Naismith's rule adapted for urban walking:
    /// - 5 km/h on flat (0.833 m/s → 1.2 s per meter)
    /// - +10 min per 100m ascent → +6 s per meter ascent
    /// - stairs 2x slower
    static WalkingCost compute(double dist_3d, double ascent, double descent,
                                bool has_stairs = false, bool has_lift = false) noexcept {
        WalkingCost wc{};
        wc.distance_3d_m = dist_3d;
        wc.ascent_m = ascent;
        wc.descent_m = descent;

        double flat_rate = 1.2;   // seconds per horizontal meter
        double climb_rate = 6.0;  // extra seconds per meter of ascent
        double stair_factor = has_stairs ? 2.0 : 1.0;
        double lift_flat_cost = has_lift ? 30.0 : 0.0; // 30s waiting+travel

        wc.estimated_seconds = flat_rate * dist_3d * stair_factor
                             + climb_rate * ascent * stair_factor
                             + lift_flat_cost;
        wc.cost = wc.estimated_seconds;  // cost ≡ seconds for routing

        return wc;
    }
};

} // namespace geodis
