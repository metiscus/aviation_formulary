#ifndef AVIATION_FORMULARY_NAVIGATION_GREAT_CIRCLE_HPP
#define AVIATION_FORMULARY_NAVIGATION_GREAT_CIRCLE_HPP

/**
 * @file great_circle.hpp
 * @brief Great circle navigation calculations
 * 
 * SIGN CONVENTION: Following Ed Williams' Aviation Formulary V1.47
 * - North latitudes are POSITIVE
 * - West longitudes are POSITIVE
 * - South latitudes are NEGATIVE
 * - East longitudes are NEGATIVE
 * 
 * This differs from standard geographic convention (N/E positive).
 * 
 * Based on Ed Williams' Aviation Formulary V1.47
 * https://edwilliams.org/avform.htm
 */

#include "../core/constants.hpp"
#include "../core/types.hpp"
#include "../core/math.hpp"
#include <cmath>

namespace aviation {

// =============================================================================
// Distance Calculations
// =============================================================================

/**
 * @brief Calculate great circle distance between two points using haversine formula
 * 
 * The haversine formula is more numerically stable for short distances
 * than the spherical law of cosines.
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param p1 First point
 * @param p2 Second point
 * @return Distance in radians (use rad_to_nm() to convert to nautical miles)
 */
[[nodiscard]] inline double distance(const LatLon& p1, const LatLon& p2) noexcept {
    // Early return for identical points
    if (std::abs(p1.lat - p2.lat) < constants::EPS && 
        std::abs(p1.lon - p2.lon) < constants::EPS) {
        return 0.0;
    }
    
    double sin_dlat = std::sin((p1.lat - p2.lat) / 2.0);
    double sin_dlon = std::sin((p1.lon - p2.lon) / 2.0);
    double a = sin_dlat * sin_dlat + 
               std::cos(p1.lat) * std::cos(p2.lat) * sin_dlon * sin_dlon;
    
    // Clamp a to [0, 1] to avoid sqrt of negative numbers from rounding
    a = clamp(a, 0.0, 1.0);
    
    return 2.0 * asin_safe(std::sqrt(a));
}

/**
 * @brief Calculate great circle distance using spherical law of cosines
 * 
 * Alternative method - less stable for short distances but sometimes
 * more convenient for verification.
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param p1 First point
 * @param p2 Second point
 * @return Distance in radians
 */
[[nodiscard]] inline double distance_cosine(const LatLon& p1, const LatLon& p2) noexcept {
    return acos_safe(std::sin(p1.lat) * std::sin(p2.lat) + 
                     std::cos(p1.lat) * std::cos(p2.lat) * 
                     std::cos(p1.lon - p2.lon));
}

/**
 * @brief Check if two points are identical (within epsilon)
 */
[[nodiscard]] inline bool points_identical(const LatLon& p1, const LatLon& p2) noexcept {
    return std::abs(p1.lat - p2.lat) < constants::EPS && 
           std::abs(p1.lon - p2.lon) < constants::EPS;
}

/**
 * @brief Check if two points are antipodal (opposite sides of earth)
 */
[[nodiscard]] inline bool points_antipodal(const LatLon& p1, const LatLon& p2) noexcept {
    double d = distance(p1, p2);
    return std::abs(d - constants::PI) < constants::EPS;
}

// =============================================================================
// Bearing/Course Calculations
// =============================================================================

/**
 * @brief Calculate initial bearing (true course) from p1 to p2
 * 
 * Returns the bearing at the starting point. For a great circle route,
 * the bearing changes continuously along the path.
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param p1 Starting point
 * @param p2 Destination point
 * @return Result containing bearing in radians [0, 2*PI), or error if undefined
 */
[[nodiscard]] inline Result<double> initial_bearing(const LatLon& p1, const LatLon& p2) noexcept {
    // Handle identical points
    if (points_identical(p1, p2)) {
        return Result<double>::err(ErrorCode::IdenticalPoints, 0.0);
    }
    
    // Special case: starting from a pole
    if (p1.is_pole()) {
        if (p1.lat > 0) {
            return Result<double>::ok(constants::PI);  // From North pole, go south
        } else {
            return Result<double>::ok(0.0);  // From South pole, go north
        }
    }
    
    // General case using atan2 (numerically stable)
    double dlon = p1.lon - p2.lon;
    double cos_p2_lat = std::cos(p2.lat);
    double sin_p2_lat = std::sin(p2.lat);
    double cos_p1_lat = std::cos(p1.lat);
    double sin_p1_lat = std::sin(p1.lat);
    
    double y = std::sin(dlon) * cos_p2_lat;
    double x = cos_p1_lat * sin_p2_lat - sin_p1_lat * cos_p2_lat * std::cos(dlon);
    
    return Result<double>::ok(normalize_angle(std::atan2(y, x)));
}

/**
 * @brief Calculate initial bearing (unchecked version)
 * 
 * Does not return error codes - returns 0 for identical points.
 * Use when you know points are valid and different.
 */
[[nodiscard]] inline double initial_bearing_unchecked(const LatLon& p1, const LatLon& p2) noexcept {
    if (points_identical(p1, p2)) {
        return 0.0;
    }
    
    if (p1.is_pole()) {
        return p1.lat > 0 ? constants::PI : 0.0;
    }
    
    double dlon = p1.lon - p2.lon;
    double y = std::sin(dlon) * std::cos(p2.lat);
    double x = std::cos(p1.lat) * std::sin(p2.lat) - 
               std::sin(p1.lat) * std::cos(p2.lat) * std::cos(dlon);
    
    return normalize_angle(std::atan2(y, x));
}

// =============================================================================
// Destination Point Calculations
// =============================================================================

/**
 * @brief Calculate destination point given start, bearing, and distance
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param start Starting point
 * @param bearing Initial bearing in radians
 * @param dist Distance in radians (use nm_to_rad() to convert from nm)
 * @return Result containing destination point
 */
[[nodiscard]] inline Result<LatLon> destination_point(const LatLon& start, 
                                                       double bearing, 
                                                       double dist) noexcept {
    // Handle zero or negative distance
    if (dist < constants::EPS) {
        return Result<LatLon>::ok(start);
    }
    
    if (dist < 0) {
        return Result<LatLon>::err(ErrorCode::InvalidDistance);
    }
    
    // Normalize bearing to [0, 2π)
    bearing = normalize_angle(bearing);
    
    double sin_dist = std::sin(dist);
    double cos_dist = std::cos(dist);
    double sin_lat = std::sin(start.lat);
    double cos_lat = std::cos(start.lat);
    double sin_brng = std::sin(bearing);
    double cos_brng = std::cos(bearing);
    
    // Calculate destination latitude
    double lat = asin_safe(sin_lat * cos_dist + cos_lat * sin_dist * cos_brng);
    
    // Check if destination is a pole
    if (std::abs(std::cos(lat)) < constants::EPS) {
        return Result<LatLon>::ok(LatLon{lat, start.lon});
    }
    
    // Calculate destination longitude
    double dlon = std::atan2(sin_brng * sin_dist * cos_lat,
                             cos_dist - sin_lat * std::sin(lat));
    double lon = normalize_angle_signed(start.lon - dlon + constants::PI) - constants::PI;
    
    return Result<LatLon>::ok(LatLon{lat, lon});
}

/**
 * @brief Calculate destination point (unchecked version)
 * 
 * Does not validate inputs. Use when you know values are valid.
 */
[[nodiscard]] inline LatLon destination_point_unchecked(const LatLon& start, 
                                                         double bearing, 
                                                         double dist) noexcept {
    if (dist < constants::EPS) {
        return start;
    }
    
    bearing = normalize_angle(bearing);
    
    double sin_dist = std::sin(dist);
    double cos_dist = std::cos(dist);
    double sin_lat = std::sin(start.lat);
    double cos_lat = std::cos(start.lat);
    double sin_brng = std::sin(bearing);
    double cos_brng = std::cos(bearing);
    
    double lat = asin_safe(sin_lat * cos_dist + cos_lat * sin_dist * cos_brng);
    
    if (std::abs(std::cos(lat)) < constants::EPS) {
        return LatLon{lat, start.lon};
    }
    
    double dlon = std::atan2(sin_brng * sin_dist * cos_lat,
                             cos_dist - sin_lat * std::sin(lat));
    double lon = normalize_angle_signed(start.lon - dlon + constants::PI) - constants::PI;
    
    return LatLon{lat, lon};
}

// =============================================================================
// Intermediate Point Calculations
// =============================================================================

/**
 * @brief Calculate intermediate point at fraction f along great circle from p1 to p2
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param p1 Starting point (f = 0)
 * @param p2 Ending point (f = 1)
 * @param f Fraction along route [0, 1]
 * @return Result containing intermediate point, or error if points are antipodal
 */
[[nodiscard]] inline Result<LatLon> intermediate_point(const LatLon& p1, 
                                                        const LatLon& p2, 
                                                        double f) noexcept {
    // Validate fraction
    if (f < 0.0 || f > 1.0) {
        return Result<LatLon>::err(ErrorCode::InvalidFraction);
    }
    
    double d = distance(p1, p2);
    
    // Handle identical points
    if (d < constants::EPS) {
        return Result<LatLon>::ok(p1);
    }
    
    // Handle antipodal points (infinite great circles)
    if (std::abs(d - constants::PI) < constants::EPS) {
        return Result<LatLon>::err(ErrorCode::AntipodalPoints);
    }
    
    double sin_d = std::sin(d);
    double a = std::sin((1.0 - f) * d) / sin_d;
    double b = std::sin(f * d) / sin_d;
    
    double cos_lat1 = std::cos(p1.lat);
    double cos_lat2 = std::cos(p2.lat);
    double sin_lat1 = std::sin(p1.lat);
    double sin_lat2 = std::sin(p2.lat);
    double cos_lon1 = std::cos(p1.lon);
    double cos_lon2 = std::cos(p2.lon);
    double sin_lon1 = std::sin(p1.lon);
    double sin_lon2 = std::sin(p2.lon);
    
    double x = a * cos_lat1 * cos_lon1 + b * cos_lat2 * cos_lon2;
    double y = a * cos_lat1 * sin_lon1 + b * cos_lat2 * sin_lon2;
    double z = a * sin_lat1 + b * sin_lat2;
    
    double lat = std::atan2(z, std::sqrt(x * x + y * y));
    double lon = std::atan2(y, x);
    
    return Result<LatLon>::ok(LatLon{lat, lon});
}

// =============================================================================
// Cross-Track and Along-Track Distance
// =============================================================================

/**
 * @brief Calculate cross-track distance (perpendicular distance from great circle)
 * 
 * Positive value means point is to the right of the course,
 * negative means to the left.
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param a Start point of great circle
 * @param b End point of great circle
 * @param d Point to check
 * @return Distance in radians (positive = right of course, negative = left)
 */
[[nodiscard]] inline double cross_track_distance(const LatLon& a, 
                                                  const LatLon& b, 
                                                  const LatLon& d) noexcept {
    double dist_ad = distance(a, d);
    
    // If d is at point a, XTD is zero
    if (dist_ad < constants::EPS) {
        return 0.0;
    }
    
    double crs_ab = initial_bearing_unchecked(a, b);
    double crs_ad = initial_bearing_unchecked(a, d);
    
    // Special case for poles
    if (a.is_pole()) {
        double angle_diff = a.lat > 0 ? (d.lon - b.lon) : (b.lon - d.lon);
        return asin_safe(std::sin(dist_ad) * std::sin(angle_diff));
    }
    
    // General case
    double angle_diff = crs_ad - crs_ab;
    return asin_safe(std::sin(dist_ad) * std::sin(angle_diff));
}

/**
 * @brief Calculate along-track distance from a towards b to point abeam d
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param a Start point of great circle
 * @param b End point of great circle
 * @param d Point to check
 * @return Distance in radians along track from a to the point abeam d
 */
[[nodiscard]] inline double along_track_distance(const LatLon& a, 
                                                  const LatLon& b, 
                                                  const LatLon& d) noexcept {
    double dist_ad = distance(a, d);
    double xtd = cross_track_distance(a, b, d);
    
    // More stable formula for short distances (about 600 nm)
    constexpr double SHORT_DISTANCE_THRESHOLD = 0.1;
    
    if (dist_ad < SHORT_DISTANCE_THRESHOLD) {
        double sin_dist = std::sin(dist_ad);
        double sin_xtd = std::sin(xtd);
        double cos_xtd = std::cos(xtd);
        if (std::abs(cos_xtd) < constants::EPS) {
            return 0.0;  // Point is perpendicular to track at start
        }
        return asin_safe(sqrt_safe(sin_dist * sin_dist - sin_xtd * sin_xtd) / cos_xtd);
    }
    
    // Standard formula for longer distances
    double cos_xtd = std::cos(xtd);
    if (std::abs(cos_xtd) < constants::EPS) {
        return 0.0;
    }
    return acos_safe(std::cos(dist_ad) / cos_xtd);
}

// =============================================================================
// Maximum Latitude (Clairaut's Formula)
// =============================================================================

/**
 * @brief Calculate maximum latitude reached on a great circle route
 * 
 * Uses Clairaut's formula: sin(tc) * cos(lat) = constant along great circle
 * 
 * @param point A point on the great circle
 * @param bearing Bearing at that point
 * @return Maximum latitude in radians
 */
[[nodiscard]] inline double max_latitude(const LatLon& point, double bearing) noexcept {
    return acos_safe(std::abs(std::sin(bearing) * std::cos(point.lat)));
}

// =============================================================================
// Intersection of Two Radials
// =============================================================================

/**
 * @brief Find intersection of two great circle radials
 * 
 * Given two points and bearings from each, find where the radials intersect.
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param p1 First point
 * @param brng1 Bearing from first point (radians)
 * @param p2 Second point
 * @param brng2 Bearing from second point (radians)
 * @return IntersectionResult with point or error status
 */
[[nodiscard]] inline IntersectionResult intersection(const LatLon& p1, double brng1,
                                                      const LatLon& p2, double brng2) noexcept {
    IntersectionResult result;
    
    double dist12 = distance(p1, p2);
    
    // Check for identical points
    if (dist12 < constants::EPS) {
        result.error = ErrorCode::IdenticalPoints;
        return result;
    }
    
    // Calculate initial bearings between the two points
    double crs12, crs21;
    double dlon = p2.lon - p1.lon;
    double sin_dist = std::sin(dist12);
    double cos_dist = std::cos(dist12);
    
    if (std::abs(sin_dist) < constants::EPS) {
        result.error = ErrorCode::IdenticalPoints;
        return result;
    }
    
    double cos_p1_lat = std::cos(p1.lat);
    double cos_p2_lat = std::cos(p2.lat);
    double sin_p1_lat = std::sin(p1.lat);
    double sin_p2_lat = std::sin(p2.lat);
    
    if (std::sin(dlon) < 0) {
        crs12 = acos_safe((sin_p2_lat - sin_p1_lat * cos_dist) / (sin_dist * cos_p1_lat));
        crs21 = constants::TWO_PI - 
                acos_safe((sin_p1_lat - sin_p2_lat * cos_dist) / (sin_dist * cos_p2_lat));
    } else {
        crs12 = constants::TWO_PI - 
                acos_safe((sin_p2_lat - sin_p1_lat * cos_dist) / (sin_dist * cos_p1_lat));
        crs21 = acos_safe((sin_p1_lat - sin_p2_lat * cos_dist) / (sin_dist * cos_p2_lat));
    }
    
    double ang1 = normalize_angle_signed(brng1 - crs12 + constants::PI) - constants::PI;
    double ang2 = normalize_angle_signed(crs21 - brng2 + constants::PI) - constants::PI;
    
    // Check for parallel/coincident paths
    if (std::abs(std::sin(ang1)) < constants::EPS && 
        std::abs(std::sin(ang2)) < constants::EPS) {
        result.error = ErrorCode::CoincidentPaths;
        return result;
    }
    
    // Check for ambiguous intersection
    if (std::sin(ang1) * std::sin(ang2) < 0) {
        result.error = ErrorCode::AmbiguousSolution;
        result.ambiguous = true;
        return result;
    }
    
    ang1 = std::abs(ang1);
    ang2 = std::abs(ang2);
    
    double ang3 = acos_safe(-std::cos(ang1) * std::cos(ang2) + 
                            std::sin(ang1) * std::sin(ang2) * cos_dist);
    double dist13 = std::atan2(sin_dist * std::sin(ang1) * std::sin(ang2),
                               std::cos(ang2) + std::cos(ang1) * std::cos(ang3));
    
    auto dest_result = destination_point(p1, brng1, dist13);
    if (dest_result.is_ok()) {
        result.point = dest_result.value;
        result.exists = true;
        result.error = ErrorCode::Success;
    } else {
        result.error = dest_result.error;
    }
    
    return result;
}

// =============================================================================
// Great Circle Crossing Parallel
// =============================================================================

/**
 * @brief Find where great circle from p1 to p2 crosses a parallel of latitude
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param p1 First point on great circle
 * @param p2 Second point on great circle
 * @param lat3 Latitude of parallel to cross (radians)
 * @return ParallelCrossingResult with crossing longitude(s)
 */
[[nodiscard]] inline ParallelCrossingResult crossing_parallels(const LatLon& p1, 
                                                                const LatLon& p2,
                                                                double lat3) noexcept {
    ParallelCrossingResult result;
    
    double l12 = p1.lon - p2.lon;
    double sin_lat3 = std::sin(lat3);
    double cos_lat3 = std::cos(lat3);
    double sin_lat1 = std::sin(p1.lat);
    double cos_lat1 = std::cos(p1.lat);
    double sin_lat2 = std::sin(p2.lat);
    double cos_lat2 = std::cos(p2.lat);
    double sin_l12 = std::sin(l12);
    double cos_l12 = std::cos(l12);
    
    double A = sin_lat1 * cos_lat2 * cos_lat3 * sin_l12;
    double B = sin_lat1 * cos_lat2 * cos_lat3 * cos_l12 - cos_lat1 * sin_lat2 * cos_lat3;
    double C = cos_lat1 * cos_lat2 * sin_lat3 * sin_l12;
    
    double lon = std::atan2(B, A);
    double r = std::sqrt(A * A + B * B);
    
    // Check if no crossing exists
    if (std::abs(C) > r) {
        result.error = ErrorCode::NoSolution;
        return result;
    }
    
    double dlon = acos_safe(C / r);
    
    result.error = ErrorCode::Success;
    result.lon1 = normalize_angle_signed(p1.lon + dlon + lon);
    result.lon2 = normalize_angle_signed(p1.lon - dlon + lon);
    
    // Determine number of distinct crossings
    if (std::abs(dlon) < constants::EPS) {
        result.num_crossings = 1;
    } else {
        result.num_crossings = 2;
    }
    
    return result;
}

// =============================================================================
// Points at Known Distance from Great Circle
// =============================================================================

/**
 * @brief Find point on great circle at given along-track distance from start
 * 
 * Given a great circle defined by two points, find the point that is
 * a specified distance along the track from the first point.
 * 
 * @param a Start point of great circle
 * @param b End point of great circle (defines direction)
 * @param atd Along-track distance in radians
 * @return Result containing point on great circle
 */
[[nodiscard]] inline Result<LatLon> point_at_distance(const LatLon& a, 
                                                       const LatLon& b,
                                                       double atd) noexcept {
    auto bearing_result = initial_bearing(a, b);
    if (!bearing_result) {
        return Result<LatLon>::err(bearing_result.error);
    }
    
    return destination_point(a, bearing_result.value, atd);
}

} // namespace aviation

#endif // AVIATION_FORMULARY_NAVIGATION_GREAT_CIRCLE_HPP
