#ifndef AVIATION_FORMULARY_NAVIGATION_RHUMB_LINE_HPP
#define AVIATION_FORMULARY_NAVIGATION_RHUMB_LINE_HPP

/**
 * @file rhumb_line.hpp
 * @brief Rhumb line (loxodrome) navigation calculations
 * 
 * Rhumb lines are tracks of constant true course. Unlike great circles,
 * they maintain a constant bearing but are not the shortest distance
 * between two points (except for meridians and the equator).
 * 
 * SIGN CONVENTION: Following Ed Williams' Aviation Formulary V1.47
 * - North latitudes are POSITIVE
 * - West longitudes are POSITIVE
 * - South latitudes are NEGATIVE
 * - East longitudes are NEGATIVE
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
// Rhumb Line Distance
// =============================================================================

/**
 * @brief Calculate distance along a rhumb line between two points
 * 
 * The rhumb line distance is generally longer than the great circle distance,
 * but the course is constant throughout.
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param p1 First point
 * @param p2 Second point
 * @return Distance in radians (use rad_to_nm() to convert to nautical miles)
 */
[[nodiscard]] inline double rhumb_distance(const LatLon& p1, const LatLon& p2) noexcept {
    double dlat = p2.lat - p1.lat;
    double dlon = p1.lon - p2.lon;  // Note: formulary convention
    
    // Calculate q (the "stretch factor")
    double q;
    
    // Handle case where latitudes are very close (avoid division by zero)
    if (std::abs(dlat) < constants::EPS) {
        q = std::cos(p1.lat);
    } else {
        // Standard formula using the Mercator projection
        double dphi = std::log(std::tan(p2.lat / 2.0 + constants::HALF_PI / 2.0) / 
                               std::tan(p1.lat / 2.0 + constants::HALF_PI / 2.0));
        q = dlat / dphi;
    }
    
    // Normalize longitude difference to [-PI, PI]
    dlon = normalize_angle_signed(dlon);
    
    // Calculate distance
    return std::sqrt(dlat * dlat + q * q * dlon * dlon);
}

/**
 * @brief Calculate rhumb line distance in nautical miles
 * 
 * Convenience function that returns distance directly in nautical miles.
 */
[[nodiscard]] inline double rhumb_distance_nm(const LatLon& p1, const LatLon& p2) noexcept {
    return rad_to_nm(rhumb_distance(p1, p2));
}

// =============================================================================
// Rhumb Line Course
// =============================================================================

/**
 * @brief Calculate constant course (bearing) for rhumb line between two points
 * 
 * Unlike great circle routes, the rhumb line course is constant from
 * start to finish.
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param p1 Starting point
 * @param p2 Destination point
 * @return Result containing course in radians [0, 2*PI), or error if undefined
 */
[[nodiscard]] inline Result<double> rhumb_course(const LatLon& p1, const LatLon& p2) noexcept {
    // Handle identical points
    if (std::abs(p1.lat - p2.lat) < constants::EPS && 
        std::abs(p1.lon - p2.lon) < constants::EPS) {
        return Result<double>::err(ErrorCode::IdenticalPoints, 0.0);
    }
    
    // Handle points at poles (rhumb line undefined)
    if (p1.is_pole() || p2.is_pole()) {
        // For meridional routes through poles, we can still define a course
        if (std::abs(p1.lon - p2.lon) < constants::EPS || 
            std::abs(std::abs(p1.lon - p2.lon) - constants::PI) < constants::EPS) {
            // Same or opposite meridian - course is due N or S
            if (p2.lat > p1.lat) {
                return Result<double>::ok(0.0);  // Due North
            } else {
                return Result<double>::ok(constants::PI);  // Due South
            }
        }
        return Result<double>::err(ErrorCode::PolePoint);
    }
    
    double dlon = p1.lon - p2.lon;  // Note: formulary convention
    
    // Normalize longitude difference
    dlon = normalize_angle_signed(dlon);
    
    // Calculate dphi (Mercator latitude difference)
    double dphi = std::log(std::tan(p2.lat / 2.0 + constants::HALF_PI / 2.0) / 
                           std::tan(p1.lat / 2.0 + constants::HALF_PI / 2.0));
    
    // Calculate course
    double tc = std::atan2(dlon, dphi);
    
    return Result<double>::ok(normalize_angle(tc));
}

/**
 * @brief Calculate rhumb line course (unchecked version)
 * 
 * Does not return error codes. Use when you know points are valid.
 */
[[nodiscard]] inline double rhumb_course_unchecked(const LatLon& p1, const LatLon& p2) noexcept {
    if (std::abs(p1.lat - p2.lat) < constants::EPS && 
        std::abs(p1.lon - p2.lon) < constants::EPS) {
        return 0.0;
    }
    
    double dlon = p1.lon - p2.lon;
    dlon = normalize_angle_signed(dlon);
    
    double dphi = std::log(std::tan(p2.lat / 2.0 + constants::HALF_PI / 2.0) / 
                           std::tan(p1.lat / 2.0 + constants::HALF_PI / 2.0));
    
    return normalize_angle(std::atan2(dlon, dphi));
}

// =============================================================================
// Rhumb Line Destination Point
// =============================================================================

/**
 * @brief Calculate destination point along a rhumb line
 * 
 * Given a starting point, constant course, and distance, calculate
 * the destination point.
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param start Starting point
 * @param course Course in radians
 * @param dist Distance in radians
 * @return Result containing destination point
 */
[[nodiscard]] inline Result<LatLon> rhumb_destination(const LatLon& start, 
                                                       double course, 
                                                       double dist) noexcept {
    // Handle zero distance
    if (dist < constants::EPS) {
        return Result<LatLon>::ok(start);
    }
    
    if (dist < 0) {
        return Result<LatLon>::err(ErrorCode::InvalidDistance);
    }
    
    // Normalize course
    course = normalize_angle(course);
    
    double dlat = dist * std::cos(course);
    double lat2 = start.lat + dlat;
    
    // Check if destination would be beyond a pole
    if (std::abs(lat2) > constants::HALF_PI) {
        return Result<LatLon>::err(ErrorCode::OutOfRange);
    }
    
    // Calculate q (stretch factor)
    double q;
    if (std::abs(dlat) < constants::EPS) {
        q = std::cos(start.lat);
    } else {
        double dphi = std::log(std::tan(lat2 / 2.0 + constants::HALF_PI / 2.0) / 
                               std::tan(start.lat / 2.0 + constants::HALF_PI / 2.0));
        q = dlat / dphi;
    }
    
    double dlon = dist * std::sin(course) / q;
    double lon2 = start.lon - dlon;  // Note: formulary convention
    
    // Normalize longitude
    lon2 = normalize_angle_signed(lon2);
    
    return Result<LatLon>::ok(LatLon{lat2, lon2});
}

/**
 * @brief Calculate rhumb line destination (unchecked version)
 */
[[nodiscard]] inline LatLon rhumb_destination_unchecked(const LatLon& start, 
                                                         double course, 
                                                         double dist) noexcept {
    if (dist < constants::EPS) {
        return start;
    }
    
    course = normalize_angle(course);
    
    double dlat = dist * std::cos(course);
    double lat2 = start.lat + dlat;
    
    // Clamp latitude if beyond poles
    lat2 = clamp(lat2, -constants::HALF_PI, constants::HALF_PI);
    
    double q;
    if (std::abs(dlat) < constants::EPS) {
        q = std::cos(start.lat);
    } else {
        double dphi = std::log(std::tan(lat2 / 2.0 + constants::HALF_PI / 2.0) / 
                               std::tan(start.lat / 2.0 + constants::HALF_PI / 2.0));
        q = dlat / dphi;
    }
    
    double dlon = (std::abs(q) > constants::EPS) ? (dist * std::sin(course) / q) : 0.0;
    double lon2 = normalize_angle_signed(start.lon - dlon);
    
    return LatLon{lat2, lon2};
}

// =============================================================================
// Rhumb Line Intermediate Point
// =============================================================================

/**
 * @brief Calculate intermediate point at fraction f along rhumb line from p1 to p2
 * 
 * @param p1 Starting point (f = 0)
 * @param p2 Ending point (f = 1)
 * @param f Fraction along route [0, 1]
 * @return Result containing intermediate point
 */
[[nodiscard]] inline Result<LatLon> rhumb_intermediate_point(const LatLon& p1, 
                                                              const LatLon& p2, 
                                                              double f) noexcept {
    // Validate fraction
    if (f < 0.0 || f > 1.0) {
        return Result<LatLon>::err(ErrorCode::InvalidFraction);
    }
    
    // Handle endpoints
    if (f < constants::EPS) {
        return Result<LatLon>::ok(p1);
    }
    if (f > 1.0 - constants::EPS) {
        return Result<LatLon>::ok(p2);
    }
    
    // Get course and total distance
    auto course_result = rhumb_course(p1, p2);
    if (!course_result) {
        return Result<LatLon>::err(course_result.error);
    }
    
    double total_dist = rhumb_distance(p1, p2);
    double partial_dist = f * total_dist;
    
    return rhumb_destination(p1, course_result.value, partial_dist);
}

// =============================================================================
// Rhumb Line Midpoint
// =============================================================================

/**
 * @brief Calculate midpoint along rhumb line between two points
 * 
 * Convenience function - equivalent to rhumb_intermediate_point(p1, p2, 0.5)
 */
[[nodiscard]] inline Result<LatLon> rhumb_midpoint(const LatLon& p1, const LatLon& p2) noexcept {
    return rhumb_intermediate_point(p1, p2, 0.5);
}

// =============================================================================
// Comparison Functions
// =============================================================================

/**
 * @brief Compare great circle and rhumb line distances
 * 
 * Returns the ratio of rhumb line distance to great circle distance.
 * Values > 1.0 indicate the rhumb line is longer (typical case).
 * Values close to 1.0 indicate routes are nearly equivalent
 * (e.g., along meridians or equator).
 * 
 * @param p1 First point
 * @param p2 Second point
 * @return Ratio of rhumb distance to great circle distance
 */
[[nodiscard]] inline double gc_rhumb_ratio(const LatLon& p1, const LatLon& p2) noexcept {
    double gc_dist = distance(p1, p2);  // Great circle distance
    double rl_dist = rhumb_distance(p1, p2);  // Rhumb line distance
    
    if (gc_dist < constants::EPS) {
        return 1.0;  // Identical points
    }
    
    return rl_dist / gc_dist;
}

/**
 * @brief Calculate the "extra" distance of rhumb line vs great circle
 * 
 * @param p1 First point
 * @param p2 Second point
 * @return Extra distance in radians (rhumb - great circle)
 */
[[nodiscard]] inline double rhumb_excess(const LatLon& p1, const LatLon& p2) noexcept {
    return rhumb_distance(p1, p2) - distance(p1, p2);
}

} // namespace aviation

#endif // AVIATION_FORMULARY_NAVIGATION_RHUMB_LINE_HPP
