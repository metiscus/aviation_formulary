#ifndef AVIATION_FORMULARY_NAVIGATION_LOCAL_FLAT_HPP
#define AVIATION_FORMULARY_NAVIGATION_LOCAL_FLAT_HPP

/**
 * @file local_flat.hpp
 * @brief Local flat earth approximations for short distances
 * 
 * For distances less than a few hundred miles, a flat earth approximation
 * can be used for faster calculations with minimal error. These approximations
 * treat the earth as locally flat with a simple Cartesian coordinate system.
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
// Flat Earth Distance
// =============================================================================

/**
 * @brief Calculate approximate distance using flat earth model
 * 
 * This approximation is accurate to about 0.1% for distances under 100 nm.
 * It becomes progressively less accurate for longer distances.
 * 
 * The approximation uses a simple Pythagorean formula with latitude
 * correction for longitude.
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param p1 First point
 * @param p2 Second point
 * @return Distance in radians (use rad_to_nm() to convert)
 */
[[nodiscard]] inline double flat_earth_distance(const LatLon& p1, const LatLon& p2) noexcept {
    double dlat = p1.lat - p2.lat;
    double dlon = p1.lon - p2.lon;
    
    // Use average latitude for the longitude correction
    double avg_lat = (p1.lat + p2.lat) / 2.0;
    double dlon_corrected = dlon * std::cos(avg_lat);
    
    return std::sqrt(dlat * dlat + dlon_corrected * dlon_corrected);
}

/**
 * @brief Calculate approximate distance in nautical miles using flat earth model
 */
[[nodiscard]] inline double flat_earth_distance_nm(const LatLon& p1, const LatLon& p2) noexcept {
    return rad_to_nm(flat_earth_distance(p1, p2));
}

// =============================================================================
// Flat Earth Bearing
// =============================================================================

/**
 * @brief Calculate approximate bearing using flat earth model
 * 
 * SIGN CONVENTION: North/West positive, South/East negative
 * 
 * @param p1 Starting point
 * @param p2 Destination point
 * @return Bearing in radians [0, 2*PI)
 */
[[nodiscard]] inline double flat_earth_bearing(const LatLon& p1, const LatLon& p2) noexcept {
    double dlat = p2.lat - p1.lat;
    double dlon = p1.lon - p2.lon;  // Note: formulary convention
    
    // Use average latitude for the longitude correction
    double avg_lat = (p1.lat + p2.lat) / 2.0;
    double dlon_corrected = dlon * std::cos(avg_lat);
    
    return normalize_angle(std::atan2(dlon_corrected, dlat));
}

// =============================================================================
// Flat Earth Destination
// =============================================================================

/**
 * @brief Calculate approximate destination using flat earth model
 * 
 * @param start Starting point
 * @param bearing Bearing in radians
 * @param dist Distance in radians
 * @return Approximate destination point
 */
[[nodiscard]] inline LatLon flat_earth_destination(const LatLon& start, 
                                                    double bearing, 
                                                    double dist) noexcept {
    if (dist < constants::EPS) {
        return start;
    }
    
    double dlat = dist * std::cos(bearing);
    double lat2 = start.lat + dlat;
    
    // Clamp latitude to valid range
    lat2 = clamp(lat2, -constants::HALF_PI, constants::HALF_PI);
    
    // Use average latitude for longitude calculation
    double avg_lat = (start.lat + lat2) / 2.0;
    double cos_avg_lat = std::cos(avg_lat);
    
    double dlon = (std::abs(cos_avg_lat) > constants::EPS) 
                  ? (dist * std::sin(bearing) / cos_avg_lat) 
                  : 0.0;
    double lon2 = normalize_angle_signed(start.lon - dlon);
    
    return LatLon{lat2, lon2};
}

// =============================================================================
// Local Cartesian Coordinates
// =============================================================================

/**
 * @brief Local Cartesian coordinates relative to a reference point
 * 
 * X is positive East, Y is positive North.
 * Note: This differs from the formulary sign convention for the coordinates
 * themselves, but uses the same lat/lon input convention.
 */
struct LocalCartesian {
    double x;   ///< X coordinate (positive East) in same units as distance
    double y;   ///< Y coordinate (positive North) in same units as distance
    
    constexpr LocalCartesian() noexcept : x(0.0), y(0.0) {}
    constexpr LocalCartesian(double x_, double y_) noexcept : x(x_), y(y_) {}
    
    /// Get distance from origin
    [[nodiscard]] double distance() const noexcept {
        return std::sqrt(x * x + y * y);
    }
    
    /// Get bearing from origin (radians, 0 = North, clockwise)
    [[nodiscard]] double bearing() const noexcept {
        return normalize_angle(std::atan2(x, y));
    }
};

/**
 * @brief Convert lat/lon to local Cartesian coordinates
 * 
 * Creates a local tangent plane coordinate system centered at the reference point.
 * 
 * @param reference Reference point (origin of local system)
 * @param point Point to convert
 * @return Local Cartesian coordinates in radians (multiply by EARTH_RADIUS_NM for nm)
 */
[[nodiscard]] inline LocalCartesian to_local_cartesian(const LatLon& reference, 
                                                        const LatLon& point) noexcept {
    double dlat = point.lat - reference.lat;
    double dlon = point.lon - reference.lon;
    
    // Y is north-south (positive north)
    double y = dlat;
    
    // X is east-west (positive east) with latitude correction
    // Note: We negate dlon because West is positive in our convention
    double x = -dlon * std::cos(reference.lat);
    
    return LocalCartesian{x, y};
}

/**
 * @brief Convert local Cartesian coordinates back to lat/lon
 * 
 * @param reference Reference point (origin of local system)
 * @param local Local Cartesian coordinates
 * @return Lat/lon point
 */
[[nodiscard]] inline LatLon from_local_cartesian(const LatLon& reference, 
                                                  const LocalCartesian& local) noexcept {
    double lat = reference.lat + local.y;
    
    // Clamp latitude
    lat = clamp(lat, -constants::HALF_PI, constants::HALF_PI);
    
    double cos_lat = std::cos(reference.lat);
    double dlon = (std::abs(cos_lat) > constants::EPS) ? (-local.x / cos_lat) : 0.0;
    double lon = normalize_angle_signed(reference.lon + dlon);
    
    return LatLon{lat, lon};
}

/**
 * @brief Convert local Cartesian coordinates to lat/lon in nautical miles
 * 
 * @param reference Reference point (origin of local system)
 * @param point Point to convert
 * @return Local Cartesian coordinates in nautical miles
 */
[[nodiscard]] inline LocalCartesian to_local_cartesian_nm(const LatLon& reference, 
                                                           const LatLon& point) noexcept {
    LocalCartesian local = to_local_cartesian(reference, point);
    return LocalCartesian{rad_to_nm(local.x), rad_to_nm(local.y)};
}

/**
 * @brief Convert local Cartesian coordinates (in nm) back to lat/lon
 * 
 * @param reference Reference point (origin of local system)
 * @param local_nm Local Cartesian coordinates in nautical miles
 * @return Lat/lon point
 */
[[nodiscard]] inline LatLon from_local_cartesian_nm(const LatLon& reference, 
                                                     const LocalCartesian& local_nm) noexcept {
    LocalCartesian local{nm_to_rad(local_nm.x), nm_to_rad(local_nm.y)};
    return from_local_cartesian(reference, local);
}

// =============================================================================
// Validity Checking
// =============================================================================

/**
 * @brief Check if flat earth approximation is valid for given distance
 * 
 * The approximation is generally accurate to within 0.1% for distances
 * under the threshold, which defaults to 100 nm.
 * 
 * @param p1 First point
 * @param p2 Second point
 * @param threshold_nm Maximum valid distance in nautical miles (default 100)
 * @return true if flat earth approximation is valid
 */
[[nodiscard]] inline bool flat_earth_valid(const LatLon& p1, 
                                            const LatLon& p2, 
                                            double threshold_nm = 100.0) noexcept {
    double dist_nm = flat_earth_distance_nm(p1, p2);
    return dist_nm < threshold_nm;
}

/**
 * @brief Estimate error of flat earth approximation
 * 
 * Returns the ratio of flat earth distance to great circle distance.
 * Values close to 1.0 indicate the approximation is accurate.
 * 
 * @param p1 First point
 * @param p2 Second point
 * @return Ratio of flat earth distance to great circle distance
 */
[[nodiscard]] inline double flat_earth_accuracy(const LatLon& p1, const LatLon& p2) noexcept {
    // Import great circle distance function
    double flat_dist = flat_earth_distance(p1, p2);
    
    // Calculate great circle distance directly here to avoid circular dependency
    double sin_dlat = std::sin((p1.lat - p2.lat) / 2.0);
    double sin_dlon = std::sin((p1.lon - p2.lon) / 2.0);
    double a = sin_dlat * sin_dlat + 
               std::cos(p1.lat) * std::cos(p2.lat) * sin_dlon * sin_dlon;
    a = clamp(a, 0.0, 1.0);
    double gc_dist = 2.0 * asin_safe(std::sqrt(a));
    
    if (gc_dist < constants::EPS) {
        return 1.0;  // Identical points
    }
    
    return flat_dist / gc_dist;
}

} // namespace aviation

#endif // AVIATION_FORMULARY_NAVIGATION_LOCAL_FLAT_HPP
