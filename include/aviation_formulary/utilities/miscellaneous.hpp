#ifndef AVIATION_FORMULARY_UTILITIES_MISCELLANEOUS_HPP
#define AVIATION_FORMULARY_UTILITIES_MISCELLANEOUS_HPP

/**
 * @file miscellaneous.hpp
 * @brief Miscellaneous aviation utilities
 * 
 * Implements calculations for:
 * - Distance to horizon
 * - Bellamy's drift formula
 * - Other miscellaneous functions
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
// Distance to Horizon
// =============================================================================

/**
 * @brief Calculate distance to the visible horizon
 * 
 * From the formulary:
 * d = sqrt((2 * R * h) + h²)
 * 
 * For small h compared to R, this simplifies to:
 * d ≈ sqrt(2 * R * h) = 1.06 * sqrt(h) (in nm with h in feet)
 * 
 * @param height_ft Observer height above the surface in feet
 * @return Distance to horizon in nautical miles
 */
[[nodiscard]] inline double horizon_distance_nm(double height_ft) noexcept {
    if (height_ft <= 0) {
        return 0.0;
    }
    
    // Simple approximation: d = 1.06 * sqrt(h) (nm)
    // More accurate: d = sqrt(2 * R * h) where R = Earth radius
    // Using refraction-corrected coefficient of 1.17 (for standard atmosphere)
    
    // The 1.17 factor accounts for atmospheric refraction which bends light
    // and allows you to see slightly "around" the curve of the earth
    return 1.17 * std::sqrt(height_ft);
}

/**
 * @brief Calculate geometric (no refraction) distance to horizon
 * 
 * @param height_ft Observer height above the surface in feet
 * @return Geometric distance to horizon in nautical miles
 */
[[nodiscard]] inline double geometric_horizon_distance_nm(double height_ft) noexcept {
    if (height_ft <= 0) {
        return 0.0;
    }
    
    // Without refraction correction: d = 1.06 * sqrt(h)
    return 1.06 * std::sqrt(height_ft);
}

/**
 * @brief Calculate distance to horizon in kilometers
 * 
 * @param height_m Observer height above the surface in meters
 * @return Distance to horizon in kilometers
 */
[[nodiscard]] inline double horizon_distance_km(double height_m) noexcept {
    if (height_m <= 0) {
        return 0.0;
    }
    
    // d = 3.57 * sqrt(h) (km with h in meters)
    // With refraction: d = 3.86 * sqrt(h)
    return 3.86 * std::sqrt(height_m);
}

/**
 * @brief Calculate radar horizon distance
 * 
 * Radar travels in a straighter path than light, so uses different coefficient.
 * 
 * @param height_ft Antenna height in feet
 * @return Radar horizon distance in nautical miles
 */
[[nodiscard]] inline double radar_horizon_nm(double height_ft) noexcept {
    if (height_ft <= 0) {
        return 0.0;
    }
    
    // Radar horizon uses 1.23 coefficient
    return 1.23 * std::sqrt(height_ft);
}

/**
 * @brief Calculate line-of-sight distance between two points at different heights
 * 
 * Maximum communication/radar range between two stations.
 * 
 * @param height1_ft Height of first station in feet
 * @param height2_ft Height of second station in feet
 * @return Maximum line-of-sight distance in nautical miles
 */
[[nodiscard]] inline double line_of_sight_distance_nm(double height1_ft, 
                                                       double height2_ft) noexcept {
    return horizon_distance_nm(height1_ft) + horizon_distance_nm(height2_ft);
}

/**
 * @brief Calculate height required to see a target at a given distance
 * 
 * @param distance_nm Distance to target in nautical miles
 * @return Required observer height in feet
 */
[[nodiscard]] inline double height_for_horizon(double distance_nm) noexcept {
    if (distance_nm <= 0) {
        return 0.0;
    }
    
    // Inverse of d = 1.17 * sqrt(h)
    // h = (d / 1.17)²
    double ratio = distance_nm / 1.17;
    return ratio * ratio;
}

// =============================================================================
// Bellamy's Drift Formula
// =============================================================================

/**
 * @brief Calculate wind drift using Bellamy's formula
 * 
 * Bellamy's formula estimates the drift angle caused by wind from
 * the difference in heading required to maintain track between
 * outbound and return legs.
 * 
 * From the formulary:
 * drift = (heading_out - heading_back - 180°) / 2
 * 
 * @param heading_out_rad Heading on outbound leg (radians)
 * @param heading_back_rad Heading on return leg (radians)
 * @return Drift angle in radians (positive = drift to right)
 */
[[nodiscard]] inline double bellamy_drift(double heading_out_rad, 
                                          double heading_back_rad) noexcept {
    // Normalize the headings
    double diff = normalize_radians(heading_out_rad - heading_back_rad - constants::PI);
    return diff / 2.0;
}

/**
 * @brief Calculate wind correction angle using Bellamy's method
 * 
 * @param heading_out_deg Heading on outbound leg (degrees)
 * @param heading_back_deg Heading on return leg (degrees)
 * @return Wind correction angle in degrees
 */
[[nodiscard]] inline double bellamy_wca_deg(double heading_out_deg, 
                                            double heading_back_deg) noexcept {
    double drift = bellamy_drift(deg_to_rad(heading_out_deg), 
                                 deg_to_rad(heading_back_deg));
    return rad_to_deg(drift);
}

// =============================================================================
// Rule of Three Navigation
// =============================================================================

/**
 * @brief Calculate a 3° descent path distance
 * 
 * The "divide by 3" rule for a 3° descent path:
 * Distance (nm) = Altitude to lose (thousands of feet) × 3
 * 
 * @param altitude_to_lose_ft Altitude to lose in feet
 * @return Distance required in nautical miles
 */
[[nodiscard]] constexpr double three_degree_descent_distance(double altitude_to_lose_ft) noexcept {
    return (altitude_to_lose_ft / 1000.0) * 3.0;
}

/**
 * @brief Calculate descent rate for a given descent angle and ground speed
 * 
 * @param angle_deg Descent angle in degrees
 * @param ground_speed_kt Ground speed in knots
 * @return Required descent rate in feet per minute
 */
[[nodiscard]] inline double descent_rate_fpm(double angle_deg, double ground_speed_kt) noexcept {
    // rate = GS * tan(angle) * (6076/60)
    // For small angles: rate ≈ GS * angle(deg) * 100 / 60
    // More precisely: rate = GS * tan(angle) * 101.27
    
    double angle_rad = deg_to_rad(angle_deg);
    return ground_speed_kt * std::tan(angle_rad) * 101.27;
}

/**
 * @brief Calculate descent angle for a given rate and ground speed
 * 
 * @param rate_fpm Descent rate in feet per minute
 * @param ground_speed_kt Ground speed in knots
 * @return Descent angle in degrees
 */
[[nodiscard]] inline double descent_angle_deg(double rate_fpm, double ground_speed_kt) noexcept {
    if (ground_speed_kt <= constants::EPS) {
        return 0.0;
    }
    
    // angle = atan(rate / (GS * 101.27))
    double tan_angle = rate_fpm / (ground_speed_kt * 101.27);
    return rad_to_deg(std::atan(tan_angle));
}

// =============================================================================
// Time-Speed-Distance
// =============================================================================

/**
 * @brief Calculate time to travel a distance at given speed
 * 
 * @param distance_nm Distance in nautical miles
 * @param speed_kt Speed in knots
 * @return Time in hours
 */
[[nodiscard]] inline double time_to_travel(double distance_nm, double speed_kt) noexcept {
    if (speed_kt <= constants::EPS) {
        return 0.0;
    }
    return distance_nm / speed_kt;
}

/**
 * @brief Calculate time to travel a distance at given speed (in minutes)
 * 
 * @param distance_nm Distance in nautical miles
 * @param speed_kt Speed in knots
 * @return Time in minutes
 */
[[nodiscard]] inline double time_to_travel_min(double distance_nm, double speed_kt) noexcept {
    return time_to_travel(distance_nm, speed_kt) * 60.0;
}

/**
 * @brief Calculate distance traveled in given time at given speed
 * 
 * @param time_hr Time in hours
 * @param speed_kt Speed in knots
 * @return Distance in nautical miles
 */
[[nodiscard]] constexpr double distance_traveled(double time_hr, double speed_kt) noexcept {
    return time_hr * speed_kt;
}

/**
 * @brief Calculate required speed for time-to-go
 * 
 * @param distance_nm Distance remaining in nautical miles
 * @param time_hr Time remaining in hours
 * @return Required speed in knots
 */
[[nodiscard]] inline double required_speed(double distance_nm, double time_hr) noexcept {
    if (time_hr <= constants::EPS) {
        return 0.0;
    }
    return distance_nm / time_hr;
}

// =============================================================================
// Fuel Calculations
// =============================================================================

/**
 * @brief Calculate fuel required for a leg
 * 
 * @param time_hr Flight time in hours
 * @param fuel_flow Fuel flow rate (in any consistent units)
 * @return Fuel required (same units as fuel_flow)
 */
[[nodiscard]] constexpr double fuel_required(double time_hr, double fuel_flow) noexcept {
    return time_hr * fuel_flow;
}

/**
 * @brief Calculate endurance from fuel quantity and flow rate
 * 
 * @param fuel_qty Fuel quantity (any units)
 * @param fuel_flow Fuel flow rate (same units per hour)
 * @return Endurance in hours
 */
[[nodiscard]] inline double endurance(double fuel_qty, double fuel_flow) noexcept {
    if (fuel_flow <= constants::EPS) {
        return 0.0;
    }
    return fuel_qty / fuel_flow;
}

/**
 * @brief Calculate range from fuel quantity, flow rate, and speed
 * 
 * @param fuel_qty Fuel quantity (any units)
 * @param fuel_flow Fuel flow rate (same units per hour)
 * @param speed_kt Ground speed in knots
 * @return Range in nautical miles
 */
[[nodiscard]] inline double range_nm(double fuel_qty, double fuel_flow, double speed_kt) noexcept {
    double endurance_hr = endurance(fuel_qty, fuel_flow);
    return endurance_hr * speed_kt;
}

// =============================================================================
// Maneuvering Speed and Load Factor
// =============================================================================

/**
 * @brief Calculate maneuvering speed for a given weight
 * 
 * Va = Va_design * sqrt(actual_weight / max_gross_weight)
 * 
 * @param va_design Design maneuvering speed at max gross weight (knots)
 * @param actual_weight Actual weight (any units)
 * @param max_weight Maximum gross weight (same units)
 * @return Adjusted maneuvering speed in knots
 */
[[nodiscard]] inline double maneuvering_speed(double va_design, 
                                               double actual_weight, 
                                               double max_weight) noexcept {
    if (max_weight <= constants::EPS || actual_weight <= 0) {
        return 0.0;
    }
    return va_design * std::sqrt(actual_weight / max_weight);
}

// =============================================================================
// Pattern Calculations
// =============================================================================

/**
 * @brief Calculate pattern altitude AGL
 * 
 * @param field_elevation_ft Field elevation MSL in feet
 * @param pattern_altitude_msl_ft Pattern altitude MSL in feet
 * @return Pattern altitude AGL in feet
 */
[[nodiscard]] constexpr double pattern_altitude_agl(double field_elevation_ft, 
                                                     double pattern_altitude_msl_ft) noexcept {
    return pattern_altitude_msl_ft - field_elevation_ft;
}

/**
 * @brief Calculate crosswind leg timing
 * 
 * For a standard traffic pattern, calculates when to turn to crosswind
 * based on climbing at a consistent rate to pattern altitude.
 * 
 * @param climb_rate_fpm Climb rate in feet per minute
 * @param pattern_altitude_agl Pattern altitude above field in feet
 * @return Time to pattern altitude in seconds
 */
[[nodiscard]] inline double time_to_pattern_altitude(double climb_rate_fpm, 
                                                      double pattern_altitude_agl) noexcept {
    if (climb_rate_fpm <= constants::EPS) {
        return 0.0;
    }
    return (pattern_altitude_agl / climb_rate_fpm) * 60.0;
}

} // namespace aviation

#endif // AVIATION_FORMULARY_UTILITIES_MISCELLANEOUS_HPP
