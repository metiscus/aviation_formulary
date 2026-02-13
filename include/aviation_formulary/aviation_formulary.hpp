#ifndef AVIATION_FORMULARY_HPP
#define AVIATION_FORMULARY_HPP

/**
 * @file aviation_formulary.hpp
 * @brief Aviation Formulary - Complete C++ Implementation
 * 
 * This is the main umbrella header that includes all aviation calculation modules.
 * Based on Ed Williams' Aviation Formulary V1.47
 * https://edwilliams.org/avform.htm
 * 
 * SIGN CONVENTION: Following Ed Williams' Aviation Formulary V1.47
 * - North latitudes are POSITIVE
 * - West longitudes are POSITIVE
 * - South latitudes are NEGATIVE
 * - East longitudes are NEGATIVE
 * This differs from standard geographic convention (N/E positive)
 * 
 * @author Based on original formulary by Ed Williams
 * @version 2.0.0
 */

// =============================================================================
// Core Module - Foundation types, constants, and math utilities
// =============================================================================

#include "core/constants.hpp"     // Mathematical and physical constants
#include "core/types.hpp"         // Result<T>, ErrorCode, LatLon, Wind, etc.
#include "core/math.hpp"          // Safe trigonometry, unit conversions

// =============================================================================
// Navigation Module - Great circle, rhumb line, and local navigation
// =============================================================================

#include "navigation/great_circle.hpp"  // Great circle navigation
#include "navigation/rhumb_line.hpp"    // Rhumb line (constant course) navigation
#include "navigation/local_flat.hpp"    // Flat earth approximations

// =============================================================================
// Atmosphere Module - Standard atmosphere, altimetry, and humidity
// =============================================================================

#include "atmosphere/standard_atmosphere.hpp"  // ISA temperature, pressure, density
#include "atmosphere/altimetry.hpp"            // Pressure/density altitude, QNH/QFE
#include "atmosphere/humidity.hpp"             // Dewpoint, relative humidity, vapor

// =============================================================================
// Performance Module - Airspeeds, wind, and turns
// =============================================================================

#include "performance/airspeed.hpp"  // TAS/CAS/EAS/Mach conversions
#include "performance/wind.hpp"      // Wind triangles, head/crosswind components
#include "performance/turns.hpp"     // Turn radius, rate, bank angle, pivotal altitude

// =============================================================================
// Utilities Module - Conversions and miscellaneous functions
// =============================================================================

#include "utilities/conversions.hpp"    // Comprehensive unit conversions
#include "utilities/miscellaneous.hpp"  // Horizon, Bellamy, time-speed-distance

// =============================================================================
// Legacy Compatibility Layer
// =============================================================================

/**
 * @brief Legacy compatibility namespace
 * 
 * These functions provide backwards compatibility with the original API
 * that used exceptions. New code should use the non-exception Result<T>-based
 * functions directly from the aviation namespace.
 * 
 * Note: The new API does NOT use exceptions - functions return Result<T>.
 */
namespace aviation {
namespace legacy {

// Re-export commonly used types and functions at namespace level for convenience
using aviation::LatLon;
using aviation::deg_to_rad;
using aviation::rad_to_deg;
using aviation::nm_to_rad;
using aviation::rad_to_nm;
using aviation::asin_safe;
using aviation::acos_safe;
using aviation::normalize_angle;
using aviation::normalize_angle_signed;

/**
 * @brief Distance between two points (legacy wrapper, returns radians)
 * 
 * @deprecated Use aviation::distance() which is exception-free
 */
[[deprecated("Use aviation::distance() directly")]]
inline double gc_distance(const LatLon& p1, const LatLon& p2) {
    return aviation::distance(p1, p2);
}

/**
 * @brief Initial bearing from p1 to p2 (legacy wrapper)
 * 
 * @deprecated Use aviation::initial_bearing_unchecked() which is exception-free
 */
[[deprecated("Use aviation::initial_bearing_unchecked() directly")]]
inline double gc_initial_bearing(const LatLon& p1, const LatLon& p2) {
    return aviation::initial_bearing_unchecked(p1, p2);
}

/**
 * @brief Calculate destination point (legacy wrapper)
 * 
 * @deprecated Use aviation::destination_point_unchecked() which is exception-free
 */
[[deprecated("Use aviation::destination_point_unchecked() directly")]]
inline LatLon gc_destination_point(const LatLon& start, double bearing, double dist) {
    return aviation::destination_point_unchecked(start, bearing, dist);
}

} // namespace legacy
} // namespace aviation

#endif // AVIATION_FORMULARY_HPP
