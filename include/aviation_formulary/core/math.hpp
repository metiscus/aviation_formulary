#ifndef AVIATION_FORMULARY_CORE_MATH_HPP
#define AVIATION_FORMULARY_CORE_MATH_HPP

/**
 * @file math.hpp
 * @brief Mathematical utilities for aviation calculations
 * 
 * Includes safe trigonometric functions, angle normalization,
 * and unit conversion utilities.
 * 
 * Based on Ed Williams' Aviation Formulary V1.47
 * https://edwilliams.org/avform.htm
 */

#include "constants.hpp"
#include <cmath>
#include <algorithm>

namespace aviation {

// =============================================================================
// Modulo and Angle Normalization
// =============================================================================

/**
 * @brief Euclidean modulo function that returns positive results
 * 
 * Result is in range [0, m) for positive m.
 * This differs from std::fmod which can return negative values.
 * 
 * @param y Dividend
 * @param m Divisor (must be positive)
 * @return Result in range [0, m)
 */
[[nodiscard]] inline double mod(double y, double m) noexcept {
    double result = std::fmod(y, m);
    if (result < 0.0) {
        result += m;
    }
    return result;
}

/**
 * @brief Normalize angle to range [0, 2*PI)
 * 
 * @param angle Angle in radians
 * @return Normalized angle in [0, 2*PI)
 */
[[nodiscard]] inline double normalize_angle(double angle) noexcept {
    return mod(angle, constants::TWO_PI);
}

/**
 * @brief Normalize angle to range [-PI, PI)
 * 
 * @param angle Angle in radians
 * @return Normalized angle in [-PI, PI)
 */
[[nodiscard]] inline double normalize_angle_signed(double angle) noexcept {
    return mod(angle + constants::PI, constants::TWO_PI) - constants::PI;
}

/**
 * @brief Alias for normalize_angle_signed (for compatibility)
 * 
 * @param angle Angle in radians
 * @return Normalized angle in [-PI, PI)
 */
[[nodiscard]] inline double normalize_radians(double angle) noexcept {
    return normalize_angle_signed(angle);
}

// =============================================================================
// Safe Trigonometric Functions
// =============================================================================

/**
 * @brief Safe arcsine that clamps input to [-1, 1]
 * 
 * Prevents NaN from rounding errors that push values slightly outside domain.
 * 
 * @param x Input value (will be clamped to [-1, 1])
 * @return Arcsine in radians [-PI/2, PI/2]
 */
[[nodiscard]] inline double asin_safe(double x) noexcept {
    if (x <= -1.0) return -constants::HALF_PI;
    if (x >= 1.0) return constants::HALF_PI;
    return std::asin(x);
}

/**
 * @brief Safe arccosine that clamps input to [-1, 1]
 * 
 * Prevents NaN from rounding errors that push values slightly outside domain.
 * 
 * @param x Input value (will be clamped to [-1, 1])
 * @return Arccosine in radians [0, PI]
 */
[[nodiscard]] inline double acos_safe(double x) noexcept {
    if (x <= -1.0) return constants::PI;
    if (x >= 1.0) return 0.0;
    return std::acos(x);
}

/**
 * @brief Safe square root that clamps negative values to zero
 * 
 * Prevents NaN from rounding errors that create tiny negative values.
 * 
 * @param x Input value (negative values return 0)
 * @return Square root, or 0 if x < 0
 */
[[nodiscard]] inline double sqrt_safe(double x) noexcept {
    if (x <= 0.0) return 0.0;
    return std::sqrt(x);
}

/**
 * @brief Check if a value is finite (not NaN or Inf)
 */
[[nodiscard]] inline bool is_valid(double x) noexcept {
    return std::isfinite(x) && !std::isnan(x);
}

/**
 * @brief Clamp a value to a range
 */
[[nodiscard]] inline double clamp(double x, double lo, double hi) noexcept {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// =============================================================================
// Angle Unit Conversions
// =============================================================================

/**
 * @brief Convert degrees to radians
 */
[[nodiscard]] constexpr double deg_to_rad(double degrees) noexcept {
    return degrees * constants::DEG_TO_RAD;
}

/**
 * @brief Convert radians to degrees
 */
[[nodiscard]] constexpr double rad_to_deg(double radians) noexcept {
    return radians * constants::RAD_TO_DEG;
}

// =============================================================================
// Distance Unit Conversions
// =============================================================================

/**
 * @brief Convert nautical miles to radians (great circle arc)
 */
[[nodiscard]] constexpr double nm_to_rad(double nm) noexcept {
    return nm * constants::NM_TO_RAD;
}

/**
 * @brief Convert radians (great circle arc) to nautical miles
 */
[[nodiscard]] constexpr double rad_to_nm(double radians) noexcept {
    return radians * constants::RAD_TO_NM;
}

/**
 * @brief Convert nautical miles to kilometers
 */
[[nodiscard]] constexpr double nm_to_km(double nm) noexcept {
    return nm * constants::NM_TO_KM;
}

/**
 * @brief Convert kilometers to nautical miles
 */
[[nodiscard]] constexpr double km_to_nm(double km) noexcept {
    return km * constants::KM_TO_NM;
}

/**
 * @brief Convert nautical miles to statute miles
 */
[[nodiscard]] constexpr double nm_to_sm(double nm) noexcept {
    return nm * constants::NM_TO_SM;
}

/**
 * @brief Convert statute miles to nautical miles
 */
[[nodiscard]] constexpr double sm_to_nm(double sm) noexcept {
    return sm * constants::SM_TO_NM;
}

/**
 * @brief Convert feet to meters
 */
[[nodiscard]] constexpr double ft_to_m(double ft) noexcept {
    return ft * constants::FT_TO_M;
}

/**
 * @brief Convert meters to feet
 */
[[nodiscard]] constexpr double m_to_ft(double m) noexcept {
    return m * constants::M_TO_FT;
}

/**
 * @brief Convert kilometers to meters
 */
[[nodiscard]] constexpr double km_to_m(double km) noexcept {
    return km * 1000.0;
}

/**
 * @brief Convert meters to kilometers
 */
[[nodiscard]] constexpr double m_to_km(double m) noexcept {
    return m / 1000.0;
}

// =============================================================================
// Speed Unit Conversions
// =============================================================================

/**
 * @brief Convert knots to km/h
 */
[[nodiscard]] constexpr double kt_to_kph(double kt) noexcept {
    return kt * constants::KT_TO_KPH;
}

/**
 * @brief Convert km/h to knots
 */
[[nodiscard]] constexpr double kph_to_kt(double kph) noexcept {
    return kph * constants::KPH_TO_KT;
}

/**
 * @brief Convert knots to mph
 */
[[nodiscard]] constexpr double kt_to_mph(double kt) noexcept {
    return kt * constants::KT_TO_MPH;
}

/**
 * @brief Convert mph to knots
 */
[[nodiscard]] constexpr double mph_to_kt(double mph) noexcept {
    return mph * constants::MPH_TO_KT;
}

/**
 * @brief Convert knots to m/s
 */
[[nodiscard]] constexpr double kt_to_mps(double kt) noexcept {
    return kt * constants::KT_TO_MPS;
}

/**
 * @brief Convert m/s to knots
 */
[[nodiscard]] constexpr double mps_to_kt(double mps) noexcept {
    return mps * constants::MPS_TO_KT;
}

// =============================================================================
// Temperature Unit Conversions
// =============================================================================

/**
 * @brief Convert Celsius to Kelvin
 */
[[nodiscard]] constexpr double c_to_k(double c) noexcept {
    return c + 273.15;
}

/**
 * @brief Convert Kelvin to Celsius
 */
[[nodiscard]] constexpr double k_to_c(double k) noexcept {
    return k - 273.15;
}

/**
 * @brief Convert Celsius to Fahrenheit
 */
[[nodiscard]] constexpr double c_to_f(double c) noexcept {
    return c * 1.8 + 32.0;
}

/**
 * @brief Convert Fahrenheit to Celsius
 */
[[nodiscard]] constexpr double f_to_c(double f) noexcept {
    return (f - 32.0) / 1.8;
}

/**
 * @brief Convert Kelvin to Fahrenheit
 */
[[nodiscard]] constexpr double k_to_f(double k) noexcept {
    return c_to_f(k_to_c(k));
}

/**
 * @brief Convert Fahrenheit to Kelvin
 */
[[nodiscard]] constexpr double f_to_k(double f) noexcept {
    return c_to_k(f_to_c(f));
}

// =============================================================================
// Pressure Unit Conversions
// =============================================================================

/**
 * @brief Convert millibars to inches of mercury
 */
[[nodiscard]] constexpr double mb_to_inhg(double mb) noexcept {
    return mb * constants::MB_TO_INHG;
}

/**
 * @brief Convert inches of mercury to millibars
 */
[[nodiscard]] constexpr double inhg_to_mb(double inhg) noexcept {
    return inhg * constants::INHG_TO_MB;
}

/**
 * @brief Convert millibars to hectopascals (identity - they're equal)
 */
[[nodiscard]] constexpr double mb_to_hpa(double mb) noexcept {
    return mb;  // 1 mb = 1 hPa
}

/**
 * @brief Convert hectopascals to millibars (identity - they're equal)
 */
[[nodiscard]] constexpr double hpa_to_mb(double hpa) noexcept {
    return hpa;  // 1 hPa = 1 mb
}

/**
 * @brief Convert millibars to Pascals
 */
[[nodiscard]] constexpr double mb_to_pa(double mb) noexcept {
    return mb * 100.0;  // 1 mb = 100 Pa
}

/**
 * @brief Convert Pascals to millibars
 */
[[nodiscard]] constexpr double pa_to_mb(double pa) noexcept {
    return pa / 100.0;
}

// =============================================================================
// Volume and Mass Unit Conversions
// =============================================================================

/**
 * @brief Convert US gallons to liters
 */
[[nodiscard]] constexpr double usgal_to_l(double gal) noexcept {
    return gal * constants::USGAL_TO_L;
}

/**
 * @brief Convert liters to US gallons
 */
[[nodiscard]] constexpr double l_to_usgal(double l) noexcept {
    return l * constants::L_TO_USGAL;
}

/**
 * @brief Convert Imperial gallons to liters
 */
[[nodiscard]] constexpr double impgal_to_l(double gal) noexcept {
    return gal * constants::IMPGAL_TO_L;
}

/**
 * @brief Convert liters to Imperial gallons
 */
[[nodiscard]] constexpr double l_to_impgal(double l) noexcept {
    return l * constants::L_TO_IMPGAL;
}

/**
 * @brief Convert pounds to kilograms
 */
[[nodiscard]] constexpr double lb_to_kg(double lb) noexcept {
    return lb * constants::LB_TO_KG;
}

/**
 * @brief Convert kilograms to pounds
 */
[[nodiscard]] constexpr double kg_to_lb(double kg) noexcept {
    return kg * constants::KG_TO_LB;
}

// =============================================================================
// Approximate Equality
// =============================================================================

/**
 * @brief Check if two values are approximately equal
 */
[[nodiscard]] inline bool approx_equal(double a, double b, double tolerance = constants::EPS) noexcept {
    return std::abs(a - b) < tolerance;
}

/**
 * @brief Check if two angles are approximately equal (handles wraparound)
 */
[[nodiscard]] inline bool angles_equal(double a, double b, double tolerance = constants::EPS) noexcept {
    double diff = normalize_angle_signed(a - b);
    return std::abs(diff) < tolerance;
}

} // namespace aviation

#endif // AVIATION_FORMULARY_CORE_MATH_HPP
