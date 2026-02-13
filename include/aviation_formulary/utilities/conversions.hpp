#ifndef AVIATION_FORMULARY_UTILITIES_CONVERSIONS_HPP
#define AVIATION_FORMULARY_UTILITIES_CONVERSIONS_HPP

/**
 * @file conversions.hpp
 * @brief Comprehensive unit conversion utilities
 * 
 * This header consolidates all unit conversions for convenience.
 * Most of these are also available in math.hpp.
 * 
 * Based on Ed Williams' Aviation Formulary V1.47
 * https://edwilliams.org/avform.htm
 */

#include "../core/constants.hpp"
#include "../core/math.hpp"

namespace aviation {

// =============================================================================
// Angle Conversions (from math.hpp)
// =============================================================================

// deg_to_rad() - defined in math.hpp
// rad_to_deg() - defined in math.hpp

// =============================================================================
// Distance Conversions
// =============================================================================

// Already in math.hpp: nm_to_rad, rad_to_nm, nm_to_km, km_to_nm, nm_to_sm, sm_to_nm
// Already in math.hpp: ft_to_m, m_to_ft, km_to_m, m_to_km

/**
 * @brief Convert kilometers to feet
 */
[[nodiscard]] constexpr double km_to_ft(double km) noexcept {
    return km * 1000.0 * constants::M_TO_FT;
}

/**
 * @brief Convert feet to kilometers
 */
[[nodiscard]] constexpr double ft_to_km(double ft) noexcept {
    return ft * constants::FT_TO_M / 1000.0;
}

/**
 * @brief Convert statute miles to feet
 */
[[nodiscard]] constexpr double sm_to_ft(double sm) noexcept {
    return sm * 5280.0;
}

/**
 * @brief Convert feet to statute miles
 */
[[nodiscard]] constexpr double ft_to_sm(double ft) noexcept {
    return ft / 5280.0;
}

/**
 * @brief Convert nautical miles to feet
 */
[[nodiscard]] constexpr double nm_to_ft(double nm) noexcept {
    return nm * 6076.12;
}

/**
 * @brief Convert feet to nautical miles
 */
[[nodiscard]] constexpr double ft_to_nm(double ft) noexcept {
    return ft / 6076.12;
}

/**
 * @brief Convert nautical miles to meters
 */
[[nodiscard]] constexpr double nm_to_m(double nm) noexcept {
    return nm * 1852.0;
}

/**
 * @brief Convert meters to nautical miles
 */
[[nodiscard]] constexpr double m_to_nm(double m) noexcept {
    return m / 1852.0;
}

/**
 * @brief Convert statute miles to kilometers
 */
[[nodiscard]] constexpr double sm_to_km(double sm) noexcept {
    return sm * 1.60934;
}

/**
 * @brief Convert kilometers to statute miles
 */
[[nodiscard]] constexpr double km_to_sm(double km) noexcept {
    return km / 1.60934;
}

/**
 * @brief Convert statute miles to meters
 */
[[nodiscard]] constexpr double sm_to_m(double sm) noexcept {
    return sm * 1609.34;
}

/**
 * @brief Convert meters to statute miles
 */
[[nodiscard]] constexpr double m_to_sm(double m) noexcept {
    return m / 1609.34;
}

// =============================================================================
// Speed Conversions
// =============================================================================

// Already in math.hpp: kt_to_kph, kph_to_kt, kt_to_mph, mph_to_kt, kt_to_mps, mps_to_kt

/**
 * @brief Convert feet per minute to knots
 */
[[nodiscard]] constexpr double fpm_to_kt(double fpm) noexcept {
    // 1 kt = 101.269 fpm
    return fpm / 101.269;
}

/**
 * @brief Convert knots to feet per minute
 */
[[nodiscard]] constexpr double kt_to_fpm(double kt) noexcept {
    return kt * 101.269;
}

/**
 * @brief Convert feet per minute to meters per second
 */
[[nodiscard]] constexpr double fpm_to_mps(double fpm) noexcept {
    return fpm * 0.00508;
}

/**
 * @brief Convert meters per second to feet per minute
 */
[[nodiscard]] constexpr double mps_to_fpm(double mps) noexcept {
    return mps / 0.00508;
}

/**
 * @brief Convert mph to km/h
 */
[[nodiscard]] constexpr double mph_to_kph(double mph) noexcept {
    return mph * 1.60934;
}

/**
 * @brief Convert km/h to mph
 */
[[nodiscard]] constexpr double kph_to_mph(double kph) noexcept {
    return kph / 1.60934;
}

/**
 * @brief Convert mph to m/s
 */
[[nodiscard]] constexpr double mph_to_mps(double mph) noexcept {
    return mph * 0.44704;
}

/**
 * @brief Convert m/s to mph
 */
[[nodiscard]] constexpr double mps_to_mph(double mps) noexcept {
    return mps / 0.44704;
}

/**
 * @brief Convert km/h to m/s
 */
[[nodiscard]] constexpr double kph_to_mps(double kph) noexcept {
    return kph / 3.6;
}

/**
 * @brief Convert m/s to km/h
 */
[[nodiscard]] constexpr double mps_to_kph(double mps) noexcept {
    return mps * 3.6;
}

// =============================================================================
// Temperature Conversions
// =============================================================================

// Already in math.hpp: c_to_k, k_to_c, c_to_f, f_to_c, k_to_f, f_to_k

/**
 * @brief Convert Rankine to Fahrenheit
 */
[[nodiscard]] constexpr double r_to_f(double r) noexcept {
    return r - 459.67;
}

/**
 * @brief Convert Fahrenheit to Rankine
 */
[[nodiscard]] constexpr double f_to_r(double f) noexcept {
    return f + 459.67;
}

/**
 * @brief Convert Kelvin to Rankine
 */
[[nodiscard]] constexpr double k_to_r(double k) noexcept {
    return k * 1.8;
}

/**
 * @brief Convert Rankine to Kelvin
 */
[[nodiscard]] constexpr double r_to_k(double r) noexcept {
    return r / 1.8;
}

// =============================================================================
// Pressure Conversions
// =============================================================================

// Already in math.hpp: mb_to_inhg, inhg_to_mb, mb_to_hpa, hpa_to_mb, mb_to_pa, pa_to_mb

/**
 * @brief Convert PSI to millibars
 */
[[nodiscard]] constexpr double psi_to_mb(double psi) noexcept {
    return psi * 68.9476;
}

/**
 * @brief Convert millibars to PSI
 */
[[nodiscard]] constexpr double mb_to_psi(double mb) noexcept {
    return mb / 68.9476;
}

/**
 * @brief Convert millimeters of mercury to millibars
 */
[[nodiscard]] constexpr double mmhg_to_mb(double mmhg) noexcept {
    return mmhg * 1.33322;
}

/**
 * @brief Convert millibars to millimeters of mercury
 */
[[nodiscard]] constexpr double mb_to_mmhg(double mb) noexcept {
    return mb / 1.33322;
}

/**
 * @brief Convert atmospheres to millibars
 */
[[nodiscard]] constexpr double atm_to_mb(double atm) noexcept {
    return atm * 1013.25;
}

/**
 * @brief Convert millibars to atmospheres
 */
[[nodiscard]] constexpr double mb_to_atm(double mb) noexcept {
    return mb / 1013.25;
}

// =============================================================================
// Mass Conversions
// =============================================================================

// Already in math.hpp: lb_to_kg, kg_to_lb

/**
 * @brief Convert ounces to grams
 */
[[nodiscard]] constexpr double oz_to_g(double oz) noexcept {
    return oz * 28.3495;
}

/**
 * @brief Convert grams to ounces
 */
[[nodiscard]] constexpr double g_to_oz(double g) noexcept {
    return g / 28.3495;
}

/**
 * @brief Convert stones to pounds
 */
[[nodiscard]] constexpr double stone_to_lb(double stone) noexcept {
    return stone * 14.0;
}

/**
 * @brief Convert pounds to stones
 */
[[nodiscard]] constexpr double lb_to_stone(double lb) noexcept {
    return lb / 14.0;
}

/**
 * @brief Convert metric tons to pounds
 */
[[nodiscard]] constexpr double tonne_to_lb(double tonne) noexcept {
    return tonne * 2204.62;
}

/**
 * @brief Convert pounds to metric tons
 */
[[nodiscard]] constexpr double lb_to_tonne(double lb) noexcept {
    return lb / 2204.62;
}

// =============================================================================
// Volume Conversions
// =============================================================================

// Already in math.hpp: usgal_to_l, l_to_usgal, impgal_to_l, l_to_impgal

/**
 * @brief Convert US gallons to Imperial gallons
 */
[[nodiscard]] constexpr double usgal_to_impgal(double usgal) noexcept {
    return usgal * 0.832674;
}

/**
 * @brief Convert Imperial gallons to US gallons
 */
[[nodiscard]] constexpr double impgal_to_usgal(double impgal) noexcept {
    return impgal / 0.832674;
}

/**
 * @brief Convert US quarts to liters
 */
[[nodiscard]] constexpr double usqt_to_l(double qt) noexcept {
    return qt * 0.946353;
}

/**
 * @brief Convert liters to US quarts
 */
[[nodiscard]] constexpr double l_to_usqt(double l) noexcept {
    return l / 0.946353;
}

/**
 * @brief Convert cubic meters to liters
 */
[[nodiscard]] constexpr double m3_to_l(double m3) noexcept {
    return m3 * 1000.0;
}

/**
 * @brief Convert liters to cubic meters
 */
[[nodiscard]] constexpr double l_to_m3(double l) noexcept {
    return l / 1000.0;
}

// =============================================================================
// Fuel Conversions
// =============================================================================

// Standard fuel densities (approximate, varies with temperature and type)
namespace fuel {

/// Avgas (100LL) density in lb/gal at 60°F
constexpr double AVGAS_DENSITY_LB_GAL = 6.0;

/// Jet A density in lb/gal at 60°F
constexpr double JET_A_DENSITY_LB_GAL = 6.7;

/// Jet A1 density in lb/gal at 60°F
constexpr double JET_A1_DENSITY_LB_GAL = 6.7;

/// Avgas density in kg/L
constexpr double AVGAS_DENSITY_KG_L = 0.72;

/// Jet A density in kg/L
constexpr double JET_A_DENSITY_KG_L = 0.804;

} // namespace fuel

/**
 * @brief Convert US gallons of avgas to pounds
 */
[[nodiscard]] constexpr double avgas_gal_to_lb(double gal) noexcept {
    return gal * fuel::AVGAS_DENSITY_LB_GAL;
}

/**
 * @brief Convert pounds of avgas to US gallons
 */
[[nodiscard]] constexpr double avgas_lb_to_gal(double lb) noexcept {
    return lb / fuel::AVGAS_DENSITY_LB_GAL;
}

/**
 * @brief Convert US gallons of Jet A to pounds
 */
[[nodiscard]] constexpr double jeta_gal_to_lb(double gal) noexcept {
    return gal * fuel::JET_A_DENSITY_LB_GAL;
}

/**
 * @brief Convert pounds of Jet A to US gallons
 */
[[nodiscard]] constexpr double jeta_lb_to_gal(double lb) noexcept {
    return lb / fuel::JET_A_DENSITY_LB_GAL;
}

/**
 * @brief Convert liters of avgas to kilograms
 */
[[nodiscard]] constexpr double avgas_l_to_kg(double l) noexcept {
    return l * fuel::AVGAS_DENSITY_KG_L;
}

/**
 * @brief Convert kilograms of avgas to liters
 */
[[nodiscard]] constexpr double avgas_kg_to_l(double kg) noexcept {
    return kg / fuel::AVGAS_DENSITY_KG_L;
}

/**
 * @brief Convert liters of Jet A to kilograms
 */
[[nodiscard]] constexpr double jeta_l_to_kg(double l) noexcept {
    return l * fuel::JET_A_DENSITY_KG_L;
}

/**
 * @brief Convert kilograms of Jet A to liters
 */
[[nodiscard]] constexpr double jeta_kg_to_l(double kg) noexcept {
    return kg / fuel::JET_A_DENSITY_KG_L;
}

// =============================================================================
// Time Conversions
// =============================================================================

/**
 * @brief Convert hours to minutes
 */
[[nodiscard]] constexpr double hr_to_min(double hr) noexcept {
    return hr * 60.0;
}

/**
 * @brief Convert minutes to hours
 */
[[nodiscard]] constexpr double min_to_hr(double min) noexcept {
    return min / 60.0;
}

/**
 * @brief Convert hours to seconds
 */
[[nodiscard]] constexpr double hr_to_sec(double hr) noexcept {
    return hr * 3600.0;
}

/**
 * @brief Convert seconds to hours
 */
[[nodiscard]] constexpr double sec_to_hr(double sec) noexcept {
    return sec / 3600.0;
}

/**
 * @brief Convert minutes to seconds
 */
[[nodiscard]] constexpr double min_to_sec(double min) noexcept {
    return min * 60.0;
}

/**
 * @brief Convert seconds to minutes
 */
[[nodiscard]] constexpr double sec_to_min(double sec) noexcept {
    return sec / 60.0;
}

// =============================================================================
// Aviation-Specific Conversions
// =============================================================================

/**
 * @brief Convert flight level to feet
 */
[[nodiscard]] constexpr double fl_to_ft(int fl) noexcept {
    return fl * 100.0;
}

/**
 * @brief Convert feet to flight level (rounded)
 */
[[nodiscard]] constexpr int ft_to_fl(double ft) noexcept {
    return static_cast<int>(ft / 100.0 + 0.5);
}

/**
 * @brief Convert degrees and minutes to decimal degrees
 * 
 * @param degrees Whole degrees
 * @param minutes Minutes (0-60)
 * @return Decimal degrees
 */
[[nodiscard]] inline double dm_to_decimal(double degrees, double minutes) noexcept {
    double sign = (degrees >= 0) ? 1.0 : -1.0;
    double abs_deg = (degrees >= 0) ? degrees : -degrees;
    return sign * (abs_deg + minutes / 60.0);
}

/**
 * @brief Convert degrees, minutes, seconds to decimal degrees
 * 
 * @param degrees Whole degrees
 * @param minutes Minutes (0-60)
 * @param seconds Seconds (0-60)
 * @return Decimal degrees
 */
[[nodiscard]] inline double dms_to_decimal(double degrees, 
                                            double minutes, 
                                            double seconds) noexcept {
    double sign = (degrees >= 0) ? 1.0 : -1.0;
    double abs_deg = (degrees >= 0) ? degrees : -degrees;
    return sign * (abs_deg + minutes / 60.0 + seconds / 3600.0);
}

/**
 * @brief Structure for degrees/minutes/seconds
 */
struct DMS {
    int degrees;
    int minutes;
    double seconds;
    bool negative;
    
    constexpr DMS() noexcept : degrees(0), minutes(0), seconds(0.0), negative(false) {}
    constexpr DMS(int d, int m, double s, bool neg = false) noexcept 
        : degrees(d), minutes(m), seconds(s), negative(neg) {}
};

/**
 * @brief Convert decimal degrees to degrees/minutes/seconds
 */
[[nodiscard]] inline DMS decimal_to_dms(double decimal) noexcept {
    DMS result;
    result.negative = (decimal < 0);
    decimal = std::abs(decimal);
    
    result.degrees = static_cast<int>(decimal);
    double remaining = (decimal - result.degrees) * 60.0;
    result.minutes = static_cast<int>(remaining);
    result.seconds = (remaining - result.minutes) * 60.0;
    
    return result;
}

} // namespace aviation

#endif // AVIATION_FORMULARY_UTILITIES_CONVERSIONS_HPP
