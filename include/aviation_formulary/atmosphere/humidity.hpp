#ifndef AVIATION_FORMULARY_ATMOSPHERE_HUMIDITY_HPP
#define AVIATION_FORMULARY_ATMOSPHERE_HUMIDITY_HPP

/**
 * @file humidity.hpp
 * @brief Humidity, dewpoint, and related calculations
 * 
 * Implements calculations for relative humidity, dewpoint, frostpoint,
 * and the effects of humidity on density altitude.
 * 
 * Based on Ed Williams' Aviation Formulary V1.47
 * https://edwilliams.org/avform.htm
 */

#include "../core/constants.hpp"
#include "../core/types.hpp"
#include "../core/math.hpp"
#include "standard_atmosphere.hpp"
#include "altimetry.hpp"
#include <cmath>

namespace aviation {

// =============================================================================
// Saturation Vapor Pressure
// =============================================================================

/**
 * @brief Calculate saturation vapor pressure over water
 * 
 * Uses the Magnus formula (also known as Tetens formula).
 * 
 * @param temperature_c Temperature in Celsius
 * @return Saturation vapor pressure in millibars
 */
[[nodiscard]] inline double saturation_vapor_pressure(double temperature_c) noexcept {
    // e_s = 6.1121 * exp(17.502 * T / (240.97 + T))
    // Valid for T > -40°C
    return constants::VAPOR_PRESSURE_A * 
           std::exp(constants::VAPOR_PRESSURE_B_WATER * temperature_c / 
                    (constants::VAPOR_PRESSURE_C_WATER + temperature_c));
}

/**
 * @brief Calculate saturation vapor pressure over ice
 * 
 * For temperatures below freezing, saturation vapor pressure over ice
 * is lower than over water.
 * 
 * @param temperature_c Temperature in Celsius (should be < 0)
 * @return Saturation vapor pressure in millibars
 */
[[nodiscard]] inline double saturation_vapor_pressure_ice(double temperature_c) noexcept {
    // e_s = 6.1121 * exp(22.587 * T / (273.86 + T))
    return constants::VAPOR_PRESSURE_A * 
           std::exp(constants::VAPOR_PRESSURE_B_ICE * temperature_c / 
                    (constants::VAPOR_PRESSURE_C_ICE + temperature_c));
}

/**
 * @brief Calculate saturation vapor pressure (automatic ice/water selection)
 * 
 * Automatically selects ice or water saturation based on temperature.
 * 
 * @param temperature_c Temperature in Celsius
 * @return Saturation vapor pressure in millibars
 */
[[nodiscard]] inline double saturation_vapor_pressure_auto(double temperature_c) noexcept {
    if (temperature_c < 0.0) {
        return saturation_vapor_pressure_ice(temperature_c);
    } else {
        return saturation_vapor_pressure(temperature_c);
    }
}

// =============================================================================
// Relative Humidity
// =============================================================================

/**
 * @brief Calculate relative humidity from temperature and dewpoint
 * 
 * @param temperature_c Air temperature in Celsius
 * @param dewpoint_c Dewpoint temperature in Celsius
 * @return Relative humidity as a fraction [0, 1]
 */
[[nodiscard]] inline double relative_humidity(double temperature_c, 
                                               double dewpoint_c) noexcept {
    // RH = e_s(Td) / e_s(T)
    // Using the Magnus formula, this simplifies to:
    double e_s_temp = saturation_vapor_pressure(temperature_c);
    double e_s_dew = saturation_vapor_pressure(dewpoint_c);
    
    if (e_s_temp < constants::EPS) {
        return 0.0;
    }
    
    double rh = e_s_dew / e_s_temp;
    return clamp(rh, 0.0, 1.0);
}

/**
 * @brief Calculate relative humidity as percentage
 * 
 * @param temperature_c Air temperature in Celsius
 * @param dewpoint_c Dewpoint temperature in Celsius
 * @return Relative humidity as percentage [0, 100]
 */
[[nodiscard]] inline double relative_humidity_percent(double temperature_c, 
                                                       double dewpoint_c) noexcept {
    return relative_humidity(temperature_c, dewpoint_c) * 100.0;
}

// =============================================================================
// Dewpoint
// =============================================================================

/**
 * @brief Calculate dewpoint from temperature and relative humidity
 * 
 * @param temperature_c Air temperature in Celsius
 * @param relative_humidity_frac Relative humidity as fraction [0, 1]
 * @return Dewpoint temperature in Celsius
 */
[[nodiscard]] inline double dewpoint(double temperature_c, 
                                      double relative_humidity_frac) noexcept {
    // Clamp RH to valid range
    relative_humidity_frac = clamp(relative_humidity_frac, 0.001, 1.0);
    
    // Using Magnus formula:
    // Td = C * (ln(e/A)) / (B - ln(e/A))
    // where e = RH * e_s(T)
    
    double e_s = saturation_vapor_pressure(temperature_c);
    double e = relative_humidity_frac * e_s;
    
    double ln_e_over_a = std::log(e / constants::VAPOR_PRESSURE_A);
    
    return constants::VAPOR_PRESSURE_C_WATER * ln_e_over_a / 
           (constants::VAPOR_PRESSURE_B_WATER - ln_e_over_a);
}

/**
 * @brief Calculate frostpoint from temperature and relative humidity
 * 
 * Frostpoint is the temperature at which frost forms (saturation over ice).
 * 
 * @param temperature_c Air temperature in Celsius
 * @param relative_humidity_frac Relative humidity as fraction [0, 1]
 * @return Frostpoint temperature in Celsius
 */
[[nodiscard]] inline double frostpoint(double temperature_c, 
                                        double relative_humidity_frac) noexcept {
    relative_humidity_frac = clamp(relative_humidity_frac, 0.001, 1.0);
    
    // Calculate vapor pressure from RH and saturation over water
    double e_s_water = saturation_vapor_pressure(temperature_c);
    double e = relative_humidity_frac * e_s_water;
    
    // Find temperature where e equals saturation over ice
    double ln_e_over_a = std::log(e / constants::VAPOR_PRESSURE_A);
    
    return constants::VAPOR_PRESSURE_C_ICE * ln_e_over_a / 
           (constants::VAPOR_PRESSURE_B_ICE - ln_e_over_a);
}

// =============================================================================
// Wet Bulb Temperature
// =============================================================================

/**
 * @brief Estimate wet bulb temperature from dry bulb and dewpoint
 * 
 * This is an approximation. For precise wet bulb temperature, iterative
 * psychrometric calculations are needed.
 * 
 * @param temperature_c Dry bulb temperature in Celsius
 * @param dewpoint_c Dewpoint temperature in Celsius
 * @return Approximate wet bulb temperature in Celsius
 */
[[nodiscard]] inline double wet_bulb_approx(double temperature_c, 
                                             double dewpoint_c) noexcept {
    // Stull formula (2011) - accurate to within 0.3°C for most conditions
    double rh = relative_humidity_percent(temperature_c, dewpoint_c);
    
    double tw = temperature_c * std::atan(0.151977 * std::sqrt(rh + 8.313659)) +
                std::atan(temperature_c + rh) - 
                std::atan(rh - 1.676331) +
                0.00391838 * std::pow(rh, 1.5) * std::atan(0.023101 * rh) -
                4.686035;
    
    return tw;
}

/**
 * @brief Calculate dewpoint from wet and dry bulb temperatures
 * 
 * Uses psychrometric relationships.
 * 
 * @param dry_bulb_c Dry bulb temperature in Celsius
 * @param wet_bulb_c Wet bulb temperature in Celsius
 * @param pressure_mb Atmospheric pressure in millibars
 * @return Dewpoint temperature in Celsius
 */
[[nodiscard]] inline double dewpoint_from_wet_bulb(double dry_bulb_c, 
                                                    double wet_bulb_c,
                                                    double pressure_mb) noexcept {
    // Psychrometric formula
    double e_s_wet = saturation_vapor_pressure(wet_bulb_c);
    
    // Psychrometric constant (approximate)
    constexpr double A = 0.00066;  // °C^-1 for standard conditions
    
    // Actual vapor pressure
    double e = e_s_wet - A * pressure_mb * (dry_bulb_c - wet_bulb_c);
    
    if (e < constants::VAPOR_PRESSURE_A * 0.001) {
        return dry_bulb_c - 50.0;  // Very dry - approximate
    }
    
    // Invert Magnus formula
    double ln_e_over_a = std::log(e / constants::VAPOR_PRESSURE_A);
    return constants::VAPOR_PRESSURE_C_WATER * ln_e_over_a / 
           (constants::VAPOR_PRESSURE_B_WATER - ln_e_over_a);
}

/**
 * @brief Calculate relative humidity from wet and dry bulb temperatures
 * 
 * @param dry_bulb_c Dry bulb temperature in Celsius
 * @param wet_bulb_c Wet bulb temperature in Celsius
 * @param pressure_mb Atmospheric pressure in millibars
 * @return Relative humidity as fraction [0, 1]
 */
[[nodiscard]] inline double rh_from_wet_bulb(double dry_bulb_c, 
                                              double wet_bulb_c,
                                              double pressure_mb) noexcept {
    double td = dewpoint_from_wet_bulb(dry_bulb_c, wet_bulb_c, pressure_mb);
    return relative_humidity(dry_bulb_c, td);
}

// =============================================================================
// Humidity Effects on Density Altitude
// =============================================================================

/**
 * @brief Calculate density altitude correction for humidity
 * 
 * Humid air is less dense than dry air at the same temperature and pressure.
 * This function calculates the additional density altitude due to humidity.
 * 
 * From the formulary:
 * DA_humid = DA + 0.0065*f*exp(0.0373*T)*(H + 6812)
 * 
 * @param relative_humidity_frac Relative humidity as fraction [0, 1]
 * @param temperature_c Outside air temperature in Celsius
 * @param pressure_altitude_ft Pressure altitude in feet
 * @return Density altitude increase due to humidity in feet
 */
[[nodiscard]] inline double humidity_density_altitude_correction(
    double relative_humidity_frac, 
    double temperature_c,
    double pressure_altitude_ft) noexcept {
    
    // Clamp inputs
    relative_humidity_frac = clamp(relative_humidity_frac, 0.0, 1.0);
    
    // Formula from Aviation Formulary
    return 0.0065 * relative_humidity_frac * 
           std::exp(0.0373 * temperature_c) * 
           (pressure_altitude_ft + 6812.0);
}

/**
 * @brief Calculate total density altitude including humidity effects
 * 
 * @param pressure_altitude_ft Pressure altitude in feet
 * @param temperature_c Outside air temperature in Celsius
 * @param dewpoint_c Dewpoint temperature in Celsius
 * @return Total density altitude in feet
 */
[[nodiscard]] inline double density_altitude_humid(double pressure_altitude_ft,
                                                    double temperature_c,
                                                    double dewpoint_c) noexcept {
    // Calculate dry density altitude
    double da_dry = density_altitude(pressure_altitude_ft, temperature_c);
    
    // Calculate humidity
    double rh = relative_humidity(temperature_c, dewpoint_c);
    
    // Add humidity correction
    double humidity_correction = humidity_density_altitude_correction(
        rh, temperature_c, pressure_altitude_ft);
    
    return da_dry + humidity_correction;
}

// =============================================================================
// Air Density with Humidity
// =============================================================================

/**
 * @brief Calculate virtual temperature
 * 
 * Virtual temperature is the temperature at which dry air would have
 * the same density as moist air at the actual temperature.
 * 
 * @param temperature_c Actual temperature in Celsius
 * @param dewpoint_c Dewpoint temperature in Celsius
 * @param pressure_mb Atmospheric pressure in millibars
 * @return Virtual temperature in Celsius
 */
[[nodiscard]] inline double virtual_temperature(double temperature_c,
                                                  double dewpoint_c,
                                                  double pressure_mb) noexcept {
    double e = saturation_vapor_pressure(dewpoint_c);  // Vapor pressure ≈ sat pressure at dewpoint
    double temperature_k = c_to_k(temperature_c);
    
    // Tv = T / (1 - 0.378 * e / p)
    double denom = 1.0 - 0.378 * e / pressure_mb;
    
    if (denom < constants::EPS) {
        return temperature_c;
    }
    
    return k_to_c(temperature_k / denom);
}

/**
 * @brief Calculate air density including humidity effects
 * 
 * @param temperature_c Temperature in Celsius
 * @param pressure_mb Pressure in millibars
 * @param relative_humidity_frac Relative humidity as fraction [0, 1]
 * @return Air density in kg/m³
 */
[[nodiscard]] inline double air_density_humid(double temperature_c,
                                               double pressure_mb,
                                               double relative_humidity_frac) noexcept {
    // Calculate vapor pressure
    double e_s = saturation_vapor_pressure(temperature_c);
    double e = relative_humidity_frac * e_s;
    
    // Dry air partial pressure
    double p_dry = pressure_mb - e;
    
    // Convert to Pa
    double p_dry_pa = p_dry * 100.0;
    double e_pa = e * 100.0;
    
    // Temperature in K
    double temperature_k = c_to_k(temperature_c);
    
    // Density = (p_dry / (R_dry * T)) + (e / (R_vapor * T))
    // R_vapor ≈ 461.5 J/(kg·K)
    constexpr double R_VAPOR = 461.5;
    
    return (p_dry_pa / (constants::R_DRY_AIR * temperature_k)) +
           (e_pa / (R_VAPOR * temperature_k));
}

// =============================================================================
// Mixing Ratio and Specific Humidity
// =============================================================================

/**
 * @brief Calculate mixing ratio
 * 
 * Mixing ratio is the mass of water vapor per unit mass of dry air.
 * 
 * @param vapor_pressure_mb Vapor pressure in millibars
 * @param total_pressure_mb Total atmospheric pressure in millibars
 * @return Mixing ratio (kg water / kg dry air)
 */
[[nodiscard]] inline double mixing_ratio(double vapor_pressure_mb,
                                          double total_pressure_mb) noexcept {
    // w = 0.622 * e / (p - e)
    double p_dry = total_pressure_mb - vapor_pressure_mb;
    
    if (p_dry < constants::EPS) {
        return 0.0;
    }
    
    return 0.622 * vapor_pressure_mb / p_dry;
}

/**
 * @brief Calculate specific humidity
 * 
 * Specific humidity is the mass of water vapor per unit mass of moist air.
 * 
 * @param vapor_pressure_mb Vapor pressure in millibars
 * @param total_pressure_mb Total atmospheric pressure in millibars
 * @return Specific humidity (kg water / kg moist air)
 */
[[nodiscard]] inline double specific_humidity(double vapor_pressure_mb,
                                               double total_pressure_mb) noexcept {
    // q = 0.622 * e / (p - 0.378 * e)
    double denom = total_pressure_mb - 0.378 * vapor_pressure_mb;
    
    if (denom < constants::EPS) {
        return 0.0;
    }
    
    return 0.622 * vapor_pressure_mb / denom;
}

} // namespace aviation

#endif // AVIATION_FORMULARY_ATMOSPHERE_HUMIDITY_HPP
