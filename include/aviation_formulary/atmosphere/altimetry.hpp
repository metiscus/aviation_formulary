#ifndef AVIATION_FORMULARY_ATMOSPHERE_ALTIMETRY_HPP
#define AVIATION_FORMULARY_ATMOSPHERE_ALTIMETRY_HPP

/**
 * @file altimetry.hpp
 * @brief Altitude calculations and conversions
 * 
 * Implements various altitude calculations including pressure altitude,
 * density altitude, true altitude, and altitude corrections.
 * 
 * Based on Ed Williams' Aviation Formulary V1.47
 * https://edwilliams.org/avform.htm
 */

#include "../core/constants.hpp"
#include "../core/types.hpp"
#include "../core/math.hpp"
#include "standard_atmosphere.hpp"
#include <cmath>

namespace aviation {

// =============================================================================
// Pressure Altitude
// =============================================================================

/**
 * @brief Calculate pressure altitude from altimeter setting
 * 
 * Pressure altitude is the altitude in standard atmosphere that corresponds
 * to the measured pressure.
 * 
 * @param indicated_altitude Indicated altitude in feet
 * @param altimeter_setting Altimeter setting in inches Hg
 * @return Pressure altitude in feet
 */
[[nodiscard]] inline double pressure_altitude(double indicated_altitude, 
                                               double altimeter_setting) noexcept {
    // PA = IA + 1000 * (29.92 - altimeter_setting)
    // This is an approximation valid for altimeter settings near standard
    return indicated_altitude + 1000.0 * (constants::ISA_P0_INHG - altimeter_setting);
}

/**
 * @brief Calculate pressure altitude from pressure
 * 
 * @param pressure_mb Atmospheric pressure in millibars
 * @return Pressure altitude in feet
 */
[[nodiscard]] inline double pressure_altitude_from_pressure(double pressure_mb) noexcept {
    if (pressure_mb <= 0 || pressure_mb > constants::ISA_P0_MB * 1.5) {
        return 0.0;  // Invalid pressure
    }
    
    // Check if we're in troposphere or stratosphere
    double p_trop = isa_pressure_mb(constants::TROPOPAUSE_FT);
    
    if (pressure_mb >= p_trop) {
        // Troposphere: inverse of pressure formula
        // P = P0 * (1 - L*h/T0)^(g/(R*L))
        // h = (T0/L) * (1 - (P/P0)^(R*L/g))
        double pressure_ratio = pressure_mb / constants::ISA_P0_MB;
        double exponent = 1.0 / constants::PRESSURE_EXPONENT;
        return (constants::ISA_T0_K / (constants::ISA_LAPSE_RATE_K_PER_M * constants::M_TO_FT)) * 
               (1.0 - std::pow(pressure_ratio, exponent));
    } else {
        // Stratosphere: inverse of exponential formula
        double h_diff = -std::log(pressure_mb / p_trop) / 4.8063e-5;
        return constants::TROPOPAUSE_FT + h_diff;
    }
}

/**
 * @brief Calculate pressure altitude from pressure (millibars), high precision
 * 
 * Uses the exact ISA formula rather than the approximation.
 * 
 * @param pressure_mb Pressure in millibars
 * @return Pressure altitude in feet
 */
[[nodiscard]] inline double pressure_altitude_exact(double pressure_mb) noexcept {
    // Iterate to find altitude (more precise than direct inversion)
    // Start with approximate altitude
    double alt = pressure_altitude_from_pressure(pressure_mb);
    
    // Newton-Raphson iteration
    for (int i = 0; i < 5; ++i) {
        double p_calc = isa_pressure_mb(alt);
        double error = p_calc - pressure_mb;
        
        if (std::abs(error) < 0.01) {  // 0.01 mb precision
            break;
        }
        
        // Approximate derivative (dp/dh)
        double dp_dh = (isa_pressure_mb(alt + 100) - isa_pressure_mb(alt - 100)) / 200.0;
        if (std::abs(dp_dh) > constants::EPS) {
            alt -= error / dp_dh;
        }
    }
    
    return alt;
}

// =============================================================================
// Density Altitude
// =============================================================================

/**
 * @brief Calculate density altitude
 * 
 * Density altitude is the altitude in standard atmosphere that has the
 * same air density as the actual conditions.
 * 
 * @param pressure_altitude_ft Pressure altitude in feet
 * @param temperature_c Outside air temperature in Celsius
 * @return Density altitude in feet
 */
[[nodiscard]] inline double density_altitude(double pressure_altitude_ft, 
                                              double temperature_c) noexcept {
    // Calculate ISA temperature at this pressure altitude
    double isa_temp_c = isa_temperature_c(pressure_altitude_ft);
    double isa_deviation = temperature_c - isa_temp_c;
    
    // Density altitude approximation:
    // DA ≈ PA + 120 * (OAT - ISA_temp)
    // This comes from the fact that density decreases about 120 ft per degree C
    return pressure_altitude_ft + 120.0 * isa_deviation;
}

/**
 * @brief Calculate density altitude (exact formula)
 * 
 * Uses the exact relationship between density and altitude.
 * 
 * @param pressure_altitude_ft Pressure altitude in feet
 * @param temperature_c Outside air temperature in Celsius
 * @return Density altitude in feet
 */
[[nodiscard]] inline double density_altitude_exact(double pressure_altitude_ft, 
                                                    double temperature_c) noexcept {
    // Get pressure ratio at this pressure altitude
    double delta = pressure_ratio(pressure_altitude_ft);
    
    // Get actual temperature ratio
    double theta = c_to_k(temperature_c) / constants::ISA_T0_K;
    
    if (theta < constants::EPS) {
        return pressure_altitude_ft;  // Avoid division by zero
    }
    
    // Calculate density ratio
    double sigma = delta / theta;
    
    // Find the ISA altitude with this density ratio
    // For troposphere: sigma = (1 - L*h/T0)^(g/(R*L) - 1)
    // Solving for h is complex, so we iterate
    
    double alt = pressure_altitude_ft;  // Start with PA
    
    for (int i = 0; i < 10; ++i) {
        double sigma_calc = density_ratio(alt);
        double error = sigma_calc - sigma;
        
        if (std::abs(error) < 1e-6) {
            break;
        }
        
        // Approximate derivative
        double d_sigma = (density_ratio(alt + 100) - density_ratio(alt - 100)) / 200.0;
        if (std::abs(d_sigma) > constants::EPS) {
            alt -= error / d_sigma;
        }
    }
    
    return alt;
}

/**
 * @brief Calculate density altitude from station conditions
 * 
 * Convenience function using altimeter setting instead of pressure altitude.
 * 
 * @param indicated_altitude Indicated altitude (field elevation) in feet
 * @param altimeter_setting Altimeter setting in inches Hg
 * @param temperature_c Outside air temperature in Celsius
 * @return Density altitude in feet
 */
[[nodiscard]] inline double density_altitude_station(double indicated_altitude,
                                                      double altimeter_setting,
                                                      double temperature_c) noexcept {
    double pa = pressure_altitude(indicated_altitude, altimeter_setting);
    return density_altitude(pa, temperature_c);
}

// =============================================================================
// True Altitude
// =============================================================================

/**
 * @brief Calculate true altitude from indicated altitude
 * 
 * True altitude is the actual height above mean sea level, corrected for
 * non-standard temperature.
 * 
 * @param indicated_altitude Indicated altitude in feet
 * @param altimeter_setting Altimeter setting in inches Hg
 * @param temperature_c Outside air temperature in Celsius
 * @return True altitude in feet
 */
[[nodiscard]] inline double true_altitude(double indicated_altitude,
                                           double altimeter_setting,
                                           double temperature_c) noexcept {
    double pa = pressure_altitude(indicated_altitude, altimeter_setting);
    
    // Get ISA temperature at this pressure altitude
    double isa_temp_k = isa_temperature_k(pa);
    double actual_temp_k = c_to_k(temperature_c);
    
    // True altitude = PA * (actual_temp / isa_temp)
    // This is an approximation for the effect of temperature on altitude
    return pa * (actual_temp_k / isa_temp_k);
}

/**
 * @brief Calculate true altitude correction factor
 * 
 * Returns the ratio by which indicated altitude differs from true altitude.
 * 
 * @param pressure_altitude_ft Pressure altitude in feet
 * @param temperature_c Outside air temperature in Celsius
 * @return Correction factor (multiply indicated altitude by this)
 */
[[nodiscard]] inline double true_altitude_factor(double pressure_altitude_ft,
                                                  double temperature_c) noexcept {
    double isa_temp_k = isa_temperature_k(pressure_altitude_ft);
    double actual_temp_k = c_to_k(temperature_c);
    
    if (isa_temp_k < constants::EPS) {
        return 1.0;
    }
    
    return actual_temp_k / isa_temp_k;
}

// =============================================================================
// Altitude Conversions
// =============================================================================

/**
 * @brief Convert QNH (altimeter setting) to QFE (field pressure)
 * 
 * @param qnh_inhg Altimeter setting (QNH) in inches Hg
 * @param field_elevation_ft Field elevation in feet
 * @return Field pressure (QFE) in inches Hg
 */
[[nodiscard]] inline double qnh_to_qfe(double qnh_inhg, double field_elevation_ft) noexcept {
    // QFE = QNH - (field_elevation / 1000)
    // More accurate: use ISA pressure relationship
    double qnh_mb = inhg_to_mb(qnh_inhg);
    double pressure_ratio_field = isa_pressure_mb(field_elevation_ft) / constants::ISA_P0_MB;
    return mb_to_inhg(qnh_mb * pressure_ratio_field);
}

/**
 * @brief Convert QFE (field pressure) to QNH (altimeter setting)
 * 
 * @param qfe_inhg Field pressure (QFE) in inches Hg
 * @param field_elevation_ft Field elevation in feet
 * @return Altimeter setting (QNH) in inches Hg
 */
[[nodiscard]] inline double qfe_to_qnh(double qfe_inhg, double field_elevation_ft) noexcept {
    double qfe_mb = inhg_to_mb(qfe_inhg);
    double pressure_ratio_field = isa_pressure_mb(field_elevation_ft) / constants::ISA_P0_MB;
    
    if (pressure_ratio_field < constants::EPS) {
        return qfe_inhg;
    }
    
    return mb_to_inhg(qfe_mb / pressure_ratio_field);
}

/**
 * @brief Calculate flight level from pressure altitude
 * 
 * @param pressure_altitude_ft Pressure altitude in feet
 * @return Flight level (FL)
 */
[[nodiscard]] inline int flight_level(double pressure_altitude_ft) noexcept {
    return static_cast<int>(pressure_altitude_ft / 100.0 + 0.5);
}

/**
 * @brief Convert flight level to pressure altitude
 * 
 * @param fl Flight level
 * @return Pressure altitude in feet
 */
[[nodiscard]] inline double fl_to_pressure_altitude(int fl) noexcept {
    return fl * 100.0;
}

// =============================================================================
// Altitude for Target Conditions
// =============================================================================

/**
 * @brief Find altitude for a given density ratio
 * 
 * @param target_sigma Target density ratio
 * @return Altitude in feet where this density ratio occurs in ISA
 */
[[nodiscard]] inline Result<double> altitude_for_density_ratio(double target_sigma) noexcept {
    if (target_sigma <= 0 || target_sigma > 1.5) {
        return Result<double>::err(ErrorCode::OutOfRange);
    }
    
    // Binary search
    double lo = -1000.0;
    double hi = 100000.0;
    
    for (int i = 0; i < 50; ++i) {
        double mid = (lo + hi) / 2.0;
        double sigma = density_ratio(mid);
        
        if (std::abs(sigma - target_sigma) < 1e-8) {
            return Result<double>::ok(mid);
        }
        
        if (sigma > target_sigma) {
            lo = mid;  // Need higher altitude (lower density)
        } else {
            hi = mid;  // Need lower altitude (higher density)
        }
    }
    
    return Result<double>::ok((lo + hi) / 2.0);
}

/**
 * @brief Find altitude for a given pressure ratio
 * 
 * @param target_delta Target pressure ratio
 * @return Altitude in feet where this pressure ratio occurs in ISA
 */
[[nodiscard]] inline Result<double> altitude_for_pressure_ratio(double target_delta) noexcept {
    if (target_delta <= 0 || target_delta > 1.5) {
        return Result<double>::err(ErrorCode::OutOfRange);
    }
    
    double pressure_mb = target_delta * constants::ISA_P0_MB;
    return Result<double>::ok(pressure_altitude_from_pressure(pressure_mb));
}

} // namespace aviation

#endif // AVIATION_FORMULARY_ATMOSPHERE_ALTIMETRY_HPP
