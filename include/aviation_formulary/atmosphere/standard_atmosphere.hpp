#ifndef AVIATION_FORMULARY_ATMOSPHERE_STANDARD_ATMOSPHERE_HPP
#define AVIATION_FORMULARY_ATMOSPHERE_STANDARD_ATMOSPHERE_HPP

/**
 * @file standard_atmosphere.hpp
 * @brief International Standard Atmosphere (ISA) calculations
 * 
 * Implements the International Standard Atmosphere model for calculating
 * temperature, pressure, and density at various altitudes.
 * 
 * The model uses two regions:
 * - Troposphere (0 to 36,089 ft / 11,000 m): Linear temperature decrease
 * - Lower Stratosphere (36,089 to 65,617 ft): Constant temperature
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
// Temperature Calculations
// =============================================================================

/**
 * @brief Calculate ISA temperature at a given altitude
 * 
 * @param altitude_ft Altitude in feet
 * @return Temperature in Kelvin
 */
[[nodiscard]] inline double isa_temperature_k(double altitude_ft) noexcept {
    if (altitude_ft <= constants::TROPOPAUSE_FT) {
        // Troposphere: linear decrease
        return constants::ISA_T0_K - (altitude_ft / 1000.0) * constants::ISA_LAPSE_RATE_C_PER_1000FT;
    } else {
        // Stratosphere: constant temperature
        return constants::TROPOPAUSE_TEMP_K;
    }
}

/**
 * @brief Calculate ISA temperature at a given altitude in Celsius
 * 
 * @param altitude_ft Altitude in feet
 * @return Temperature in Celsius
 */
[[nodiscard]] inline double isa_temperature_c(double altitude_ft) noexcept {
    return k_to_c(isa_temperature_k(altitude_ft));
}

/**
 * @brief Calculate ISA temperature at a given altitude (meters input)
 * 
 * @param altitude_m Altitude in meters
 * @return Temperature in Kelvin
 */
[[nodiscard]] inline double isa_temperature_k_m(double altitude_m) noexcept {
    return isa_temperature_k(m_to_ft(altitude_m));
}

/**
 * @brief Calculate temperature ratio (theta = T/T0)
 * 
 * @param altitude_ft Altitude in feet
 * @return Temperature ratio (dimensionless)
 */
[[nodiscard]] inline double temperature_ratio(double altitude_ft) noexcept {
    return isa_temperature_k(altitude_ft) / constants::ISA_T0_K;
}

/**
 * @brief Calculate ISA temperature deviation
 * 
 * Returns the difference between actual temperature and ISA temperature.
 * Positive values indicate warmer than ISA.
 * 
 * @param altitude_ft Altitude in feet
 * @param actual_temp_c Actual temperature in Celsius
 * @return Temperature deviation in Celsius (positive = warmer than ISA)
 */
[[nodiscard]] inline double isa_deviation(double altitude_ft, double actual_temp_c) noexcept {
    return actual_temp_c - isa_temperature_c(altitude_ft);
}

// =============================================================================
// Pressure Calculations
// =============================================================================

/**
 * @brief Calculate ISA pressure at a given altitude
 * 
 * @param altitude_ft Altitude in feet
 * @return Pressure in millibars (hPa)
 */
[[nodiscard]] inline double isa_pressure_mb(double altitude_ft) noexcept {
    if (altitude_ft <= constants::TROPOPAUSE_FT) {
        // Troposphere: pressure decreases with temperature
        double temp_ratio = temperature_ratio(altitude_ft);
        return constants::ISA_P0_MB * std::pow(temp_ratio, constants::PRESSURE_EXPONENT);
    } else {
        // Stratosphere: isothermal - pressure decreases exponentially
        double p_trop = isa_pressure_mb(constants::TROPOPAUSE_FT);
        double h_diff = altitude_ft - constants::TROPOPAUSE_FT;
        // Exponential decay in isothermal atmosphere
        // P = P_trop * exp(-g*h / (R*T))
        // Using feet: coefficient is approximately 4.8063e-5 per foot
        return p_trop * std::exp(-4.8063e-5 * h_diff);
    }
}

/**
 * @brief Calculate ISA pressure in inches of mercury
 * 
 * @param altitude_ft Altitude in feet
 * @return Pressure in inches Hg
 */
[[nodiscard]] inline double isa_pressure_inhg(double altitude_ft) noexcept {
    return mb_to_inhg(isa_pressure_mb(altitude_ft));
}

/**
 * @brief Calculate pressure ratio (delta = P/P0)
 * 
 * @param altitude_ft Altitude in feet
 * @return Pressure ratio (dimensionless)
 */
[[nodiscard]] inline double pressure_ratio(double altitude_ft) noexcept {
    return isa_pressure_mb(altitude_ft) / constants::ISA_P0_MB;
}

/**
 * @brief Calculate pressure ratio from actual pressure
 * 
 * @param pressure_mb Actual pressure in millibars
 * @return Pressure ratio (dimensionless)
 */
[[nodiscard]] inline double pressure_ratio_from_pressure(double pressure_mb) noexcept {
    return pressure_mb / constants::ISA_P0_MB;
}

// =============================================================================
// Density Calculations
// =============================================================================

/**
 * @brief Calculate ISA density at a given altitude
 * 
 * @param altitude_ft Altitude in feet
 * @return Density in kg/m³
 */
[[nodiscard]] inline double isa_density(double altitude_ft) noexcept {
    if (altitude_ft <= constants::TROPOPAUSE_FT) {
        // Troposphere
        double temp_ratio = temperature_ratio(altitude_ft);
        return constants::ISA_RHO0 * std::pow(temp_ratio, constants::DENSITY_EXPONENT);
    } else {
        // Stratosphere: density follows pressure (isothermal)
        double rho_trop = isa_density(constants::TROPOPAUSE_FT);
        double p_ratio = isa_pressure_mb(altitude_ft) / isa_pressure_mb(constants::TROPOPAUSE_FT);
        return rho_trop * p_ratio;
    }
}

/**
 * @brief Calculate density ratio (sigma = rho/rho0)
 * 
 * @param altitude_ft Altitude in feet
 * @return Density ratio (dimensionless)
 */
[[nodiscard]] inline double density_ratio(double altitude_ft) noexcept {
    return isa_density(altitude_ft) / constants::ISA_RHO0;
}

/**
 * @brief Calculate density ratio from pressure and temperature
 * 
 * Using ideal gas law: sigma = delta / theta
 * 
 * @param pressure_ratio_val Pressure ratio (delta)
 * @param temperature_ratio_val Temperature ratio (theta)
 * @return Density ratio (sigma)
 */
[[nodiscard]] inline double density_ratio_from_pt(double pressure_ratio_val, 
                                                   double temperature_ratio_val) noexcept {
    if (temperature_ratio_val < constants::EPS) {
        return 0.0;  // Avoid division by zero
    }
    return pressure_ratio_val / temperature_ratio_val;
}

// =============================================================================
// Speed of Sound
// =============================================================================

/**
 * @brief Calculate speed of sound at a given temperature
 * 
 * @param temperature_k Temperature in Kelvin
 * @return Speed of sound in knots
 */
[[nodiscard]] inline double speed_of_sound_kt(double temperature_k) noexcept {
    if (temperature_k <= 0) {
        return 0.0;  // Invalid temperature
    }
    return constants::SPEED_OF_SOUND_COEFF * std::sqrt(temperature_k);
}

/**
 * @brief Calculate speed of sound at a given altitude (ISA conditions)
 * 
 * @param altitude_ft Altitude in feet
 * @return Speed of sound in knots
 */
[[nodiscard]] inline double speed_of_sound_at_altitude(double altitude_ft) noexcept {
    return speed_of_sound_kt(isa_temperature_k(altitude_ft));
}

/**
 * @brief Calculate speed of sound in m/s
 * 
 * @param temperature_k Temperature in Kelvin
 * @return Speed of sound in m/s
 */
[[nodiscard]] inline double speed_of_sound_mps(double temperature_k) noexcept {
    if (temperature_k <= 0) {
        return 0.0;
    }
    // a = sqrt(gamma * R * T)
    return std::sqrt(constants::GAMMA * constants::R_DRY_AIR * temperature_k);
}

// =============================================================================
// Complete Atmosphere State
// =============================================================================

/**
 * @brief Get complete ISA atmosphere state at altitude
 * 
 * @param altitude_ft Altitude in feet
 * @return AtmosphereResult containing temperature, pressure, and density
 */
[[nodiscard]] inline AtmosphereResult isa_atmosphere(double altitude_ft) noexcept {
    AtmosphereResult result;
    
    // Check altitude limits
    constexpr double MAX_ALTITUDE_FT = 100000.0;  // Model limit
    if (altitude_ft < -1000.0) {
        result.error = ErrorCode::BelowFloor;
        return result;
    }
    if (altitude_ft > MAX_ALTITUDE_FT) {
        result.error = ErrorCode::AboveCeiling;
        return result;
    }
    
    result.temperature = isa_temperature_k(altitude_ft);
    result.pressure = isa_pressure_mb(altitude_ft);
    result.density = isa_density(altitude_ft);
    result.error = ErrorCode::Success;
    
    return result;
}

/**
 * @brief Calculate all atmosphere ratios at altitude
 * 
 * Returns theta (temperature ratio), delta (pressure ratio), and sigma (density ratio).
 */
struct AtmosphereRatios {
    double theta;   ///< Temperature ratio (T/T0)
    double delta;   ///< Pressure ratio (P/P0)
    double sigma;   ///< Density ratio (rho/rho0)
    
    constexpr AtmosphereRatios() noexcept : theta(1.0), delta(1.0), sigma(1.0) {}
    constexpr AtmosphereRatios(double t, double d, double s) noexcept 
        : theta(t), delta(d), sigma(s) {}
};

/**
 * @brief Get atmosphere ratios at altitude
 * 
 * @param altitude_ft Altitude in feet
 * @return AtmosphereRatios (theta, delta, sigma)
 */
[[nodiscard]] inline AtmosphereRatios atmosphere_ratios(double altitude_ft) noexcept {
    return AtmosphereRatios{
        temperature_ratio(altitude_ft),
        pressure_ratio(altitude_ft),
        density_ratio(altitude_ft)
    };
}

// =============================================================================
// Non-Standard Atmosphere
// =============================================================================

/**
 * @brief Calculate pressure ratio for non-standard temperature
 * 
 * Adjusts the standard atmosphere pressure ratio for actual temperature.
 * 
 * @param altitude_ft Pressure altitude in feet
 * @param actual_temp_c Actual outside air temperature in Celsius
 * @return Adjusted pressure ratio
 */
[[nodiscard]] inline double pressure_ratio_nonstandard(double altitude_ft, 
                                                        [[maybe_unused]] double actual_temp_c) noexcept {
    // Temperature affects density, not pressure at a given pressure altitude
    // So the pressure ratio is the same as standard at this pressure altitude
    return pressure_ratio(altitude_ft);
}

/**
 * @brief Calculate density ratio for non-standard temperature
 * 
 * @param altitude_ft Pressure altitude in feet
 * @param actual_temp_c Actual outside air temperature in Celsius
 * @return Adjusted density ratio
 */
[[nodiscard]] inline double density_ratio_nonstandard(double altitude_ft, 
                                                       double actual_temp_c) noexcept {
    double actual_temp_k = c_to_k(actual_temp_c);
    double delta = pressure_ratio(altitude_ft);
    double theta = actual_temp_k / constants::ISA_T0_K;
    
    if (theta < constants::EPS) {
        return 0.0;
    }
    
    return delta / theta;
}

} // namespace aviation

#endif // AVIATION_FORMULARY_ATMOSPHERE_STANDARD_ATMOSPHERE_HPP
