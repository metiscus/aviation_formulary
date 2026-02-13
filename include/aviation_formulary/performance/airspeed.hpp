#ifndef AVIATION_FORMULARY_PERFORMANCE_AIRSPEED_HPP
#define AVIATION_FORMULARY_PERFORMANCE_AIRSPEED_HPP

/**
 * @file airspeed.hpp
 * @brief Airspeed and Mach number calculations
 * 
 * Implements conversions between various airspeeds (IAS, CAS, TAS, EAS)
 * and Mach number calculations.
 * 
 * Airspeed Types:
 * - IAS: Indicated Airspeed (what the pilot reads)
 * - CAS: Calibrated Airspeed (IAS corrected for instrument/position error)
 * - EAS: Equivalent Airspeed (CAS corrected for compressibility)
 * - TAS: True Airspeed (actual speed through the air)
 * - GS: Ground Speed (speed over the ground)
 * 
 * Based on Ed Williams' Aviation Formulary V1.47
 * https://edwilliams.org/avform.htm
 */

#include "../core/constants.hpp"
#include "../core/types.hpp"
#include "../core/math.hpp"
#include "../atmosphere/standard_atmosphere.hpp"
#include <cmath>

namespace aviation {

// =============================================================================
// Mach Number Calculations
// =============================================================================

/**
 * @brief Calculate Mach number from TAS and temperature
 * 
 * @param tas_kt True airspeed in knots
 * @param temperature_k Temperature in Kelvin
 * @return Mach number
 */
[[nodiscard]] inline double mach_from_tas(double tas_kt, double temperature_k) noexcept {
    double a = speed_of_sound_kt(temperature_k);
    if (a < constants::EPS) {
        return 0.0;
    }
    return tas_kt / a;
}

/**
 * @brief Calculate Mach number from TAS at altitude (ISA)
 * 
 * @param tas_kt True airspeed in knots
 * @param altitude_ft Altitude in feet (uses ISA temperature)
 * @return Mach number
 */
[[nodiscard]] inline double mach_from_tas_altitude(double tas_kt, double altitude_ft) noexcept {
    double a = speed_of_sound_at_altitude(altitude_ft);
    if (a < constants::EPS) {
        return 0.0;
    }
    return tas_kt / a;
}

/**
 * @brief Calculate TAS from Mach number and temperature
 * 
 * @param mach Mach number
 * @param temperature_k Temperature in Kelvin
 * @return True airspeed in knots
 */
[[nodiscard]] inline double tas_from_mach(double mach, double temperature_k) noexcept {
    return mach * speed_of_sound_kt(temperature_k);
}

/**
 * @brief Calculate TAS from Mach number at altitude (ISA)
 * 
 * @param mach Mach number
 * @param altitude_ft Altitude in feet
 * @return True airspeed in knots
 */
[[nodiscard]] inline double tas_from_mach_altitude(double mach, double altitude_ft) noexcept {
    return mach * speed_of_sound_at_altitude(altitude_ft);
}

// =============================================================================
// CAS/Mach Relationships (Subsonic)
// =============================================================================

/**
 * @brief Calculate Mach number from CAS and pressure altitude (subsonic)
 * 
 * Valid for Mach < 1.0
 * 
 * From the formulary:
 * M = sqrt(5*((1 + 0.2*(CAS/a0)^2)^3.5 * (1/delta) - 1/delta + 1)^(2/7) - 1))
 * 
 * @param cas_kt Calibrated airspeed in knots
 * @param altitude_ft Pressure altitude in feet
 * @return Result containing Mach number
 */
[[nodiscard]] inline Result<double> mach_from_cas(double cas_kt, double altitude_ft) noexcept {
    if (cas_kt < 0) {
        return Result<double>::err(ErrorCode::InvalidSpeed, 0.0);
    }
    
    if (cas_kt < constants::EPS) {
        return Result<double>::ok(0.0);
    }
    
    double delta = pressure_ratio(altitude_ft);
    
    if (delta < constants::EPS) {
        return Result<double>::err(ErrorCode::OutOfRange, 0.0);
    }
    
    // a0 is sea level speed of sound
    double cas_ratio = cas_kt / constants::SPEED_OF_SOUND_SL_KT;
    
    // Impact pressure ratio at sea level
    double qc_p0 = std::pow(1.0 + 0.2 * cas_ratio * cas_ratio, 3.5) - 1.0;
    
    // Mach number
    double inner = (qc_p0 / delta + 1.0);
    if (inner < 0) {
        return Result<double>::err(ErrorCode::OutOfRange, 0.0);
    }
    
    double mach_sq = 5.0 * (std::pow(inner, 2.0/7.0) - 1.0);
    if (mach_sq < 0) {
        return Result<double>::err(ErrorCode::OutOfRange, 0.0);
    }
    
    double mach = std::sqrt(mach_sq);
    
    // Check for supersonic (this formula only valid for subsonic)
    if (mach > 1.0) {
        return Result<double>::err(ErrorCode::InvalidMachNumber, mach);
    }
    
    return Result<double>::ok(mach);
}

/**
 * @brief Calculate CAS from Mach number and pressure altitude (subsonic)
 * 
 * Valid for Mach < 1.0
 * 
 * @param mach Mach number
 * @param altitude_ft Pressure altitude in feet
 * @return Result containing CAS in knots
 */
[[nodiscard]] inline Result<double> cas_from_mach(double mach, double altitude_ft) noexcept {
    if (mach < 0) {
        return Result<double>::err(ErrorCode::InvalidMachNumber, 0.0);
    }
    
    if (mach < constants::EPS) {
        return Result<double>::ok(0.0);
    }
    
    if (mach > 1.0) {
        // Use supersonic formula
        return Result<double>::err(ErrorCode::InvalidMachNumber, 0.0);
    }
    
    double delta = pressure_ratio(altitude_ft);
    
    // Impact pressure ratio at altitude
    double qc_p = std::pow(1.0 + 0.2 * mach * mach, 3.5) - 1.0;
    
    // Impact pressure ratio at sea level
    double qc_p0 = qc_p * delta;
    
    // CAS
    double inner = qc_p0 + 1.0;
    if (inner < 0) {
        return Result<double>::err(ErrorCode::OutOfRange, 0.0);
    }
    
    double cas_ratio_sq = 5.0 * (std::pow(inner, 2.0/7.0) - 1.0);
    if (cas_ratio_sq < 0) {
        return Result<double>::err(ErrorCode::OutOfRange, 0.0);
    }
    
    double cas = constants::SPEED_OF_SOUND_SL_KT * std::sqrt(cas_ratio_sq);
    
    return Result<double>::ok(cas);
}

// =============================================================================
// CAS/Mach Relationships (Supersonic)
// =============================================================================

/**
 * @brief Calculate Mach number from CAS and pressure altitude (supersonic)
 * 
 * Valid for Mach > 1.0
 * Uses Rayleigh pitot formula.
 * 
 * @param cas_kt Calibrated airspeed in knots
 * @param altitude_ft Pressure altitude in feet
 * @return Result containing Mach number (> 1.0)
 */
[[nodiscard]] inline Result<double> mach_from_cas_supersonic(double cas_kt, 
                                                              double altitude_ft) noexcept {
    // First check if we're actually supersonic
    auto subsonic_result = mach_from_cas(cas_kt, altitude_ft);
    if (subsonic_result && subsonic_result.value < 1.0) {
        return subsonic_result;  // Actually subsonic
    }
    
    // For supersonic, we need to iterate using Rayleigh formula
    double delta = pressure_ratio(altitude_ft);
    double cas_ratio = cas_kt / constants::SPEED_OF_SOUND_SL_KT;
    double qc_p0 = std::pow(1.0 + 0.2 * cas_ratio * cas_ratio, 3.5) - 1.0;
    
    // Initial guess
    double mach = 1.5;
    
    // Newton-Raphson iteration
    for (int i = 0; i < 20; ++i) {
        // Rayleigh pitot formula: qc/p = (166.9216 * M^7) / ((7*M^2 - 1)^2.5) - 1
        double m2 = mach * mach;
        double m7 = std::pow(mach, 7);
        double denom = std::pow(7.0 * m2 - 1.0, 2.5);
        
        if (denom < constants::EPS) {
            return Result<double>::err(ErrorCode::DivisionByZero, mach);
        }
        
        double qc_p_calc = 166.9216 * m7 / denom - 1.0;
        double error = qc_p_calc * delta - qc_p0;
        
        if (std::abs(error) < 1e-10) {
            break;
        }
        
        // Numerical derivative
        double dm = 0.0001;
        double m2p = (mach + dm) * (mach + dm);
        double m7p = std::pow(mach + dm, 7);
        double denomp = std::pow(7.0 * m2p - 1.0, 2.5);
        double qc_p_calc_p = 166.9216 * m7p / denomp - 1.0;
        double deriv = (qc_p_calc_p - qc_p_calc) / dm * delta;
        
        if (std::abs(deriv) > constants::EPS) {
            mach -= error / deriv;
        }
        
        // Clamp to reasonable range
        mach = clamp(mach, 1.0, 10.0);
    }
    
    return Result<double>::ok(mach);
}

// =============================================================================
// TAS/CAS/EAS Conversions
// =============================================================================

/**
 * @brief Calculate TAS from CAS and altitude/temperature
 * 
 * @param cas_kt Calibrated airspeed in knots
 * @param altitude_ft Pressure altitude in feet
 * @param temperature_c Outside air temperature in Celsius (use ISA if unknown)
 * @return True airspeed in knots
 */
[[nodiscard]] inline double tas_from_cas(double cas_kt, 
                                          double altitude_ft,
                                          double temperature_c) noexcept {
    // First convert CAS to Mach
    auto mach_result = mach_from_cas(cas_kt, altitude_ft);
    if (!mach_result) {
        // Try supersonic
        mach_result = mach_from_cas_supersonic(cas_kt, altitude_ft);
        if (!mach_result) {
            // Fall back to approximation
            double sigma = density_ratio_nonstandard(altitude_ft, temperature_c);
            if (sigma > constants::EPS) {
                return cas_kt / std::sqrt(sigma);
            }
            return cas_kt;
        }
    }
    
    // Convert Mach to TAS using actual temperature
    double temperature_k = c_to_k(temperature_c);
    return tas_from_mach(mach_result.value, temperature_k);
}

/**
 * @brief Calculate TAS from CAS using ISA temperature
 * 
 * @param cas_kt Calibrated airspeed in knots
 * @param altitude_ft Pressure altitude in feet
 * @return True airspeed in knots
 */
[[nodiscard]] inline double tas_from_cas_isa(double cas_kt, double altitude_ft) noexcept {
    return tas_from_cas(cas_kt, altitude_ft, isa_temperature_c(altitude_ft));
}

/**
 * @brief Calculate CAS from TAS and altitude/temperature
 * 
 * @param tas_kt True airspeed in knots
 * @param altitude_ft Pressure altitude in feet
 * @param temperature_c Outside air temperature in Celsius
 * @return Calibrated airspeed in knots
 */
[[nodiscard]] inline double cas_from_tas(double tas_kt, 
                                          double altitude_ft,
                                          double temperature_c) noexcept {
    // First convert TAS to Mach
    double temperature_k = c_to_k(temperature_c);
    double mach = mach_from_tas(tas_kt, temperature_k);
    
    // Convert Mach to CAS
    auto cas_result = cas_from_mach(mach, altitude_ft);
    if (cas_result) {
        return cas_result.value;
    }
    
    // Fall back to approximation
    double sigma = density_ratio_nonstandard(altitude_ft, temperature_c);
    return tas_kt * std::sqrt(sigma);
}

/**
 * @brief Calculate EAS from TAS and density ratio
 * 
 * EAS = TAS * sqrt(sigma)
 * 
 * @param tas_kt True airspeed in knots
 * @param sigma Density ratio (rho/rho0)
 * @return Equivalent airspeed in knots
 */
[[nodiscard]] inline double eas_from_tas(double tas_kt, double sigma) noexcept {
    if (sigma <= 0) {
        return 0.0;
    }
    return tas_kt * std::sqrt(sigma);
}

/**
 * @brief Calculate TAS from EAS and density ratio
 * 
 * TAS = EAS / sqrt(sigma)
 * 
 * @param eas_kt Equivalent airspeed in knots
 * @param sigma Density ratio (rho/rho0)
 * @return True airspeed in knots
 */
[[nodiscard]] inline double tas_from_eas(double eas_kt, double sigma) noexcept {
    if (sigma <= constants::EPS) {
        return eas_kt;  // Avoid division by zero
    }
    return eas_kt / std::sqrt(sigma);
}

/**
 * @brief Calculate EAS from CAS (compressibility correction)
 * 
 * At low Mach numbers, EAS ≈ CAS. At higher speeds, compressibility
 * causes CAS to read higher than EAS.
 * 
 * @param cas_kt Calibrated airspeed in knots
 * @param altitude_ft Pressure altitude in feet
 * @return Equivalent airspeed in knots
 */
[[nodiscard]] inline double eas_from_cas(double cas_kt, double altitude_ft) noexcept {
    // Get Mach number
    auto mach_result = mach_from_cas(cas_kt, altitude_ft);
    double mach = mach_result ? mach_result.value : 0.0;
    
    // Compressibility correction factor
    // At low Mach, this approaches 1.0
    double correction = 1.0;
    if (mach > 0.3) {
        // Simplified compressibility correction
        correction = 1.0 - 0.0008 * (mach - 0.3) * (mach - 0.3);
    }
    
    return cas_kt * correction;
}

// =============================================================================
// IAS/CAS Relationship
// =============================================================================

/**
 * @brief Convert IAS to CAS
 * 
 * The difference between IAS and CAS is due to instrument and position errors.
 * Without specific aircraft data, we assume IAS ≈ CAS for this implementation.
 * 
 * In practice, you would apply aircraft-specific corrections here.
 * 
 * @param ias_kt Indicated airspeed in knots
 * @param position_error_kt Position error correction (typically +/- 5 kt)
 * @return Calibrated airspeed in knots
 */
[[nodiscard]] inline double cas_from_ias(double ias_kt, 
                                          double position_error_kt = 0.0) noexcept {
    return ias_kt + position_error_kt;
}

/**
 * @brief Convert CAS to IAS
 * 
 * @param cas_kt Calibrated airspeed in knots
 * @param position_error_kt Position error correction (typically +/- 5 kt)
 * @return Indicated airspeed in knots
 */
[[nodiscard]] inline double ias_from_cas(double cas_kt, 
                                          double position_error_kt = 0.0) noexcept {
    return cas_kt - position_error_kt;
}

// =============================================================================
// Speed Limits
// =============================================================================

/**
 * @brief Calculate VMO/MMO crossover altitude
 * 
 * The altitude at which a constant Mach number equals a constant IAS/CAS.
 * 
 * @param vmo_kt Maximum operating speed in knots
 * @param mmo Maximum operating Mach number
 * @return Crossover altitude in feet
 */
[[nodiscard]] inline Result<double> vmo_mmo_crossover(double vmo_kt, double mmo) noexcept {
    if (vmo_kt <= 0 || mmo <= 0 || mmo >= 1.0) {
        return Result<double>::err(ErrorCode::InvalidSpeed);
    }
    
    // Binary search for altitude where CAS from Mach equals VMO
    double lo = 0.0;
    double hi = 50000.0;
    
    for (int i = 0; i < 50; ++i) {
        double mid = (lo + hi) / 2.0;
        auto cas_result = cas_from_mach(mmo, mid);
        
        if (!cas_result) {
            hi = mid;
            continue;
        }
        
        double cas = cas_result.value;
        
        if (std::abs(cas - vmo_kt) < 0.1) {
            return Result<double>::ok(mid);
        }
        
        if (cas > vmo_kt) {
            lo = mid;  // Need higher altitude to reduce CAS
        } else {
            hi = mid;
        }
    }
    
    return Result<double>::ok((lo + hi) / 2.0);
}

} // namespace aviation

#endif // AVIATION_FORMULARY_PERFORMANCE_AIRSPEED_HPP
