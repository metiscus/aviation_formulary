#ifndef AVIATION_FORMULARY_PERFORMANCE_TURNS_HPP
#define AVIATION_FORMULARY_PERFORMANCE_TURNS_HPP

/**
 * @file turns.hpp
 * @brief Turn calculations including radius, rate, and pivotal altitude
 * 
 * Implements calculations for:
 * - Turn radius
 * - Rate of turn
 * - Bank angle for a given rate of turn
 * - Load factor (G-force) in turns
 * - Pivotal altitude for ground reference maneuvers
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
// Turn Radius
// =============================================================================

/**
 * @brief Calculate turn radius for a given speed and bank angle
 * 
 * R = V² / (g * tan(bank))
 * 
 * @param speed_kt Speed in knots (TAS or GS depending on context)
 * @param bank_angle_rad Bank angle in radians
 * @return Turn radius in feet
 */
[[nodiscard]] inline Result<double> turn_radius_ft(double speed_kt, 
                                                    double bank_angle_rad) noexcept {
    if (speed_kt < 0) {
        return Result<double>::err(ErrorCode::InvalidSpeed);
    }
    
    // Clamp bank angle to valid range (0, 90 degrees)
    if (bank_angle_rad <= 0 || bank_angle_rad >= constants::HALF_PI) {
        return Result<double>::err(ErrorCode::InvalidBankAngle);
    }
    
    double tan_bank = std::tan(bank_angle_rad);
    
    if (tan_bank < constants::EPS) {
        return Result<double>::err(ErrorCode::InvalidBankAngle);
    }
    
    // R = V² / (11.26 * tan(bank))
    // where 11.26 is g in ft/s² / conversion factor for knots
    // More precisely: R_ft = (V_kt * 1.68781)² / (32.174 * tan(bank))
    // Simplified: R_ft = V_kt² / (11.26 * tan(bank))
    
    double radius = (speed_kt * speed_kt) / (constants::TURN_RADIUS_COEFF_FT * tan_bank);
    
    return Result<double>::ok(radius);
}

/**
 * @brief Calculate turn radius in nautical miles
 * 
 * @param speed_kt Speed in knots
 * @param bank_angle_rad Bank angle in radians
 * @return Turn radius in nautical miles
 */
[[nodiscard]] inline Result<double> turn_radius_nm(double speed_kt, 
                                                    double bank_angle_rad) noexcept {
    auto result = turn_radius_ft(speed_kt, bank_angle_rad);
    if (!result) {
        return result;
    }
    
    // Convert feet to nautical miles
    return Result<double>::ok(result.value / 6076.12);
}

/**
 * @brief Calculate turn diameter in feet
 */
[[nodiscard]] inline Result<double> turn_diameter_ft(double speed_kt, 
                                                      double bank_angle_rad) noexcept {
    auto result = turn_radius_ft(speed_kt, bank_angle_rad);
    if (!result) {
        return result;
    }
    return Result<double>::ok(result.value * 2.0);
}

// =============================================================================
// Rate of Turn
// =============================================================================

/**
 * @brief Calculate rate of turn for a given speed and bank angle
 * 
 * Rate = 1091 * tan(bank) / V
 * 
 * @param speed_kt Speed in knots
 * @param bank_angle_rad Bank angle in radians
 * @return Rate of turn in degrees per second
 */
[[nodiscard]] inline Result<double> turn_rate_deg_per_sec(double speed_kt, 
                                                           double bank_angle_rad) noexcept {
    if (speed_kt <= constants::EPS) {
        return Result<double>::err(ErrorCode::InvalidSpeed);
    }
    
    if (bank_angle_rad <= 0 || bank_angle_rad >= constants::HALF_PI) {
        return Result<double>::err(ErrorCode::InvalidBankAngle);
    }
    
    double tan_bank = std::tan(bank_angle_rad);
    
    // Rate (deg/s) = 1091 * tan(bank) / V_kt
    double rate = constants::TURN_RATE_COEFF * tan_bank / speed_kt;
    
    return Result<double>::ok(rate);
}

/**
 * @brief Calculate rate of turn in radians per second
 */
[[nodiscard]] inline Result<double> turn_rate_rad_per_sec(double speed_kt, 
                                                           double bank_angle_rad) noexcept {
    auto result = turn_rate_deg_per_sec(speed_kt, bank_angle_rad);
    if (!result) {
        return result;
    }
    return Result<double>::ok(deg_to_rad(result.value));
}

// =============================================================================
// Bank Angle Calculations
// =============================================================================

/**
 * @brief Calculate bank angle required for a given rate of turn
 * 
 * bank = atan(rate * V / 1091)
 * 
 * @param speed_kt Speed in knots
 * @param rate_deg_per_sec Desired rate of turn in degrees per second
 * @return Bank angle in radians
 */
[[nodiscard]] inline Result<double> bank_angle_for_rate(double speed_kt, 
                                                         double rate_deg_per_sec) noexcept {
    if (speed_kt <= constants::EPS) {
        return Result<double>::err(ErrorCode::InvalidSpeed);
    }
    
    if (rate_deg_per_sec <= 0) {
        return Result<double>::ok(0.0);
    }
    
    // tan(bank) = rate * V / 1091
    double tan_bank = rate_deg_per_sec * speed_kt / constants::TURN_RATE_COEFF;
    
    // Check for unreasonable bank angle
    if (tan_bank > 5.67) {  // > 80 degrees
        return Result<double>::err(ErrorCode::OutOfRange);
    }
    
    return Result<double>::ok(std::atan(tan_bank));
}

/**
 * @brief Calculate bank angle for standard rate turn (3°/sec)
 * 
 * @param speed_kt Speed in knots
 * @return Bank angle in radians for a standard rate turn
 */
[[nodiscard]] inline Result<double> standard_rate_bank_angle(double speed_kt) noexcept {
    return bank_angle_for_rate(speed_kt, constants::STANDARD_RATE_DEG_PER_SEC);
}

/**
 * @brief Calculate bank angle required for a given turn radius
 * 
 * bank = atan(V² / (g * R))
 * 
 * @param speed_kt Speed in knots
 * @param radius_ft Turn radius in feet
 * @return Bank angle in radians
 */
[[nodiscard]] inline Result<double> bank_angle_for_radius(double speed_kt, 
                                                           double radius_ft) noexcept {
    if (speed_kt <= constants::EPS) {
        return Result<double>::err(ErrorCode::InvalidSpeed);
    }
    
    if (radius_ft <= constants::EPS) {
        return Result<double>::err(ErrorCode::OutOfRange);
    }
    
    // tan(bank) = V² / (11.26 * R)
    double tan_bank = (speed_kt * speed_kt) / (constants::TURN_RADIUS_COEFF_FT * radius_ft);
    
    // Check for unreasonable bank angle
    if (tan_bank > 5.67) {  // > 80 degrees
        return Result<double>::err(ErrorCode::OutOfRange);
    }
    
    return Result<double>::ok(std::atan(tan_bank));
}

// =============================================================================
// Load Factor
// =============================================================================

/**
 * @brief Calculate load factor (G-force) in a coordinated turn
 * 
 * n = 1 / cos(bank)
 * 
 * @param bank_angle_rad Bank angle in radians
 * @return Load factor (G's)
 */
[[nodiscard]] inline Result<double> load_factor(double bank_angle_rad) noexcept {
    if (bank_angle_rad < 0 || bank_angle_rad >= constants::HALF_PI) {
        return Result<double>::err(ErrorCode::InvalidBankAngle);
    }
    
    double cos_bank = std::cos(bank_angle_rad);
    
    if (cos_bank < constants::EPS) {
        return Result<double>::err(ErrorCode::DivisionByZero);
    }
    
    return Result<double>::ok(1.0 / cos_bank);
}

/**
 * @brief Calculate bank angle that produces a given load factor
 * 
 * bank = acos(1/n)
 * 
 * @param n Load factor (must be >= 1)
 * @return Bank angle in radians
 */
[[nodiscard]] inline Result<double> bank_angle_for_load_factor(double n) noexcept {
    if (n < 1.0) {
        return Result<double>::err(ErrorCode::OutOfRange);
    }
    
    return Result<double>::ok(acos_safe(1.0 / n));
}

/**
 * @brief Calculate stall speed multiplier in a turn
 * 
 * Stall speed increases by sqrt(n) in a turn.
 * 
 * @param bank_angle_rad Bank angle in radians
 * @return Multiplier for stall speed (>= 1.0)
 */
[[nodiscard]] inline double stall_speed_multiplier(double bank_angle_rad) noexcept {
    auto n_result = load_factor(bank_angle_rad);
    if (!n_result) {
        return 1.0;
    }
    return std::sqrt(n_result.value);
}

// =============================================================================
// Time to Complete Turn
// =============================================================================

/**
 * @brief Calculate time to complete a turn of given angle
 * 
 * @param turn_angle_deg Turn angle in degrees
 * @param rate_deg_per_sec Rate of turn in degrees per second
 * @return Time in seconds
 */
[[nodiscard]] inline double time_for_turn(double turn_angle_deg, 
                                           double rate_deg_per_sec) noexcept {
    if (rate_deg_per_sec <= constants::EPS) {
        return 0.0;  // Infinite time
    }
    return std::abs(turn_angle_deg) / rate_deg_per_sec;
}

/**
 * @brief Calculate time for a 360° turn
 * 
 * @param speed_kt Speed in knots
 * @param bank_angle_rad Bank angle in radians
 * @return Time in seconds for a complete circle
 */
[[nodiscard]] inline Result<double> time_for_360(double speed_kt, 
                                                  double bank_angle_rad) noexcept {
    auto rate_result = turn_rate_deg_per_sec(speed_kt, bank_angle_rad);
    if (!rate_result) {
        return Result<double>::err(rate_result.error);
    }
    
    return Result<double>::ok(360.0 / rate_result.value);
}

// =============================================================================
// Pivotal Altitude
// =============================================================================

/**
 * @brief Calculate pivotal altitude for ground reference maneuvers
 * 
 * Pivotal altitude is the altitude at which, during a turn around a point,
 * the line of sight to the point remains constant. This is used for
 * turns around a point and eights on pylons.
 * 
 * PA = GS² / 11.26
 * 
 * @param ground_speed_kt Ground speed in knots
 * @return Pivotal altitude in feet AGL
 */
[[nodiscard]] inline double pivotal_altitude(double ground_speed_kt) noexcept {
    // PA = GS² / 11.26
    return (ground_speed_kt * ground_speed_kt) / constants::TURN_RADIUS_COEFF_FT;
}

/**
 * @brief Calculate ground speed for a desired pivotal altitude
 * 
 * @param altitude_ft Desired pivotal altitude in feet AGL
 * @return Required ground speed in knots
 */
[[nodiscard]] inline double ground_speed_for_pivotal_altitude(double altitude_ft) noexcept {
    if (altitude_ft <= 0) {
        return 0.0;
    }
    // GS = sqrt(PA * 11.26)
    return std::sqrt(altitude_ft * constants::TURN_RADIUS_COEFF_FT);
}

// =============================================================================
// Angular Velocity
// =============================================================================

/**
 * @brief Calculate angular velocity in a turn
 * 
 * @param speed_kt Speed in knots
 * @param bank_angle_rad Bank angle in radians
 * @return Angular velocity in radians per second
 */
[[nodiscard]] inline Result<double> angular_velocity(double speed_kt, 
                                                      double bank_angle_rad) noexcept {
    return turn_rate_rad_per_sec(speed_kt, bank_angle_rad);
}

// =============================================================================
// Turn Coordination
// =============================================================================

/**
 * @brief Check if a turn is coordinated (slip/skid indicator centered)
 * 
 * A turn is coordinated when the bank angle matches the rate of turn.
 * This function returns the slip/skid amount.
 * 
 * @param actual_bank_rad Actual bank angle in radians
 * @param actual_rate_deg_per_sec Actual rate of turn in degrees per second
 * @param speed_kt Speed in knots
 * @return Slip/skid amount (0 = coordinated, positive = skidding, negative = slipping)
 */
[[nodiscard]] inline double turn_coordination(double actual_bank_rad,
                                               double actual_rate_deg_per_sec,
                                               double speed_kt) noexcept {
    // Calculate the bank angle that would be needed for the actual rate
    auto required_bank = bank_angle_for_rate(speed_kt, actual_rate_deg_per_sec);
    
    if (!required_bank) {
        return 0.0;
    }
    
    // Positive means over-banked (skidding), negative means under-banked (slipping)
    return actual_bank_rad - required_bank.value;
}

// =============================================================================
// Holding Pattern Calculations
// =============================================================================

/**
 * @brief Calculate holding pattern dimensions
 * 
 * Returns the approximate length and width of a standard holding pattern.
 */
struct HoldingPatternDimensions {
    double length_nm;       ///< Length of the racetrack (nm)
    double width_nm;        ///< Width of the pattern (nm)
    double turn_radius_nm;  ///< Radius of the turns (nm)
    double total_time_sec;  ///< Total time for one circuit (seconds)
    
    constexpr HoldingPatternDimensions() noexcept 
        : length_nm(0), width_nm(0), turn_radius_nm(0), total_time_sec(0) {}
};

/**
 * @brief Calculate holding pattern dimensions
 * 
 * @param speed_kt Holding speed in knots (TAS or GS)
 * @param bank_angle_rad Bank angle for turns (radians)
 * @param outbound_time_sec Outbound leg time (typically 60 or 90 seconds)
 * @return HoldingPatternDimensions structure
 */
[[nodiscard]] inline HoldingPatternDimensions holding_pattern_dimensions(
    double speed_kt,
    double bank_angle_rad,
    double outbound_time_sec = 60.0) noexcept {
    
    HoldingPatternDimensions result;
    
    auto radius_result = turn_radius_nm(speed_kt, bank_angle_rad);
    if (!radius_result) {
        return result;
    }
    
    result.turn_radius_nm = radius_result.value;
    
    // Width is the turn diameter
    result.width_nm = 2.0 * result.turn_radius_nm;
    
    // Leg length in nm (speed in nm/hour, time in seconds)
    double leg_length_nm = (speed_kt / 3600.0) * outbound_time_sec;
    
    // Total length = 2 * leg length + turn diameter (approximately)
    result.length_nm = 2.0 * leg_length_nm + result.width_nm;
    
    // Time for 360 degrees of turn
    auto turn_time_result = time_for_360(speed_kt, bank_angle_rad);
    double turn_time = turn_time_result ? turn_time_result.value : 0.0;
    
    // Total time = 2 legs + 2 turns (180 degrees each)
    result.total_time_sec = 2.0 * outbound_time_sec + turn_time;
    
    return result;
}

} // namespace aviation

#endif // AVIATION_FORMULARY_PERFORMANCE_TURNS_HPP
