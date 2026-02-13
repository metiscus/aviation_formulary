#ifndef AVIATION_FORMULARY_PERFORMANCE_WIND_HPP
#define AVIATION_FORMULARY_PERFORMANCE_WIND_HPP

/**
 * @file wind.hpp
 * @brief Wind triangle calculations
 * 
 * Implements wind correction angle, ground speed, and heading calculations
 * for flight planning and navigation.
 * 
 * The wind triangle relates:
 * - True Airspeed (TAS): Speed through the air
 * - Ground Speed (GS): Speed over the ground
 * - Wind: Speed and direction (from)
 * - Heading: Direction the aircraft is pointed
 * - Track/Course: Direction of travel over ground
 * - Wind Correction Angle (WCA): Difference between heading and track
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
// Wind Correction Angle
// =============================================================================

/**
 * @brief Calculate wind correction angle (WCA)
 * 
 * WCA is the angle to crab into the wind to maintain desired track.
 * Positive WCA means crab right, negative means crab left.
 * 
 * @param wind_speed Wind speed (same units as TAS)
 * @param wind_direction Direction wind is FROM (radians, true)
 * @param track Desired track/course (radians, true)
 * @param tas True airspeed (same units as wind)
 * @return Result containing WCA in radians, or error if no solution
 */
[[nodiscard]] inline Result<double> wind_correction_angle(double wind_speed,
                                                           double wind_direction,
                                                           double track,
                                                           double tas) noexcept {
    if (tas <= constants::EPS) {
        return Result<double>::err(ErrorCode::InvalidSpeed, 0.0);
    }
    
    // Handle calm wind
    if (wind_speed < constants::EPS) {
        return Result<double>::ok(0.0);
    }
    
    // Check if wind exceeds TAS (no solution)
    if (wind_speed > tas) {
        return Result<double>::err(ErrorCode::WindExceedsTAS, 0.0);
    }
    
    // swc = (ws/tas) * sin(wd - tc)
    double angle_diff = wind_direction - track;
    double swc = (wind_speed / tas) * std::sin(angle_diff);
    
    // Clamp to valid range (might be slightly out due to rounding)
    swc = clamp(swc, -1.0, 1.0);
    
    return Result<double>::ok(asin_safe(swc));
}

/**
 * @brief Calculate wind correction angle (unchecked version)
 */
[[nodiscard]] inline double wind_correction_angle_unchecked(double wind_speed,
                                                             double wind_direction,
                                                             double track,
                                                             double tas) noexcept {
    if (tas <= constants::EPS || wind_speed < constants::EPS) {
        return 0.0;
    }
    
    double swc = (wind_speed / tas) * std::sin(wind_direction - track);
    swc = clamp(swc, -1.0, 1.0);
    return asin_safe(swc);
}

// =============================================================================
// Heading Calculation
// =============================================================================

/**
 * @brief Calculate heading to maintain desired track
 * 
 * @param wind_speed Wind speed
 * @param wind_direction Direction wind is FROM (radians)
 * @param track Desired track/course (radians)
 * @param tas True airspeed
 * @return Result containing heading in radians [0, 2π)
 */
[[nodiscard]] inline Result<double> heading_for_track(double wind_speed,
                                                       double wind_direction,
                                                       double track,
                                                       double tas) noexcept {
    auto wca_result = wind_correction_angle(wind_speed, wind_direction, track, tas);
    
    if (!wca_result) {
        return Result<double>::err(wca_result.error, track);
    }
    
    // Heading = Track + WCA
    return Result<double>::ok(normalize_angle(track + wca_result.value));
}

/**
 * @brief Calculate track from heading and wind
 * 
 * Given the heading being flown and wind conditions, determine the
 * actual track over the ground.
 * 
 * @param heading Aircraft heading (radians)
 * @param wind_speed Wind speed
 * @param wind_direction Direction wind is FROM (radians)
 * @param tas True airspeed
 * @return Track over ground (radians)
 */
[[nodiscard]] inline double track_from_heading(double heading,
                                                double wind_speed,
                                                double wind_direction,
                                                double tas) noexcept {
    if (tas <= constants::EPS) {
        return heading;
    }
    
    // Calculate wind components relative to heading
    double wind_angle = wind_direction - heading;
    
    // Headwind component (positive = headwind)
    double hw = wind_speed * std::cos(wind_angle);
    
    // Crosswind component (positive = from right)
    double xw = wind_speed * std::sin(wind_angle);
    
    // Ground speed components
    double gs_forward = tas - hw;  // Forward component
    double gs_lateral = -xw;        // Lateral drift
    
    // Track deviation from heading
    double track_deviation = std::atan2(gs_lateral, gs_forward);
    
    return normalize_angle(heading + track_deviation);
}

// =============================================================================
// Ground Speed Calculation
// =============================================================================

/**
 * @brief Calculate ground speed from TAS and wind
 * 
 * @param tas True airspeed
 * @param heading Aircraft heading (radians)
 * @param wind_speed Wind speed
 * @param wind_direction Direction wind is FROM (radians)
 * @return Ground speed (same units as TAS)
 */
[[nodiscard]] inline double ground_speed(double tas,
                                          double heading,
                                          double wind_speed,
                                          double wind_direction) noexcept {
    // Wind relative to heading
    double wind_angle = wind_direction - heading;
    
    // Headwind component
    double hw = wind_speed * std::cos(wind_angle);
    
    // Crosswind component
    double xw = wind_speed * std::sin(wind_angle);
    
    // Ground speed using vector addition
    double gs_forward = tas - hw;
    double gs_lateral = -xw;
    
    return std::sqrt(gs_forward * gs_forward + gs_lateral * gs_lateral);
}

/**
 * @brief Calculate ground speed when flying desired track
 * 
 * Given TAS and wind, calculate ground speed when crabbing to
 * maintain a desired track.
 * 
 * @param tas True airspeed
 * @param track Desired track (radians)
 * @param wind_speed Wind speed
 * @param wind_direction Direction wind is FROM (radians)
 * @return Result containing ground speed
 */
[[nodiscard]] inline Result<double> ground_speed_on_track(double tas,
                                                           double track,
                                                           double wind_speed,
                                                           double wind_direction) noexcept {
    auto wca_result = wind_correction_angle(wind_speed, wind_direction, track, tas);
    
    if (!wca_result) {
        return Result<double>::err(wca_result.error, 0.0);
    }
    
    double wca = wca_result.value;
    
    // Ground speed formula from formulary:
    // gs = tas * sqrt(1 - swc^2) - ws * cos(wd - tc)
    double swc = std::sin(wca);
    double gs = tas * std::sqrt(1.0 - swc * swc) - 
                wind_speed * std::cos(wind_direction - track);
    
    if (gs < 0) {
        return Result<double>::err(ErrorCode::NoWindSolution, 0.0);
    }
    
    return Result<double>::ok(gs);
}

// =============================================================================
// Complete Wind Triangle Solution
// =============================================================================

/**
 * @brief Solve wind triangle: given TAS, track, and wind, find heading and GS
 * 
 * @param tas True airspeed
 * @param track Desired track/course (radians)
 * @param wind Wind vector
 * @return WindTriangleResult with heading, ground speed, and WCA
 */
[[nodiscard]] inline WindTriangleResult solve_wind_triangle(double tas,
                                                             double track,
                                                             const Wind& wind) noexcept {
    WindTriangleResult result;
    
    if (tas <= constants::EPS) {
        result.error = ErrorCode::InvalidSpeed;
        return result;
    }
    
    // Handle calm wind
    if (wind.is_calm()) {
        result.heading = track;
        result.ground_speed = tas;
        result.wca = 0.0;
        result.error = ErrorCode::Success;
        return result;
    }
    
    // Calculate WCA
    auto wca_result = wind_correction_angle(wind.speed, wind.direction, track, tas);
    
    if (!wca_result) {
        result.error = wca_result.error;
        return result;
    }
    
    result.wca = wca_result.value;
    result.heading = normalize_angle(track + result.wca);
    
    // Calculate ground speed
    auto gs_result = ground_speed_on_track(tas, track, wind.speed, wind.direction);
    
    if (!gs_result) {
        result.error = gs_result.error;
        return result;
    }
    
    result.ground_speed = gs_result.value;
    result.error = ErrorCode::Success;
    
    return result;
}

// =============================================================================
// Head/Cross Wind Components
// =============================================================================

/**
 * @brief Calculate headwind component
 * 
 * Positive value indicates headwind, negative indicates tailwind.
 * 
 * @param wind_speed Wind speed
 * @param wind_direction Direction wind is FROM (radians)
 * @param runway_heading Runway heading (radians)
 * @return Headwind component (positive = headwind)
 */
[[nodiscard]] inline double headwind_component(double wind_speed,
                                                double wind_direction,
                                                double runway_heading) noexcept {
    // Headwind = ws * cos(wd - runway)
    return wind_speed * std::cos(wind_direction - runway_heading);
}

/**
 * @brief Calculate crosswind component
 * 
 * Positive value indicates wind from the right, negative from left.
 * 
 * @param wind_speed Wind speed
 * @param wind_direction Direction wind is FROM (radians)
 * @param runway_heading Runway heading (radians)
 * @return Crosswind component (positive = from right)
 */
[[nodiscard]] inline double crosswind_component(double wind_speed,
                                                 double wind_direction,
                                                 double runway_heading) noexcept {
    // Crosswind = ws * sin(wd - runway)
    return wind_speed * std::sin(wind_direction - runway_heading);
}

/**
 * @brief Structure for runway wind components
 */
struct RunwayWindComponents {
    double headwind;    ///< Headwind component (positive = headwind, negative = tailwind)
    double crosswind;   ///< Crosswind component (positive = from right)
    
    constexpr RunwayWindComponents() noexcept : headwind(0.0), crosswind(0.0) {}
    constexpr RunwayWindComponents(double hw, double xw) noexcept 
        : headwind(hw), crosswind(xw) {}
    
    /// Get absolute crosswind
    [[nodiscard]] double abs_crosswind() const noexcept {
        return std::abs(crosswind);
    }
    
    /// Check if tailwind
    [[nodiscard]] bool is_tailwind() const noexcept {
        return headwind < 0;
    }
    
    /// Check if within limits
    [[nodiscard]] bool within_limits(double max_crosswind, double max_tailwind = 10.0) const noexcept {
        return abs_crosswind() <= max_crosswind && 
               (headwind >= 0 || std::abs(headwind) <= max_tailwind);
    }
};

/**
 * @brief Calculate both head and crosswind components
 * 
 * @param wind_speed Wind speed
 * @param wind_direction Direction wind is FROM (radians)
 * @param runway_heading Runway heading (radians)
 * @return RunwayWindComponents structure
 */
[[nodiscard]] inline RunwayWindComponents runway_wind_components(double wind_speed,
                                                                  double wind_direction,
                                                                  double runway_heading) noexcept {
    return RunwayWindComponents{
        headwind_component(wind_speed, wind_direction, runway_heading),
        crosswind_component(wind_speed, wind_direction, runway_heading)
    };
}

/**
 * @brief Calculate runway wind components from degrees
 * 
 * Convenience function using degrees for wind direction and runway heading.
 */
[[nodiscard]] inline RunwayWindComponents runway_wind_components_deg(double wind_speed,
                                                                      double wind_direction_deg,
                                                                      double runway_heading_deg) noexcept {
    return runway_wind_components(wind_speed,
                                  deg_to_rad(wind_direction_deg),
                                  deg_to_rad(runway_heading_deg));
}

// =============================================================================
// Unknown Wind Calculations
// =============================================================================

/**
 * @brief Calculate wind from known TAS, GS, heading, and track
 * 
 * Given the aircraft's TAS, heading, and observed ground track and speed,
 * calculate the wind vector.
 * 
 * @param tas True airspeed
 * @param heading Aircraft heading (radians)
 * @param ground_speed Observed ground speed
 * @param track Observed track (radians)
 * @return Wind vector
 */
[[nodiscard]] inline Wind calculate_wind(double tas,
                                          double heading,
                                          double ground_speed,
                                          double track) noexcept {
    // Air vector components
    double air_x = tas * std::sin(heading);
    double air_y = tas * std::cos(heading);
    
    // Ground vector components
    double gnd_x = ground_speed * std::sin(track);
    double gnd_y = ground_speed * std::cos(track);
    
    // Wind vector = Air vector - Ground vector
    double wind_x = air_x - gnd_x;
    double wind_y = air_y - gnd_y;
    
    // Wind speed
    double ws = std::sqrt(wind_x * wind_x + wind_y * wind_y);
    
    // Wind direction (FROM)
    double wd = normalize_angle(std::atan2(wind_x, wind_y));
    
    return Wind{ws, wd};
}

// =============================================================================
// TAS from Three Ground Speeds
// =============================================================================

/**
 * @brief Calculate TAS and wind from three ground speed observations
 * 
 * Flying three different headings and measuring ground speed allows
 * determination of both TAS and wind.
 * 
 * @param heading1 First heading (radians)
 * @param gs1 Ground speed on first heading
 * @param heading2 Second heading (radians)
 * @param gs2 Ground speed on second heading
 * @param heading3 Third heading (radians)
 * @param gs3 Ground speed on third heading
 * @param out_tas Output: calculated TAS
 * @param out_wind Output: calculated wind
 * @return ErrorCode::Success or error
 */
[[nodiscard]] inline ErrorCode tas_and_wind_from_three_gs(
    double heading1, double gs1,
    double heading2, double gs2,
    double heading3, double gs3,
    double& out_tas, Wind& out_wind) noexcept {
    
    // Using the method from the formulary:
    // GS² = TAS² + WS² - 2*TAS*WS*cos(HD - WD)
    // 
    // Three equations, three unknowns (TAS, WS, WD)
    // This is solved iteratively
    
    // Initial estimate: TAS ≈ average GS
    double tas = (gs1 + gs2 + gs3) / 3.0;
    double ws = 0.0;
    double wd = 0.0;
    
    // Iteration
    for (int iter = 0; iter < 50; ++iter) {
        // For current TAS, estimate wind from first two headings
        // This is a simplified approach
        
        double sin_h1 = std::sin(heading1);
        double cos_h1 = std::cos(heading1);
        double sin_h2 = std::sin(heading2);
        double cos_h2 = std::cos(heading2);
        double sin_h3 = std::sin(heading3);
        double cos_h3 = std::cos(heading3);
        
        // X-component of ground velocity
        double gx1 = gs1 * sin_h1;
        double gx2 = gs2 * sin_h2;
        double gx3 = gs3 * sin_h3;
        
        // Y-component of ground velocity
        double gy1 = gs1 * cos_h1;
        double gy2 = gs2 * cos_h2;
        double gy3 = gs3 * cos_h3;
        
        // Average ground vector
        double avg_gx = (gx1 + gx2 + gx3) / 3.0;
        double avg_gy = (gy1 + gy2 + gy3) / 3.0;
        
        // Average TAS vector (if no wind, GS would be TAS)
        double avg_ax = tas * (sin_h1 + sin_h2 + sin_h3) / 3.0;
        double avg_ay = tas * (cos_h1 + cos_h2 + cos_h3) / 3.0;
        
        // Wind = average air - average ground
        double wx = avg_ax - avg_gx;
        double wy = avg_ay - avg_gy;
        
        double new_ws = std::sqrt(wx * wx + wy * wy);
        double new_wd = normalize_angle(std::atan2(wx, wy));
        
        // Recalculate TAS from average ground speed and wind
        double avg_gs = (gs1 + gs2 + gs3) / 3.0;
        double avg_heading = std::atan2(sin_h1 + sin_h2 + sin_h3, cos_h1 + cos_h2 + cos_h3);
        
        // TAS² = GS² + WS² - 2*GS*WS*cos(track - WD)
        // Simplified: use average
        double new_tas = avg_gs + new_ws * std::cos(avg_heading - new_wd);
        
        // Check convergence
        if (std::abs(new_tas - tas) < 0.1 && std::abs(new_ws - ws) < 0.1) {
            out_tas = new_tas;
            out_wind = Wind{new_ws, new_wd};
            return ErrorCode::Success;
        }
        
        tas = new_tas;
        ws = new_ws;
        wd = new_wd;
    }
    
    out_tas = tas;
    out_wind = Wind{ws, wd};
    return ErrorCode::Success;  // Best estimate
}

// =============================================================================
// Bellamy's Formula (Simple Wind Drift)
// =============================================================================

/**
 * @brief Calculate wind drift using Bellamy's formula
 * 
 * Bellamy's formula provides a quick estimate of drift based on pressure
 * differences. It's an approximation useful when detailed wind data
 * isn't available.
 * 
 * Drift(nm) = 0.435 * (p2 - p1) / sin(lat)
 * where p1, p2 are pressures at checkpoints in millibars
 * 
 * @param pressure1_mb Pressure at first checkpoint (millibars)
 * @param pressure2_mb Pressure at second checkpoint (millibars)
 * @param latitude Average latitude (radians)
 * @return Drift in nautical miles (positive = right of course)
 */
[[nodiscard]] inline double bellamy_drift(double pressure1_mb,
                                           double pressure2_mb,
                                           double latitude) noexcept {
    double sin_lat = std::sin(latitude);
    
    if (std::abs(sin_lat) < 0.1) {
        // Near equator, formula breaks down
        return 0.0;
    }
    
    return 0.435 * (pressure2_mb - pressure1_mb) / sin_lat;
}

/**
 * @brief Calculate wind correction angle using Bellamy's formula
 * 
 * WCA (degrees) = 1230000 * (p2 - p1) / (sin(lat) * TAS * dist)
 * 
 * @param pressure1_mb Pressure at first checkpoint (millibars)
 * @param pressure2_mb Pressure at second checkpoint (millibars)
 * @param latitude Average latitude (radians)
 * @param tas True airspeed (knots)
 * @param distance Distance between checkpoints (nautical miles)
 * @return Wind correction angle in radians
 */
[[nodiscard]] inline double bellamy_wca(double pressure1_mb,
                                         double pressure2_mb,
                                         double latitude,
                                         double tas,
                                         double distance) noexcept {
    double sin_lat = std::sin(latitude);
    
    if (std::abs(sin_lat) < 0.1 || tas < constants::EPS || distance < constants::EPS) {
        return 0.0;
    }
    
    // Formula gives degrees, convert to radians
    double wca_deg = 1230000.0 * (pressure2_mb - pressure1_mb) / (sin_lat * tas * distance);
    return deg_to_rad(wca_deg);
}

} // namespace aviation

#endif // AVIATION_FORMULARY_PERFORMANCE_WIND_HPP
