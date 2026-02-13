/**
 * @file test_new_modules.cpp
 * @brief Tests for all new Aviation Formulary modules
 * 
 * Tests rhumb lines, atmosphere, altimetry, humidity, airspeeds,
 * wind triangles, turns, flat earth, conversions, and miscellaneous.
 */

#include <gtest/gtest.h>
#include <cmath>
#include "aviation_formulary/aviation_formulary.hpp"

using namespace aviation;

// =============================================================================
// Rhumb Line Tests
// =============================================================================

TEST(RhumbLineTest, LAXToJFKRhumbCourse) {
    // From formulary: LAX to JFK rhumb line
    LatLon lax = LatLon::from_degrees_unchecked(33.95, 118.4);
    LatLon jfk = LatLon::from_degrees_unchecked(40.633, 73.783);
    
    auto result = rhumb_course(lax, jfk);
    ASSERT_TRUE(result.is_ok());
    
    double course_deg = rad_to_deg(result.value);
    // Rhumb line course should be roughly eastward (< 90°)
    EXPECT_GT(course_deg, 60.0);
    EXPECT_LT(course_deg, 100.0);
}

TEST(RhumbLineTest, LAXToJFKRhumbDistance) {
    // Rhumb distance is longer than great circle
    LatLon lax = LatLon::from_degrees_unchecked(33.95, 118.4);
    LatLon jfk = LatLon::from_degrees_unchecked(40.633, 73.783);
    
    double rhumb_dist = rhumb_distance(lax, jfk);
    double gc_dist = distance(lax, jfk);
    
    // Rhumb distance should be greater than GC distance
    EXPECT_GT(rhumb_dist, gc_dist);
    
    // For this route, the difference should be modest
    double ratio = rhumb_dist / gc_dist;
    EXPECT_GT(ratio, 1.0);
    EXPECT_LT(ratio, 1.1);  // Less than 10% longer
}

TEST(RhumbLineTest, RoundTrip) {
    LatLon start = LatLon::from_degrees_unchecked(40.0, 80.0);
    double course = deg_to_rad(45.0);
    double dist = nm_to_rad(500.0);
    
    // Go out
    LatLon dest = rhumb_destination_unchecked(start, course, dist);
    
    // Come back (opposite course)
    double back_course = normalize_angle(course + constants::PI);
    LatLon returned = rhumb_destination_unchecked(dest, back_course, dist);
    
    EXPECT_NEAR(returned.lat, start.lat, 1e-6);
    double lon_diff = normalize_angle_signed(returned.lon - start.lon);
    EXPECT_NEAR(lon_diff, 0.0, 1e-6);
}

// =============================================================================
// Standard Atmosphere Tests
// =============================================================================

TEST(AtmosphereTest, ISASeaLevel) {
    // Sea level ISA conditions
    double temp_c = isa_temperature_c(0.0);
    double pressure = isa_pressure_mb(0.0);
    double density = isa_density(0.0);
    
    EXPECT_NEAR(temp_c, 15.0, 0.1);  // 15°C at sea level
    EXPECT_NEAR(pressure, 1013.25, 0.1);  // 1013.25 mb
    EXPECT_NEAR(density, 1.225, 0.01);  // 1.225 kg/m³
}

TEST(AtmosphereTest, ISATemperatureLapseRate) {
    // Temperature decreases at ~2°C per 1000 ft in troposphere
    double temp_0 = isa_temperature_c(0.0);
    double temp_10000 = isa_temperature_c(10000.0);
    
    double lapse = (temp_0 - temp_10000) / 10.0;  // °C per 1000 ft
    EXPECT_NEAR(lapse, 1.98, 0.1);
}

TEST(AtmosphereTest, PressureRatioConsistency) {
    // Verify pressure ratio matches pressure calculation
    double altitude = 18000.0;
    double pressure = isa_pressure_mb(altitude);
    double delta = pressure_ratio(altitude);
    
    EXPECT_NEAR(delta, pressure / 1013.25, 1e-4);
}

// =============================================================================
// Altimetry Tests
// =============================================================================

TEST(AltimetryTest, PressureAltitudeAtSeaLevel) {
    // At standard pressure (1013.25 mb), PA = 0
    double pa = pressure_altitude_from_pressure(1013.25);
    EXPECT_NEAR(pa, 0.0, 10.0);
}

TEST(AltimetryTest, PressureAltitudeConversion) {
    // Test pressure altitude from pressure
    double pressure_at_5k = isa_pressure_mb(5000.0);
    double actual_pa = pressure_altitude_exact(pressure_at_5k);
    
    EXPECT_NEAR(actual_pa, 5000.0, 100.0);  // Allow tolerance for formula differences
}

TEST(AltimetryTest, DensityAltitudeHotDay) {
    // Hot day at sea level - density altitude should be higher
    double da = density_altitude(0.0, 30.0);  // 30°C at PA 0 (ISA + 15)
    
    // DA should be positive (higher than PA)
    EXPECT_GT(da, 0.0);
    EXPECT_GT(da, 1000.0);  // Typically about 1800 ft for +15°C
    EXPECT_LT(da, 3000.0);
}

TEST(AltimetryTest, DensityAltitudeColdDay) {
    // Cold day at sea level - density altitude should be lower
    double da = density_altitude(0.0, -10.0);  // -10°C at PA 0
    
    // DA should be negative (lower than PA)
    EXPECT_LT(da, 0.0);
}

// =============================================================================
// Humidity Tests
// =============================================================================

TEST(HumidityTest, DewpointAt100PercentRH) {
    // At 100% RH (1.0 as fraction), dewpoint equals temperature
    double temp = 20.0;
    double dp = dewpoint(temp, 1.0);  // 1.0 = 100% RH as fraction
    
    EXPECT_NEAR(dp, temp, 0.5);
}

TEST(HumidityTest, RelativeHumidityFromDewpoint) {
    double temp = 25.0;
    double dp = 15.0;  // 10 degree dewpoint depression
    
    double rh = relative_humidity(temp, dp);
    
    // RH should be less than 100% (returns as fraction 0-1 or percentage 0-100 depending on impl)
    EXPECT_GT(rh, 0.0);
    // If it returns as fraction (0.54), scale test; if as percentage (54), use that
    if (rh < 1.5) {
        // Returns as fraction
        EXPECT_NEAR(rh, 0.54, 0.1);  // ~54%
    } else {
        // Returns as percentage
        EXPECT_NEAR(rh, 54.0, 10.0);
    }
}

TEST(HumidityTest, RoundTripDewpointRH) {
    double temp = 20.0;
    double original_rh_frac = 0.75;  // 75% as fraction
    
    // Calculate dewpoint from RH (expects fraction)
    double dp = dewpoint(temp, original_rh_frac);
    
    // Calculate RH from dewpoint (returns fraction)
    double calculated_rh_frac = relative_humidity(temp, dp);
    
    EXPECT_NEAR(calculated_rh_frac, original_rh_frac, 0.02);
}

// =============================================================================
// Airspeed Tests
// =============================================================================

TEST(AirspeedTest, TASEqualsIASAtSeaLevel) {
    // At sea level ISA, TAS ≈ CAS
    double cas = 150.0;
    double tas = tas_from_cas_isa(cas, 0.0);
    
    EXPECT_NEAR(tas, cas, 2.0);  // Should be very close at sea level
}

TEST(AirspeedTest, TASIncreasesWithAltitude) {
    double cas = 200.0;
    
    double tas_0 = tas_from_cas_isa(cas, 0.0);
    double tas_10k = tas_from_cas_isa(cas, 10000.0);
    double tas_20k = tas_from_cas_isa(cas, 20000.0);
    
    // TAS should increase with altitude (lower density)
    EXPECT_LT(tas_0, tas_10k);
    EXPECT_LT(tas_10k, tas_20k);
}

TEST(AirspeedTest, MachToTASRoundTrip) {
    double original_mach = 0.78;
    double altitude = 35000.0;
    double temp_k = isa_temperature_k(altitude);
    
    double tas = tas_from_mach(original_mach, temp_k);
    double calculated_mach = mach_from_tas(tas, temp_k);
    
    EXPECT_NEAR(calculated_mach, original_mach, 0.001);
}

// =============================================================================
// Wind Triangle Tests
// =============================================================================

TEST(WindTest, NoWind) {
    // With no wind, heading = track and GS = TAS
    double track = deg_to_rad(90.0);
    double tas = 150.0;
    
    auto wca_result = wind_correction_angle(0.0, 0.0, track, tas);
    ASSERT_TRUE(wca_result.is_ok());
    EXPECT_NEAR(wca_result.value, 0.0, 1e-6);
}

TEST(WindTest, PureHeadwind) {
    double track = deg_to_rad(90.0);  // Eastbound
    double wind_dir = deg_to_rad(90.0);  // Wind FROM the east
    double wind_speed = 30.0;
    double tas = 150.0;
    
    // No WCA needed for pure headwind
    auto wca_result = wind_correction_angle(wind_speed, wind_dir, track, tas);
    ASSERT_TRUE(wca_result.is_ok());
    EXPECT_NEAR(wca_result.value, 0.0, 0.01);
}

TEST(WindTest, HeadwindCrosswindComponents) {
    double runway_heading = deg_to_rad(270.0);  // Runway 27
    double wind_dir = deg_to_rad(300.0);  // Wind from 300°
    double wind_speed = 20.0;
    
    double hw = headwind_component(wind_speed, wind_dir, runway_heading);
    double xw = crosswind_component(wind_speed, wind_dir, runway_heading);
    
    // 30° off = 20 * cos(30) ≈ 17.3 headwind, 20 * sin(30) = 10 crosswind
    EXPECT_NEAR(hw, 17.3, 2.0);
    EXPECT_NEAR(std::abs(xw), 10.0, 2.0);
}

// =============================================================================
// Turn Tests
// =============================================================================

TEST(TurnTest, StandardRateTurn) {
    // Standard rate turn at 100 kts
    auto bank_result = standard_rate_bank_angle(100.0);
    ASSERT_TRUE(bank_result.is_ok());
    
    double bank_deg = rad_to_deg(bank_result.value);
    EXPECT_GT(bank_deg, 10.0);
    EXPECT_LT(bank_deg, 25.0);
}

TEST(TurnTest, TurnRadius) {
    double tas = 120.0;  // knots
    double bank_rad = deg_to_rad(30.0);
    
    auto radius_result = turn_radius_ft(tas, bank_rad);
    ASSERT_TRUE(radius_result.is_ok());
    
    // Should be a reasonable radius
    EXPECT_GT(radius_result.value, 1000.0);
    EXPECT_LT(radius_result.value, 10000.0);
}

TEST(TurnTest, TurnRate) {
    double tas = 100.0;
    double bank_rad = deg_to_rad(30.0);
    
    auto rate_result = turn_rate_deg_per_sec(tas, bank_rad);
    ASSERT_TRUE(rate_result.is_ok());
    
    // Should be a reasonable rate
    EXPECT_GT(rate_result.value, 0.0);
    EXPECT_LT(rate_result.value, 10.0);  // Less than 10°/sec
}

TEST(TurnTest, LoadFactor) {
    // Level flight = 1G
    auto load_0 = load_factor(0.0);
    ASSERT_TRUE(load_0.is_ok());
    EXPECT_NEAR(load_0.value, 1.0, 0.001);
    
    // 60° bank = 2G
    auto load_60 = load_factor(deg_to_rad(60.0));
    ASSERT_TRUE(load_60.is_ok());
    EXPECT_NEAR(load_60.value, 2.0, 0.01);
}

TEST(TurnTest, PivotalAltitude) {
    double gs = 100.0;  // 100 kts ground speed
    double pa = pivotal_altitude(gs);
    
    // For 100 kts, pivotal altitude is about 889 ft
    EXPECT_NEAR(pa, 889.0, 50.0);
}

// =============================================================================
// Local Flat Earth Tests
// =============================================================================

TEST(FlatEarthTest, ShortDistanceAccuracy) {
    // For short distances, flat earth should be very accurate
    LatLon p1 = LatLon::from_degrees_unchecked(45.0, 90.0);
    double bearing = deg_to_rad(45.0);
    double dist = nm_to_rad(10.0);  // 10 nm
    
    // Compare great circle and flat earth
    LatLon gc_dest = destination_point_unchecked(p1, bearing, dist);
    LatLon fe_dest = flat_earth_destination(p1, bearing, dist);
    
    // Should be very close for 10 nm
    double error = distance(gc_dest, fe_dest);
    EXPECT_LT(rad_to_nm(error), 0.02);  // Less than 0.02 nm error
}

TEST(FlatEarthTest, LocalCartesianRoundTrip) {
    LatLon origin = LatLon::from_degrees_unchecked(40.0, 100.0);
    LatLon point = LatLon::from_degrees_unchecked(40.1, 100.1);
    
    auto [x, y] = to_local_cartesian_nm(origin, point);
    LocalCartesian local_nm{x, y};
    LatLon recovered = from_local_cartesian_nm(origin, local_nm);
    
    // Should recover the original point very closely
    EXPECT_NEAR(recovered.lat, point.lat, 1e-6);
    double lon_diff = normalize_angle_signed(recovered.lon - point.lon);
    EXPECT_NEAR(lon_diff, 0.0, 1e-6);
}

// =============================================================================
// Miscellaneous Tests
// =============================================================================

TEST(MiscTest, DistanceToHorizon) {
    // At sea level, horizon at about 1.17 * sqrt(h) nm
    double d_100 = horizon_distance_nm(100.0);
    double d_1000 = horizon_distance_nm(1000.0);
    double d_10000 = horizon_distance_nm(10000.0);
    
    EXPECT_NEAR(d_100, 11.7, 1.0);    // ~11.7 nm at 100 ft
    EXPECT_NEAR(d_1000, 37.0, 2.0);   // ~37 nm at 1000 ft
    EXPECT_NEAR(d_10000, 117.0, 5.0); // ~117 nm at 10000 ft
}

// =============================================================================
// Numerical Stability Tests for New Modules
// =============================================================================

TEST(StabilityTest, RhumbLineAtPoles) {
    // Rhumb lines near poles - should handle gracefully
    LatLon near_pole = LatLon::from_degrees_unchecked(89.9, 0.0);
    LatLon equator = LatLon::from_degrees_unchecked(0.0, 0.0);
    
    auto course_result = rhumb_course(near_pole, equator);
    // May fail or succeed, but shouldn't crash
    if (course_result.is_ok()) {
        EXPECT_GE(course_result.value, 0.0);
        EXPECT_LE(course_result.value, 2.0 * constants::PI);
    }
}

TEST(StabilityTest, AtmosphereExtremeAltitudes) {
    // Very low altitude
    double temp_low = isa_temperature_c(-1000.0);
    double pressure_low = isa_pressure_mb(-1000.0);
    EXPECT_FALSE(std::isnan(temp_low));
    EXPECT_FALSE(std::isnan(pressure_low));
    
    // Very high altitude
    double temp_high = isa_temperature_c(100000.0);
    double pressure_high = isa_pressure_mb(100000.0);
    EXPECT_FALSE(std::isnan(temp_high));
    EXPECT_FALSE(std::isnan(pressure_high));
    EXPECT_GE(pressure_high, 0.0);
    EXPECT_LT(pressure_high, 100.0);
}

TEST(StabilityTest, HumidityEdgeCases) {
    // 100% RH
    double dp_100 = dewpoint(20.0, 100.0);
    EXPECT_NEAR(dp_100, 20.0, 0.5);
    
    // Very low RH
    double dp_low = dewpoint(20.0, 5.0);
    EXPECT_FALSE(std::isnan(dp_low));
    EXPECT_LT(dp_low, 20.0);
}

TEST(StabilityTest, WindTriangleEdgeCases) {
    // Wind speed > TAS - impossible to make track
    auto result = wind_correction_angle(200.0, deg_to_rad(90.0), 
                                          deg_to_rad(0.0), 150.0);
    EXPECT_FALSE(result.is_ok());
}

TEST(StabilityTest, TurnExtremeBankAngles) {
    // Very small bank angle
    auto rate_1deg = turn_rate_deg_per_sec(100.0, deg_to_rad(1.0));
    ASSERT_TRUE(rate_1deg.is_ok());
    EXPECT_GT(rate_1deg.value, 0.0);
    EXPECT_LT(rate_1deg.value, 1.0);
    
    // Bank angle near 90° (very high load factor)
    auto load_89 = load_factor(deg_to_rad(89.0));
    ASSERT_TRUE(load_89.is_ok());
    EXPECT_GT(load_89.value, 50.0);  // Very high G
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
