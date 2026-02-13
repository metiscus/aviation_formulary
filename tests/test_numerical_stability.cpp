#include <gtest/gtest.h>
#include "aviation_formulary/aviation_formulary.hpp"
#include <cmath>
#include <limits>

using namespace aviation;

// Numerical stability test suite
class NumericalStabilityTest : public ::testing::Test {
protected:
    static constexpr double EPSILON = std::numeric_limits<double>::epsilon();
    static constexpr double SMALL = 1e-10;
    
    // Helper to check if result is finite and not NaN
    bool is_valid_result(double value) {
        return std::isfinite(value) && !std::isnan(value);
    }
};

// Test 1: Very short distances (< 1 meter)
TEST_F(NumericalStabilityTest, VeryShortDistances) {
    LatLon p1 = LatLon::from_degrees_unchecked(45.0, 10.0);
    
    // Points 1 meter apart (~0.00001 degrees)
    LatLon p2 = LatLon::from_degrees_unchecked(45.00001, 10.0);
    
    double dist = distance(p1, p2);
    EXPECT_TRUE(is_valid_result(dist));
    EXPECT_GT(dist, 0.0);
    EXPECT_LT(dist, nm_to_rad(0.01)); // Less than 0.01 nm
    
    // Should not return NaN or Inf
    EXPECT_FALSE(std::isnan(dist));
    EXPECT_FALSE(std::isinf(dist));
}

// Test 2: Identical points (zero distance)
TEST_F(NumericalStabilityTest, IdenticalPoints) {
    LatLon p1 = LatLon::from_degrees_unchecked(45.0, 10.0);
    LatLon p2 = LatLon::from_degrees_unchecked(45.0, 10.0);
    
    double dist = distance(p1, p2);
    EXPECT_TRUE(is_valid_result(dist));
    EXPECT_NEAR(dist, 0.0, 1e-15);
    
    // Bearing of identical points should still be calculable (returns error for identical)
    auto bearing_result = initial_bearing(p1, p2);
    // For identical points, error is expected, but value should still be valid
    EXPECT_TRUE(is_valid_result(bearing_result.value));
}

// Test 3: Nearly antipodal points (opposite side of earth)
TEST_F(NumericalStabilityTest, NearlyAntipodalPoints) {
    LatLon p1 = LatLon::from_degrees_unchecked(0.0, 0.0);
    LatLon p2 = LatLon::from_degrees_unchecked(0.0, 179.9999);  // Almost 180° away
    
    double dist = distance(p1, p2);
    EXPECT_TRUE(is_valid_result(dist));
    EXPECT_GT(dist, deg_to_rad(170.0));
    EXPECT_LT(dist, constants::PI);
    
    // Bearing should still be valid
    auto bearing_result = initial_bearing(p1, p2);
    EXPECT_TRUE(is_valid_result(bearing_result.value));
}

// Test 4: Points very close to poles
TEST_F(NumericalStabilityTest, VeryCloseToNorthPole) {
    LatLon p1 = LatLon::from_degrees_unchecked(89.9999, 10.0);
    LatLon p2 = LatLon::from_degrees_unchecked(89.9999, 170.0);
    
    double dist = distance(p1, p2);
    EXPECT_TRUE(is_valid_result(dist));
    EXPECT_GT(dist, 0.0);
    
    auto bearing_result = initial_bearing(p1, p2);
    EXPECT_TRUE(is_valid_result(bearing_result.value));
}

// Test 5: Points very close to South Pole
TEST_F(NumericalStabilityTest, VeryCloseToSouthPole) {
    LatLon p1 = LatLon::from_degrees_unchecked(-89.9999, 10.0);
    LatLon p2 = LatLon::from_degrees_unchecked(-89.9999, 170.0);
    
    double dist = distance(p1, p2);
    EXPECT_TRUE(is_valid_result(dist));
    EXPECT_GT(dist, 0.0);
    
    auto bearing_result = initial_bearing(p1, p2);
    EXPECT_TRUE(is_valid_result(bearing_result.value));
}

// Test 6: Crossing equator at very small angles
TEST_F(NumericalStabilityTest, SmallAngleEquatorCrossing) {
    LatLon p1 = LatLon::from_degrees_unchecked(-0.0001, 10.0);
    LatLon p2 = LatLon::from_degrees_unchecked(0.0001, 10.0);
    
    double dist = distance(p1, p2);
    EXPECT_TRUE(is_valid_result(dist));
    
    auto bearing_result = initial_bearing(p1, p2);
    EXPECT_TRUE(is_valid_result(bearing_result.value));
}

// Test 7: Very small cross-track errors
TEST_F(NumericalStabilityTest, VerySmallCrossTrackError) {
    LatLon a = LatLon::from_degrees_unchecked(45.0, 10.0);
    LatLon b = LatLon::from_degrees_unchecked(45.0, 20.0);
    
    // Point almost exactly on the line
    LatLon d = LatLon::from_degrees_unchecked(45.00001, 15.0);
    
    double xtd = cross_track_distance(a, b, d);
    EXPECT_TRUE(is_valid_result(xtd));
    EXPECT_LT(std::abs(xtd), deg_to_rad(1.0));  // Less than 1 degree
}

// Test 8: Bearing at exactly 0, 90, 180, 270 degrees
TEST_F(NumericalStabilityTest, CardinalBearings) {
    LatLon center = LatLon::from_degrees_unchecked(45.0, 10.0);
    
    // Test all cardinal directions
    std::vector<double> bearings = {0.0, 90.0, 180.0, 270.0};
    
    for (double brng_deg : bearings) {
        double brng = deg_to_rad(brng_deg);
        double dist = nm_to_rad(100.0);
        
        LatLon dest = destination_point_unchecked(center, brng, dist);
        EXPECT_TRUE(is_valid_result(dest.lat));
        EXPECT_TRUE(is_valid_result(dest.lon));
    }
}

// Test 9: Very long distances
TEST_F(NumericalStabilityTest, VeryLongDistances) {
    LatLon p1 = LatLon::from_degrees_unchecked(45.0, 10.0);
    
    // Fly nearly around the world
    double bearing = deg_to_rad(90.0);
    double dist = deg_to_rad(350.0);  // 350 degrees
    
    LatLon dest = destination_point_unchecked(p1, bearing, dist);
    EXPECT_TRUE(is_valid_result(dest.lat));
    EXPECT_TRUE(is_valid_result(dest.lon));
    
    // Verify round-trip distance
    double dist_back = distance(p1, dest);
    EXPECT_TRUE(is_valid_result(dist_back));
}

// Test 10: Intermediate points at extremes (f = 0 and f = 1)
TEST_F(NumericalStabilityTest, IntermediatePointExtremes) {
    LatLon p1 = LatLon::from_degrees_unchecked(45.0, 10.0);
    LatLon p2 = LatLon::from_degrees_unchecked(50.0, 15.0);
    
    // f = 0 should return p1
    auto start_result = intermediate_point(p1, p2, 0.0);
    EXPECT_TRUE(start_result.is_ok());
    EXPECT_NEAR(start_result.value.lat, p1.lat, 1e-10);
    EXPECT_NEAR(start_result.value.lon, p1.lon, 1e-10);
    
    // f = 1 should return p2
    auto end_result = intermediate_point(p1, p2, 1.0);
    EXPECT_TRUE(end_result.is_ok());
    EXPECT_NEAR(end_result.value.lat, p2.lat, 1e-10);
    EXPECT_NEAR(end_result.value.lon, p2.lon, 1e-10);
    
    // Very small f
    auto tiny_result = intermediate_point(p1, p2, 1e-10);
    EXPECT_TRUE(tiny_result.is_ok());
    EXPECT_TRUE(is_valid_result(tiny_result.value.lat));
    EXPECT_TRUE(is_valid_result(tiny_result.value.lon));
    
    // f very close to 1
    auto almost_result = intermediate_point(p1, p2, 1.0 - 1e-10);
    EXPECT_TRUE(almost_result.is_ok());
    EXPECT_TRUE(is_valid_result(almost_result.value.lat));
    EXPECT_TRUE(is_valid_result(almost_result.value.lon));
}

// Test 11: Parallel great circles (no intersection)
TEST_F(NumericalStabilityTest, ParallelGreatCircles) {
    LatLon p1 = LatLon::from_degrees_unchecked(45.0, 10.0);
    LatLon p2 = LatLon::from_degrees_unchecked(46.0, 10.0);
    
    double bearing1 = deg_to_rad(90.0);
    double bearing2 = deg_to_rad(90.0);
    
    IntersectionResult result = intersection(p1, bearing1, p2, bearing2);
    
    // Should handle gracefully (may not exist or be ambiguous)
    if (result.exists) {
        EXPECT_TRUE(is_valid_result(result.point.lat));
        EXPECT_TRUE(is_valid_result(result.point.lon));
    }
}

// Test 12: Nearly parallel bearings (very acute intersection)
TEST_F(NumericalStabilityTest, NearlyParallelBearings) {
    LatLon p1 = LatLon::from_degrees_unchecked(45.0, 10.0);
    LatLon p2 = LatLon::from_degrees_unchecked(45.1, 10.0);
    
    double bearing1 = deg_to_rad(90.0);
    double bearing2 = deg_to_rad(90.01);  // Almost parallel
    
    IntersectionResult result = intersection(p1, bearing1, p2, bearing2);
    
    if (result.exists && !result.ambiguous) {
        EXPECT_TRUE(is_valid_result(result.point.lat));
        EXPECT_TRUE(is_valid_result(result.point.lon));
    }
}

// Test 13: Maximum latitude calculation stability
TEST_F(NumericalStabilityTest, MaxLatitudeStability) {
    LatLon p = LatLon::from_degrees_unchecked(45.0, 10.0);
    
    // Test all bearings
    for (int brng_deg = 0; brng_deg <= 360; brng_deg += 10) {
        double brng = deg_to_rad(brng_deg);
        double max_lat = max_latitude(p, brng);
        
        EXPECT_TRUE(is_valid_result(max_lat));
        EXPECT_GE(max_lat, 0.0);
        EXPECT_LE(max_lat, constants::HALF_PI);
    }
}

// Test 14: Along-track distance with very small XTD
TEST_F(NumericalStabilityTest, AlongTrackWithSmallXTD) {
    LatLon a = LatLon::from_degrees_unchecked(45.0, 10.0);
    LatLon b = LatLon::from_degrees_unchecked(45.0, 20.0);
    LatLon d = LatLon::from_degrees_unchecked(45.000001, 15.0);  // Almost on track
    
    double atd = along_track_distance(a, b, d);
    EXPECT_TRUE(is_valid_result(atd));
    EXPECT_GT(atd, 0.0);
}

// Test 15: Round-trip stability (go out and come back)
TEST_F(NumericalStabilityTest, RoundTripStability) {
    LatLon start = LatLon::from_degrees_unchecked(45.0, 10.0);
    
    for (int brng_deg = 0; brng_deg < 360; brng_deg += 30) {
        double brng = deg_to_rad(brng_deg);
        double dist = nm_to_rad(100.0);
        
        // Go out
        LatLon dest = destination_point_unchecked(start, brng, dist);
        
        // Come back
        double back_brng = initial_bearing_unchecked(dest, start);
        double back_dist = distance(dest, start);
        LatLon returned = destination_point_unchecked(dest, back_brng, back_dist);
        
        // Should return to starting point (within tolerance)
        EXPECT_NEAR(returned.lat, start.lat, 1e-10);
        
        // Compare longitudes accounting for wrap-around (normalize to [-PI, PI))
        double lon_diff = normalize_angle_signed(returned.lon - start.lon);
        EXPECT_NEAR(lon_diff, 0.0, 1e-10) << "Bearing: " << brng_deg << " degrees";
    }
}

// Test 16: Denormalized numbers handling
TEST_F(NumericalStabilityTest, DenormalizedNumbers) {
    LatLon p1 = LatLon::from_degrees_unchecked(45.0, 10.0);
    
    // Create a point with a denormalized distance away
    double tiny_dist = std::numeric_limits<double>::min() * 10.0;
    LatLon p2 = destination_point_unchecked(p1, deg_to_rad(90.0), tiny_dist);
    
    double dist = distance(p1, p2);
    EXPECT_TRUE(is_valid_result(dist));
    EXPECT_GE(dist, 0.0);
}

// Test 17: Crossing parallel at pole vicinity
TEST_F(NumericalStabilityTest, ParallelCrossingNearPole) {
    LatLon p1 = LatLon::from_degrees_unchecked(85.0, 10.0);
    LatLon p2 = LatLon::from_degrees_unchecked(85.0, 170.0);
    
    double parallel = deg_to_rad(88.0);
    
    ParallelCrossingResult result = crossing_parallels(p1, p2, parallel);
    
    if (result.num_crossings > 0) {
        EXPECT_TRUE(is_valid_result(result.lon1));
        EXPECT_TRUE(is_valid_result(result.lon2));
    }
}

// Test 18: Repeated calculations don't accumulate errors excessively
TEST_F(NumericalStabilityTest, ErrorAccumulation) {
    LatLon start = LatLon::from_degrees_unchecked(45.0, 10.0);
    LatLon current = start;
    
    double bearing = deg_to_rad(90.0);
    double step = nm_to_rad(10.0);
    
    // Take 100 steps
    for (int i = 0; i < 100; i++) {
        current = destination_point_unchecked(current, bearing, step);
    }
    
    // The total distance won't be exactly 1000 nm because we're following
    // constant bearings (which curve on a sphere), but it should be close
    double total_dist = distance(start, current);
    EXPECT_TRUE(is_valid_result(total_dist));
    EXPECT_GT(total_dist, nm_to_rad(900.0)); // At least 900 nm
    EXPECT_LT(total_dist, nm_to_rad(1100.0)); // At most 1100 nm
    
    // Verify latitude changed (we're not stuck at start)
    EXPECT_NE(current.lat, start.lat);
}

// Test 19: Extreme latitude differences
TEST_F(NumericalStabilityTest, ExtremeLatitudeDifferences) {
    LatLon p1 = LatLon::from_degrees_unchecked(89.9, 10.0);
    LatLon p2 = LatLon::from_degrees_unchecked(-89.9, 10.0);
    
    double dist = distance(p1, p2);
    EXPECT_TRUE(is_valid_result(dist));
    EXPECT_GT(dist, deg_to_rad(170.0));
    
    auto bearing_result = initial_bearing(p1, p2);
    EXPECT_TRUE(is_valid_result(bearing_result.value));
}

// Test 20: Very small angles in trigonometric functions
TEST_F(NumericalStabilityTest, SmallAngleTrigonometry) {
    // Test safe trig functions with values very close to domain boundaries
    EXPECT_TRUE(is_valid_result(asin_safe(1.0 + 1e-15)));
    EXPECT_TRUE(is_valid_result(asin_safe(-1.0 - 1e-15)));
    EXPECT_TRUE(is_valid_result(acos_safe(1.0 + 1e-15)));
    EXPECT_TRUE(is_valid_result(acos_safe(-1.0 - 1e-15)));
    
    // Values exactly at boundaries
    EXPECT_DOUBLE_EQ(asin_safe(1.0), constants::HALF_PI);
    EXPECT_DOUBLE_EQ(asin_safe(-1.0), -constants::HALF_PI);
    EXPECT_DOUBLE_EQ(acos_safe(1.0), 0.0);
    EXPECT_DOUBLE_EQ(acos_safe(-1.0), constants::PI);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
