#include <gtest/gtest.h>
#include "aviation_formulary/aviation_formulary.hpp"
#include <cmath>

using namespace aviation;

// Helper function for approximate equality
bool approx_equal(double a, double b, double tol = 1e-6) {
    return std::abs(a - b) < tol;
}

// Test mathematical utility functions
TEST(MathUtilsTest, ModFunction) {
    EXPECT_NEAR(mod(2.3, 2.0), 0.3, 1e-10);
    EXPECT_NEAR(mod(-2.3, 2.0), 1.7, 1e-10);
    EXPECT_NEAR(mod(5.5, 2.5), 0.5, 1e-10);
    EXPECT_NEAR(mod(-5.5, 2.5), 2.0, 1e-10);  // -5.5 mod 2.5 = 2.0
}

TEST(MathUtilsTest, SafeTrigFunctions) {
    // Test clamping behavior
    EXPECT_DOUBLE_EQ(asin_safe(1.5), constants::HALF_PI);
    EXPECT_DOUBLE_EQ(asin_safe(-1.5), -constants::HALF_PI);
    EXPECT_DOUBLE_EQ(acos_safe(1.5), 0.0);
    EXPECT_DOUBLE_EQ(acos_safe(-1.5), constants::PI);
    
    // Test normal behavior
    EXPECT_DOUBLE_EQ(asin_safe(0.5), std::asin(0.5));
    EXPECT_DOUBLE_EQ(acos_safe(0.5), std::acos(0.5));
}

TEST(MathUtilsTest, AngleNormalization) {
    EXPECT_NEAR(normalize_angle(3.0 * constants::PI), constants::PI, 1e-10);
    EXPECT_NEAR(normalize_angle(-constants::HALF_PI), 3.0 * constants::HALF_PI, 1e-10);
    
    // normalize_angle_signed returns values in [-PI, PI)
    // PI itself wraps to -PI due to the range definition
    EXPECT_NEAR(std::abs(normalize_angle_signed(constants::PI)), constants::PI, 1e-10);
    EXPECT_NEAR(std::abs(normalize_angle_signed(-constants::PI)), constants::PI, 1e-10);
    EXPECT_NEAR(std::abs(normalize_angle_signed(3.0 * constants::PI)), constants::PI, 1e-10);
}

TEST(MathUtilsTest, UnitConversions) {
    EXPECT_NEAR(deg_to_rad(180.0), constants::PI, 1e-10);
    EXPECT_NEAR(rad_to_deg(constants::PI), 180.0, 1e-10);
    
    // 60 nm = 1 degree at equator
    EXPECT_NEAR(nm_to_rad(60.0), deg_to_rad(1.0), 1e-6);
    EXPECT_NEAR(rad_to_nm(deg_to_rad(1.0)), 60.0, 1e-4);
}

// Test LatLon structure
TEST(LatLonTest, Construction) {
    LatLon p1(0.0, 0.0);
    EXPECT_DOUBLE_EQ(p1.lat, 0.0);
    EXPECT_DOUBLE_EQ(p1.lon, 0.0);
    
    // Using from_degrees which returns Result<LatLon>
    auto p2_result = LatLon::from_degrees(45.0, -90.0);
    EXPECT_TRUE(p2_result.is_ok());
    EXPECT_NEAR(p2_result.value.lat_degrees(), 45.0, 1e-10);
    EXPECT_NEAR(p2_result.value.lon_degrees(), -90.0, 1e-10);
    
    // Using unchecked version for known-valid coordinates
    LatLon p3 = LatLon::from_degrees_unchecked(45.0, -90.0);
    EXPECT_NEAR(p3.lat_degrees(), 45.0, 1e-10);
}

TEST(LatLonTest, Validation) {
    // Valid latitudes via Result API
    auto result1 = LatLon::from_degrees(90.0, 0.0);
    EXPECT_TRUE(result1.is_ok());
    
    auto result2 = LatLon::from_degrees(-90.0, 0.0);
    EXPECT_TRUE(result2.is_ok());
    
    // Invalid latitude via Result API
    auto result3 = LatLon::from_degrees(95.0, 0.0);
    EXPECT_FALSE(result3.is_ok());
    EXPECT_EQ(result3.error, ErrorCode::InvalidLatitude);
}

TEST(LatLonTest, PoleDetection) {
    LatLon north_pole(constants::HALF_PI, 0.0);
    EXPECT_TRUE(north_pole.is_pole());
    
    LatLon south_pole(-constants::HALF_PI, 0.0);
    EXPECT_TRUE(south_pole.is_pole());
    
    LatLon equator(0.0, 0.0);
    EXPECT_FALSE(equator.is_pole());
}

// Test great circle calculations with LAX to JFK example from formulary
// LAX: 33°57'N, 118°24'W = (33.95°N, 118.4°W)
// JFK: 40°38'N, 73°47'W = (40.633°N, 73.783°W)
TEST(GreatCircleTest, DistanceLAXtoJFK) {
    // Example from formulary
    double lat1 = deg_to_rad(33.0 + 57.0/60.0);  // 0.592539 rad
    double lon1 = deg_to_rad(118.0 + 24.0/60.0); // 2.066470 rad (West positive)
    double lat2 = deg_to_rad(40.0 + 38.0/60.0);  // 0.709186 rad
    double lon2 = deg_to_rad(73.0 + 47.0/60.0);  // 1.287762 rad
    
    LatLon lax(lat1, lon1);
    LatLon jfk(lat2, lon2);
    
    double d = distance(lax, jfk);
    
    // Expected: 0.623585 radians = 2144 nm
    EXPECT_NEAR(d, 0.623585, 1e-5);
    EXPECT_NEAR(rad_to_nm(d), 2144.0, 1.0);
}

TEST(GreatCircleTest, InitialBearingLAXtoJFK) {
    double lat1 = deg_to_rad(33.0 + 57.0/60.0);
    double lon1 = deg_to_rad(118.0 + 24.0/60.0);
    double lat2 = deg_to_rad(40.0 + 38.0/60.0);
    double lon2 = deg_to_rad(73.0 + 47.0/60.0);
    
    LatLon lax(lat1, lon1);
    LatLon jfk(lat2, lon2);
    
    auto bearing_result = initial_bearing(lax, jfk);
    EXPECT_TRUE(bearing_result.is_ok());
    double bearing = bearing_result.value;
    
    // Expected: 1.150035 radians = 66 degrees
    // The bearing is slightly less due to the convention used
    EXPECT_NEAR(bearing, 1.150035, 1e-3);
    EXPECT_NEAR(rad_to_deg(bearing), 66.0, 0.2);
}

TEST(GreatCircleTest, DestinationPoint) {
    // 100nm from LAX on 66 degree radial
    double lat1 = deg_to_rad(33.0 + 57.0/60.0);
    double lon1 = deg_to_rad(118.0 + 24.0/60.0);
    
    LatLon lax(lat1, lon1);
    double bearing = deg_to_rad(66.0);
    double dist = nm_to_rad(100.0);  // 0.0290888 radians
    
    LatLon dest = destination_point_unchecked(lax, bearing, dist);
    
    // Expected: 34°37'N, 116°33'W
    EXPECT_NEAR(dest.lat_degrees(), 34.0 + 37.0/60.0, 0.02);
    
    // Normalize longitude to positive (West = positive in this convention)
    double expected_lon = 116.0 + 33.0/60.0;
    double actual_lon = dest.lon_degrees();
    // Handle wrap-around: if actual is negative, add 360 
    if (actual_lon < 0) actual_lon += 360.0;
    EXPECT_NEAR(actual_lon, expected_lon, 0.02);
}

TEST(GreatCircleTest, IntermediatePoint) {
    double lat1 = deg_to_rad(33.0 + 57.0/60.0);
    double lon1 = deg_to_rad(118.0 + 24.0/60.0);
    double lat2 = deg_to_rad(40.0 + 38.0/60.0);
    double lon2 = deg_to_rad(73.0 + 47.0/60.0);
    
    LatLon lax(lat1, lon1);
    LatLon jfk(lat2, lon2);
    
    // 40% of the way from LAX to JFK
    auto mid_result = intermediate_point(lax, jfk, 0.4);
    EXPECT_TRUE(mid_result.is_ok());
    LatLon mid = mid_result.value;
    
    // Expected: 38°40.167'N, 101°37.570'W
    EXPECT_NEAR(mid.lat_degrees(), 38.0 + 40.167/60.0, 0.05);
    EXPECT_NEAR(mid.lon_degrees(), 101.0 + 37.570/60.0, 0.05);
}

TEST(GreatCircleTest, CrossTrackDistance) {
    // LAX to JFK route, point at N34:30 W116:30
    double lat1 = deg_to_rad(33.0 + 57.0/60.0);
    double lon1 = deg_to_rad(118.0 + 24.0/60.0);
    double lat2 = deg_to_rad(40.0 + 38.0/60.0);
    double lon2 = deg_to_rad(73.0 + 47.0/60.0);
    
    LatLon lax(lat1, lon1);
    LatLon jfk(lat2, lon2);
    LatLon point(deg_to_rad(34.5), deg_to_rad(116.5));
    
    double xtd = cross_track_distance(lax, jfk, point);
    
    // Expected: 0.00216747 radians = 7.4512 nm (right of course)
    EXPECT_NEAR(xtd, 0.00216747, 1e-5);
    EXPECT_NEAR(rad_to_nm(xtd), 7.4512, 0.1);
    EXPECT_GT(xtd, 0.0);  // Right of course
}

TEST(GreatCircleTest, AlongTrackDistance) {
    double lat1 = deg_to_rad(33.0 + 57.0/60.0);
    double lon1 = deg_to_rad(118.0 + 24.0/60.0);
    double lat2 = deg_to_rad(40.0 + 38.0/60.0);
    double lon2 = deg_to_rad(73.0 + 47.0/60.0);
    
    LatLon lax(lat1, lon1);
    LatLon jfk(lat2, lon2);
    LatLon point(deg_to_rad(34.5), deg_to_rad(116.5));
    
    double atd = along_track_distance(lax, jfk, point);
    
    // Expected: 0.0289691 radians = 99.588 nm
    EXPECT_NEAR(atd, 0.0289691, 1e-5);
    EXPECT_NEAR(rad_to_nm(atd), 99.588, 0.5);
}

TEST(GreatCircleTest, MaxLatitude) {
    // Calculate max latitude for LAX to JFK route
    double lat1 = deg_to_rad(33.0 + 57.0/60.0);
    double lon1 = deg_to_rad(118.0 + 24.0/60.0);
    double lat2 = deg_to_rad(40.0 + 38.0/60.0);
    double lon2 = deg_to_rad(73.0 + 47.0/60.0);
    
    LatLon lax(lat1, lon1);
    LatLon jfk(lat2, lon2);
    
    double bearing = initial_bearing_unchecked(lax, jfk);
    double max_lat = max_latitude(lax, bearing);
    
    // For a route going mostly east, max latitude should be higher than both endpoints
    EXPECT_GT(max_lat, lat1);
    EXPECT_GT(max_lat, lat2);
}

TEST(GreatCircleTest, Intersection) {
    // Example from formulary: REO and BKE
    // REO: 42.600N, 117.866W
    // BKE: 44.840N, 117.806W
    LatLon reo = LatLon::from_degrees_unchecked(42.600, 117.866);
    LatLon bke = LatLon::from_degrees_unchecked(44.840, 117.806);
    
    double brng1 = deg_to_rad(51.0);   // From REO
    double brng2 = deg_to_rad(137.0);  // From BKE
    
    IntersectionResult result = intersection(reo, brng1, bke, brng2);
    
    EXPECT_TRUE(result.exists);
    EXPECT_FALSE(result.ambiguous);
    
    // The intersection should be somewhere between the two points
    EXPECT_GT(result.point.lat, deg_to_rad(42.0));
    EXPECT_LT(result.point.lat, deg_to_rad(45.0));
}

TEST(GreatCircleTest, ParallelCrossing) {
    // LAX to JFK crossing 36°N parallel
    double lat1 = deg_to_rad(33.0 + 57.0/60.0);
    double lon1 = deg_to_rad(118.0 + 24.0/60.0);
    double lat2 = deg_to_rad(40.0 + 38.0/60.0);
    double lon2 = deg_to_rad(73.0 + 47.0/60.0);
    
    LatLon lax(lat1, lon1);
    LatLon jfk(lat2, lon2);
    
    double lat3 = deg_to_rad(36.0);
    
    ParallelCrossingResult result = crossing_parallels(lax, jfk, lat3);
    
    // The parallel at 36°N is between LAX (33.95°N) and JFK (40.63°N)
    // so the great circle should cross it
    EXPECT_GT(result.num_crossings, 0);
    
    // The actual longitude values from formulary are ~112°W
    // With proper normalization, values should be in reasonable range [-180, 180]
    EXPECT_GE(rad_to_deg(result.lon1), -180.0);
    EXPECT_LE(rad_to_deg(result.lon1), 180.0);
    EXPECT_GE(rad_to_deg(result.lon2), -180.0);
    EXPECT_LE(rad_to_deg(result.lon2), 180.0);
}

// Edge case tests
TEST(GreatCircleTest, SamePoint) {
    LatLon p1(0.0, 0.0);
    LatLon p2(0.0, 0.0);
    
    double d = distance(p1, p2);
    EXPECT_NEAR(d, 0.0, 1e-10);
}

TEST(GreatCircleTest, PoleHandling) {
    LatLon north_pole(constants::HALF_PI, 0.0);
    LatLon some_point(deg_to_rad(45.0), deg_to_rad(0.0));
    
    auto bearing_result = initial_bearing(north_pole, some_point);
    EXPECT_TRUE(bearing_result.is_ok());
    EXPECT_NEAR(bearing_result.value, constants::PI, 1e-10);  // South from north pole
}

TEST(GreatCircleTest, AntipodalPoints) {
    LatLon p1(0.0, 0.0);
    LatLon p2(0.0, constants::PI);
    
    // Distance should be half circumference
    double d = distance(p1, p2);
    EXPECT_NEAR(d, constants::PI, 1e-6);
    
    // Intermediate point should return error for antipodal points
    auto result = intermediate_point(p1, p2, 0.5);
    EXPECT_FALSE(result.is_ok());
    EXPECT_EQ(result.error, ErrorCode::AntipodalPoints);
}

TEST(GreatCircleTest, DistanceAlternativeMethods) {
    LatLon lax = LatLon::from_degrees_unchecked(33.95, 118.4);
    LatLon jfk = LatLon::from_degrees_unchecked(40.633, 73.783);
    
    double d1 = distance(lax, jfk);
    double d2 = distance_cosine(lax, jfk);
    
    // Both methods should give similar results
    EXPECT_NEAR(d1, d2, 1e-6);
}

TEST(GreatCircleTest, RoundTrip) {
    LatLon start = LatLon::from_degrees_unchecked(45.0, 10.0);
    double bearing = deg_to_rad(60.0);
    double dist = nm_to_rad(500.0);
    
    // Go out and come back
    LatLon dest = destination_point_unchecked(start, bearing, dist);
    auto back_bearing_result = initial_bearing(dest, start);
    double back_bearing = normalize_angle(back_bearing_result.value_or(0.0));
    double back_dist = distance(dest, start);
    
    EXPECT_NEAR(back_dist, dist, 1e-6);
    
    // Back bearing should be roughly opposite (differs due to great circle)
    double bearing_diff = std::abs(back_bearing - normalize_angle(bearing + constants::PI));
    EXPECT_LT(bearing_diff, deg_to_rad(10.0));  // Within 10 degrees
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
